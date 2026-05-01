#!/usr/bin/env python3
"""
Combined static-file server + local control-plane for the Stable-Bot GUI.

Replaces `python3 -m http.server 8080 --directory ...` — serves the same
HTML/JS, but also exposes POST endpoints the browser can hit when the
ROS stack is dead (i.e., when rosbridge itself is unreachable, so the
usual /control_cmd path doesn't work):

  POST /reset        → runs reset_stewart_stack.py (sync, ~3-5 s)
  POST /launch       → spawns `ros2 launch stewart_bringup stewart_gui_launch.py`
                        detached, with stdout/stderr to
                        ~/ros2_ws/src/stewart_bringup/logs/last_launch.log
  POST /stop_launch  → SIGINTs the launch subprocess we spawned
  GET  /launch_status → {"running": bool, "pid": N or null, "log_tail": "..."}

Everything listens on localhost only. No ROS sourcing is required to run
this (it shells out to /opt/ros/kilted/bin/ros2 via the reset script).

Usage:
  python3 gui_server.py
  python3 gui_server.py --port 8080 --web-dir ~/ros2_ws/src/stewart_bringup/web
"""
import argparse
import http.server
import json
import math
import os
import re
import signal
import socketserver
import subprocess
import sys
import threading
import time


def _finite_scrub(obj):
    """Recursively replace NaN/+Inf/-Inf floats with None so the result
    serializes as strict RFC 8259 JSON. Walks dicts, lists, tuples;
    leaves other types alone. Used at every JSON-write boundary
    because some SDO reads legitimately return non-finite values
    (e.g. vel_integrator_limit defaults to +inf for "no limit")."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: _finite_scrub(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_finite_scrub(v) for v in obj]
    return obj

def _find_stewart_bringup_dir():
    for cand in ('~/ros2_ws/src/stewart_bringup',
                 '~/ros2_ws/src/stable_bot/stewart_bringup'):
        p = os.path.expanduser(cand)
        if os.path.isfile(os.path.join(p, 'package.xml')):
            return p
    return os.path.expanduser('~/ros2_ws/src/stewart_bringup')
_BRINGUP_DIR = _find_stewart_bringup_dir()
DEFAULT_WEB_DIR = os.path.join(_BRINGUP_DIR, 'web')
RESET_SCRIPT = os.path.join(_BRINGUP_DIR, 'scripts/reset_stewart_stack.py')
LAUNCH_LOG = os.path.join(_BRINGUP_DIR, 'logs/last_launch.log')
BAGS_DIR = os.path.expanduser('~/stable_bot_bags')

# Global launch subprocess state (guarded by a lock)
_launch_lock = threading.Lock()
_launch_proc = None

# IMU-vs-Camera (IVA) bag recorder. Separate from the demo bag_recorder
# node — that one is wired to demo mode transitions; this one is a
# manual operator tool for verifying Stage C calibration accuracy by
# sweeping platform tilt and comparing IMU RPY to the camera-derived
# pose RPY. Bags land under tuning_data/ in the repo so they're easy
# to commit + push.
_iva_lock = threading.Lock()
_iva_proc = None
_iva_path = None
_iva_started_at = None

# Vision-debug bag recorder. Same pattern as IVA, different topic
# allowlist + a different folder name. Used to capture everything
# the depth-blob detector sees so we can replay why a misdetection
# happened in foxglove or rosbag2.
_vision_lock = threading.Lock()
_vision_proc = None
_vision_path = None
_vision_started_at = None

# Demo-run bag recorder (mirror IVA / vision pattern).
# Captures everything needed to evaluate a tuning attempt for
# Demo 1 (orbit), Demo 2 (goto), or Demo 3 (path):
# ball state vs reference, motor commands, platform pose, /status,
# /control_cmd. NO image streams — keeps bag size manageable.
_demo_lock = threading.Lock()
_demo_proc = None
_demo_path = None
_demo_started_at = None
_demo_label = None    # 'demo1', 'demo2', 'demo3', or 'untagged'
DEMO_TOPICS = [
    # Ball perception + reference
    '/ball_state',
    '/ball_ref',
    '/ball_xy_mono',
    '/ball_xy_depth',
    # BALL_TRACK loop's per-tick FSM diagnostic (phase, commanded
    # tilt, error/velocity quantities). Lets the digest color-code
    # trajectories by which phase the bang-bang controller was in.
    '/ball_track/diagnostic',
    '/oak/ball/v0/rgb_pixel',
    '/oak/ball/v0/diagnostic',
    '/oak/ball/depth/rgb_pixel',
    '/oak/ball/depth/diagnostic',
    # See-to-Pi latency (capture→host wall-clock from the OAK driver).
    # Lets digest_demo_bag plot the actual vision lag the controller
    # is fighting — independent of any control_latency_s constant.
    '/oak/latency_ms',
    # Per-stream Hz + per-path latency snapshot every 5 s. Same data
    # the [health] log line carries, but in the bag so the digest can
    # plot it over the run.
    '/oak/health',
    # OAK-side tunables (rgb_fps, jpeg quality, depth on/off, …) so
    # the digest can correlate behavior changes with config tweaks.
    '/oak/config',
    # Pose ground truth
    '/platform_pose',
    '/platform_pose/markers_visible',
    '/platform_rpy',
    '/platform/imu/data',
    # Operator + node IO
    '/control_cmd',
    '/control_result',
    '/status',
    # Motor state (so post-run analysis can tie tracking error to
    # actuator current/encoder behavior)
    '/leg_encoders',
    '/leg_currents',
    '/odrive_errors',
]
VISION_DEBUG_TOPICS = [
    # Camera streams
    '/oak/rgb/image_compressed',
    '/oak/depth_blob/debug_image',
    '/oak/latency_ms',
    # Detector outputs
    '/oak/ball/v0/rgb_pixel',
    '/oak/ball/v0/diagnostic',
    '/oak/ball/depth/rgb_pixel',
    '/oak/ball/depth/diagnostic',
    '/oak/ball/spatial',
    # Localizer outputs (truth-like, in platform frame mm)
    '/ball_xy_mono',
    '/ball_xy_depth',
    '/ball_xy_oak',
    # Pose ground truth + ArUco diagnostic
    '/platform_pose',
    '/platform_pose/markers_visible',
    '/platform_rpy',
    '/platform/imu/data',
    # What the operator was doing during the recording
    '/control_cmd',
    '/control_result',
    '/status',
]

# Where IVA bags live on disk. Resolved to the in-repo tuning_data/
# so a single git commit ships the dataset.
def _iva_bags_root():
    for cand in ('~/stable_bot_repo/tuning_data',
                 '~/ros2_ws/src/stewart_bringup/../tuning_data'):
        p = os.path.expanduser(cand)
        if os.path.isdir(os.path.dirname(p)):
            return p
    return os.path.expanduser('~/stable_bot_bags')
IVA_TOPICS = [
    # Pose truth + vision pose (the comparison subjects)
    '/platform_rpy',
    '/platform_pose',
    '/platform_pose/markers_visible',
    # Raw IMU for offline deeper-look (acceleration / angular vel)
    '/platform/imu/data',
    # Operator commands + control-node replies — needed to know what
    # tilt was commanded at each timestamp during a sweep
    '/control_cmd',
    '/control_result',
    # Motor state — same level of detail as level_pi tuning bags so
    # post-run analysis can correlate vision residuals against
    # what the actuators were actually doing
    '/leg_encoders',
    '/leg_currents',
    '/status',
    '/odrive_errors',
]
# Default bag location: in-repo so a commit is one step.
def _iva_default_dir():
    return _iva_bags_root()


def _iva_dir_size(path):
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for fn in files:
                try:
                    total += os.path.getsize(os.path.join(root, fn))
                except OSError:
                    pass
    except Exception:
        pass
    return total


def _list_iva_bags():
    """Enumerate IVA sweep bags under the tuning_data/ root.
    Returns a list of dicts ordered by mtime descending. Each entry
    flags whether digest.png and digest.summary.json are present
    so the GUI can show which bags have already been analyzed.
    """
    root = _iva_bags_root()
    out = []
    if not os.path.isdir(root):
        return out
    try:
        entries = os.listdir(root)
    except OSError:
        return out
    for name in entries:
        # Only IVA bag directories — written under
        # tuning_data/<UTC>Z_imu_vs_camera/.
        if 'imu_vs_camera' not in name:
            continue
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        png = os.path.join(full, 'digest.png')
        summary = os.path.join(full, 'digest.summary.json')
        entry = {
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
            'has_png': os.path.isfile(png),
            'has_summary': os.path.isfile(summary),
        }
        # If a digest summary exists, surface the headline residual stats
        # so the list can render them inline without an extra round-trip.
        if entry['has_summary']:
            try:
                with open(summary) as f:
                    s = json.load(f)
                rd = s.get('residual_deg', {})
                entry['residual'] = {
                    axis: {
                        'p95': (rd.get(axis) or {}).get('p95'),
                        'std': (rd.get(axis) or {}).get('std'),
                        'max_abs': (rd.get(axis) or {}).get('max_abs'),
                    } for axis in ('roll', 'pitch', 'yaw')
                }
                entry['duration_s'] = s.get('duration_s')
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _list_auto_tune_sessions():
    """Enumerate auto_tune sessions under tuning_data/. Mirrors
    _list_iva_bags' shape so the GUI can reuse the same render
    pattern: one row per session with Digest / Push / Delete
    actions + an inline summary of best fitness."""
    root = _iva_bags_root()
    out = []
    if not os.path.isdir(root):
        return out
    try:
        entries = os.listdir(root)
    except OSError:
        return out
    for name in entries:
        if not name.startswith('auto_tune_'):
            continue
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        png = os.path.join(full, 'fitness_curve.png')
        summary = os.path.join(full, 'summary.json')
        digest_summary = os.path.join(full, 'digest.summary.json')
        log = os.path.join(full, 'log.jsonl')
        bag = os.path.join(full, 'bag')
        entry = {
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
            'has_png': os.path.isfile(png),
            'has_summary': os.path.isfile(summary),
            'has_digest': os.path.isfile(digest_summary),
            'has_bag': os.path.isdir(bag),
            'has_log': os.path.isfile(log),
        }
        # Inline best-fitness so the list shows it without a click.
        if os.path.isfile(summary):
            try:
                with open(summary) as f:
                    sd = json.load(f) or {}
                entry['n_trials'] = sd.get('n_trials')
                entry['best_fitness'] = sd.get('best_fitness')
                entry['elapsed_s'] = sd.get('elapsed_s')
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _resolve_auto_tune_session(name):
    """Path-traversal guard for auto_tune session names."""
    if not name or '/' in name or name in ('.', '..'):
        return None
    if not name.startswith('auto_tune_'):
        return None
    root = os.path.realpath(_iva_bags_root())
    full = os.path.realpath(os.path.join(root, name))
    if not full.startswith(root + os.sep) and full != root:
        return None
    if not os.path.isdir(full):
        return None
    return full


def _list_step_id_sessions():
    """Enumerate step_id_<UTC>/ sessions under tuning_data/.
    Mirrors _list_auto_tune_sessions; the inline summary surfaces the
    open-loop G_eff and recommended ωn so operators can compare runs
    at a glance without opening the digest PNG."""
    root = _iva_bags_root()
    out = []
    if not os.path.isdir(root):
        return out
    try:
        entries = os.listdir(root)
    except OSError:
        return out
    for name in entries:
        if not name.startswith('step_id_'):
            continue
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        png = os.path.join(full, 'plant_gain_fit.png')
        summary = os.path.join(full, 'summary.json')
        digest_summary = os.path.join(full, 'step_id_summary.json')
        recommendation = os.path.join(full, 'step_id_recommendation.json')
        bag = os.path.join(full, 'bag')
        entry = {
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
            'has_png': os.path.isfile(png),
            'has_summary': os.path.isfile(summary),
            'has_digest': os.path.isfile(digest_summary),
            'has_recommendation': os.path.isfile(recommendation),
            'has_bag': os.path.isdir(bag),
        }
        # Inline open-loop G_eff and recommended ωn so the list shows
        # those without a click — same UX as best_fitness for
        # auto_tune sessions.
        if os.path.isfile(summary):
            try:
                with open(summary) as f:
                    sd = json.load(f) or {}
                rec = (sd.get('phases', {}) or {}).get('recommendation', {})
                ol = (sd.get('phases', {}) or {}).get('open_loop', {})
                entry['g_eff_used'] = rec.get('g_eff_used')
                entry['omega_n_rad_s'] = rec.get('omega_n_rad_s')
                entry['kp'] = rec.get('kp')
                entry['kd'] = rec.get('kd')
                entry['start_marker'] = sd.get('start_marker')
                entry['td_observed_s'] = ol.get('td_observed_s')
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _resolve_step_id_session(name):
    """Path-traversal guard for step_id session names."""
    if not name or '/' in name or name in ('.', '..'):
        return None
    if not name.startswith('step_id_'):
        return None
    root = os.path.realpath(_iva_bags_root())
    full = os.path.realpath(os.path.join(root, name))
    if not full.startswith(root + os.sep) and full != root:
        return None
    if not os.path.isdir(full):
        return None
    return full


def _resolve_iva_bag_path(name):
    """Path-traversal guard. Resolves a basename under the IVA root."""
    if not name or '/' in name or name in ('.', '..'):
        return None
    root = os.path.realpath(_iva_bags_root())
    full = os.path.realpath(os.path.join(root, name))
    if not full.startswith(root + os.sep) and full != root:
        return None
    if not os.path.isdir(full):
        return None
    return full


def _digest_iva_bag(name):
    """Run digest_iva_bag.py against the named bag.
    Output PNG + JSON land in the bag dir."""
    full = _resolve_iva_bag_path(name)
    if full is None:
        return False, "bad name"
    analyzer = os.path.join(_BRINGUP_DIR, 'scripts/digest_iva_bag.py')
    if not os.path.isfile(analyzer):
        return False, f"analyzer not found at {analyzer}"
    cmd = (
        'source /opt/ros/kilted/setup.bash && '
        'source ~/ros2_ws/install/local_setup.bash && '
        f'python3 {analyzer!r} {full!r}'
    )
    try:
        r = subprocess.run(['bash', '-c', cmd],
                           capture_output=True, text=True, timeout=180)
        ok = (r.returncode == 0)
        out = (r.stdout or '').strip()
        err = (r.stderr or '').strip()
        return ok, (out + ('\n' + err if err else ''))[-4000:]
    except subprocess.TimeoutExpired:
        return False, "digest timed out after 180 s"
    except Exception as e:
        return False, f"digest failed: {e}"


def _delete_iva_bag(name):
    """Delete a single IVA bag dir. shutil.rmtree under the path-
    traversal guard."""
    import shutil
    full = _resolve_iva_bag_path(name)
    if full is None:
        return False, "bad name"
    try:
        shutil.rmtree(full)
        return True, f"deleted {name}"
    except Exception as e:
        return False, f"delete failed: {e}"


def _iva_bag_png_path(name):
    """Path to the digest.png inside a named bag, or None."""
    full = _resolve_iva_bag_path(name)
    if full is None:
        return None
    p = os.path.join(full, 'digest.png')
    return p if os.path.isfile(p) else None


def _apply_iva_alignment(name):
    """Copy a named IVA bag's aruco_imu_alignment.yaml into the live
    config locations so ball_localizer picks it up on next restart.
    Writes both the in-repo path (so a future colcon build doesn't
    lose it) and the install share dir (so the running node sees it
    after a service restart, before any rebuild)."""
    bag_dir = _resolve_iva_bag_path(name)
    if bag_dir is None:
        return False, f'no such IVA bag: {name}'
    src = os.path.join(bag_dir, 'aruco_imu_alignment.yaml')
    if not os.path.isfile(src):
        return False, ('no aruco_imu_alignment.yaml in this bag — '
                       'either the IVA sweep was too static to '
                       'solve alignment (need ≥0.2° spread on both '
                       'axes), or the digest predates the alignment '
                       'export. Re-run digest on a varied sweep.')
    repo_dst = os.path.expanduser(
        '~/stable_bot_repo/stewart_vision/config/aruco_imu_alignment.yaml')
    install_dst = os.path.expanduser(
        '~/ros2_ws/install/stewart_vision/share/'
        'stewart_vision/config/aruco_imu_alignment.yaml')
    out_lines = [f'source: {src}']
    import shutil
    for dst in (repo_dst, install_dst):
        try:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            out_lines.append(f'wrote: {dst}')
        except Exception as e:
            out_lines.append(f'FAILED {dst}: {e}')
    out_lines.append(
        'restart stable_bot.service so ball_localizer reloads the '
        'alignment')
    return True, '\n'.join(out_lines)


def _git_push_iva_bag(name):
    """git add tuning_data/<name> && git commit && git push, with the
    auto-generated commit message embedding the digest summary's
    headline residual stats if a digest has been run.

    Path-traversal guarded via _resolve_iva_bag_path. Runs from the
    repo root, fails (without partial commits) if any step errors.
    """
    full = _resolve_iva_bag_path(name)
    if full is None:
        return False, "bad name"

    # Best-effort: try to find the repo. _iva_bags_root resolves to
    # tuning_data/ inside the repo, so the parent of that is the repo.
    repo_dir = os.path.dirname(_iva_bags_root())
    if not os.path.isdir(os.path.join(repo_dir, '.git')):
        return False, (
            f"not a git repo: {repo_dir}. Configure ~/stable_bot_repo "
            f"as the repo root or use the CLI workflow.")

    # Build a default commit message. If a digest summary is on disk,
    # embed the per-axis p95 residuals so the GitHub feed is grep-able
    # at a glance.
    msg = f"IVA sweep {name}"
    summary_path = os.path.join(full, 'digest.summary.json')
    if os.path.isfile(summary_path):
        try:
            with open(summary_path) as f:
                s = json.load(f)
            rd = s.get('residual_deg') or {}
            parts = []
            for axis in ('roll', 'pitch', 'yaw'):
                p95 = (rd.get(axis) or {}).get('p95')
                if p95 is not None:
                    parts.append(f"{axis} p95 {p95:.2f}°")
            if parts:
                msg += " — Δ " + ', '.join(parts)
        except Exception:
            pass

    rel_bag = os.path.relpath(full, repo_dir)
    out_lines = []

    def _run(cmd):
        try:
            r = subprocess.run(cmd, cwd=repo_dir,
                               capture_output=True, text=True,
                               timeout=120)
            out_lines.append(f"$ {' '.join(cmd)}")
            if r.stdout.strip():
                out_lines.append(r.stdout.strip())
            if r.stderr.strip():
                out_lines.append(r.stderr.strip())
            return r.returncode == 0
        except Exception as e:
            out_lines.append(f"FAILED: {' '.join(cmd)}: {e}")
            return False

    if not _run(['git', 'add', rel_bag]):
        return False, '\n'.join(out_lines)

    # Skip commit if nothing was actually staged (re-pushing same bag).
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'],
                          cwd=repo_dir)
    nothing_to_commit = (diff.returncode == 0)
    if nothing_to_commit:
        out_lines.append('(nothing to commit — bag already in HEAD)')
    else:
        if not _run(['git', 'commit', '-m', msg]):
            return False, '\n'.join(out_lines)
    # Pull --rebase before pushing so a concurrent push from elsewhere
    # (laptop, another GUI session) doesn't reject this push with
    # "fetch first." The /auto_tune/bags/push and /step_id/bags/push
    # handlers do this; the IVA push handler used to skip it and
    # surfaced a confusing "rejected" message to the operator any
    # time a concurrent push had landed (observed 2026-05-01 when an
    # IVA sweep collided with a code commit pushed seconds earlier).
    if not _run(['git', 'pull', '--rebase']):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append(
        '✓ pushed' if nothing_to_commit else '✓ pushed to origin')
    return True, '\n'.join(out_lines)


def _iva_start():
    """Spawn `ros2 bag record` as a subprocess. Returns
    (ok, msg, path).
    """
    global _iva_proc, _iva_path, _iva_started_at
    with _iva_lock:
        if _iva_proc is not None and _iva_proc.poll() is None:
            return False, 'already recording', _iva_path
        ts = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        out_dir = os.path.join(_iva_default_dir(),
                               f'{ts}_imu_vs_camera')
        os.makedirs(os.path.dirname(out_dir), exist_ok=True)
        # Source ROS so `ros2 bag record` resolves. The systemd
        # service for stable_bot already sources Kilted; we mirror
        # that here for robustness when launched outside systemd.
        cmd = ('source /opt/ros/kilted/setup.bash && '
               'source /home/sorak/ros2_ws/install/local_setup.bash && '
               f'exec ros2 bag record -s mcap -o {out_dir} '
               + ' '.join(IVA_TOPICS))
        try:
            _iva_proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            _iva_path = out_dir
            _iva_started_at = time.time()
            return True, f'recording → {out_dir}', out_dir
        except Exception as e:
            _iva_proc = None
            _iva_path = None
            _iva_started_at = None
            return False, f'failed to spawn: {e}', None


def _iva_stop():
    """SIGINT the recorder so rosbag2 flushes cleanly. Returns
    (ok, msg, path).
    """
    global _iva_proc, _iva_path, _iva_started_at
    with _iva_lock:
        if _iva_proc is None:
            return False, 'not recording', _iva_path
        if _iva_proc.poll() is not None:
            # Process exited on its own; just clear state.
            path = _iva_path
            _iva_proc = None
            _iva_path = None
            _iva_started_at = None
            return True, f'recorder already exited; bag at {path}', path
        try:
            os.killpg(os.getpgid(_iva_proc.pid), signal.SIGINT)
        except Exception as e:
            return False, f'SIGINT failed: {e}', _iva_path
        try:
            _iva_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(_iva_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        path = _iva_path
        _iva_proc = None
        _iva_path = None
        _iva_started_at = None
        return True, f'saved → {path}', path


def _iva_status():
    """Return a dict describing the recorder state."""
    with _iva_lock:
        running = (_iva_proc is not None and _iva_proc.poll() is None)
        elapsed = (time.time() - _iva_started_at) \
            if (_iva_started_at and running) else None
        return {
            'recording': running,
            'path': _iva_path,
            'elapsed_s': elapsed,
        }


# ---- Vision-debug bag recorder (mirrors IVA, different topic set) ---------

def _vision_default_dir():
    return _iva_bags_root()


def _list_vision_bags():
    """Enumerate vision-debug bags. Same shape as _list_iva_bags so
    the GUI can render them with shared helpers."""
    root = _iva_bags_root()
    out = []
    if not os.path.isdir(root):
        return out
    try:
        entries = os.listdir(root)
    except OSError:
        return out
    for name in entries:
        if 'vision_debug' not in name:
            continue
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        png = os.path.join(full, 'digest.png')
        js  = os.path.join(full, 'digest.summary.json')
        entry = {
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
            'has_png': os.path.isfile(png),
            'has_summary': os.path.isfile(js),
        }
        if entry['has_summary']:
            try:
                with open(js) as f:
                    s = json.load(f)
                v0 = s.get('v0', {})
                dp = s.get('depth_blob', {})
                po = (dp.get('plane_offset_mm') or {})
                md = ((s.get('platform_frame_mm') or {}).get(
                    'detector_disagreement_mm') or {})
                entry['summary'] = {
                    'duration_s': s.get('duration_s'),
                    'v0_n': v0.get('detections_n'),
                    'depth_n': dp.get('detections_n'),
                    'depth_rate_hz': dp.get('detection_rate_hz_mean'),
                    'plane_offset_mean_mm': po.get('mean'),
                    'plane_offset_max_abs_mm': po.get('max_abs'),
                    'v0_vs_depth_p95_mm': md.get('p95'),
                }
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _resolve_vision_bag_path(name):
    """Map a bag name (basename only, no slashes) to its absolute
    directory under the vision-debug root."""
    if '/' in name or '\\' in name or name.startswith('.'):
        return None
    full = os.path.realpath(os.path.join(_iva_bags_root(), name))
    root = os.path.realpath(_iva_bags_root())
    if not full.startswith(root + os.sep):
        return None
    if not os.path.isdir(full):
        return None
    return full


def _vision_start():
    """Spawn `ros2 bag record` for the vision-debug topic set."""
    global _vision_proc, _vision_path, _vision_started_at
    with _vision_lock:
        if _vision_proc is not None and _vision_proc.poll() is None:
            return False, 'already recording', _vision_path
        ts = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        out_dir = os.path.join(_vision_default_dir(),
                               f'{ts}_vision_debug')
        os.makedirs(os.path.dirname(out_dir), exist_ok=True)
        cmd = ('source /opt/ros/kilted/setup.bash && '
               'source /home/sorak/ros2_ws/install/local_setup.bash && '
               f'exec ros2 bag record -s mcap -o {out_dir} '
               + ' '.join(VISION_DEBUG_TOPICS))
        try:
            _vision_proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            _vision_path = out_dir
            _vision_started_at = time.time()
            return True, f'recording → {out_dir}', out_dir
        except Exception as e:
            _vision_proc = None
            _vision_path = None
            _vision_started_at = None
            return False, f'failed to spawn: {e}', None


def _vision_stop():
    """SIGINT the recorder so rosbag2 flushes cleanly."""
    global _vision_proc, _vision_path, _vision_started_at
    with _vision_lock:
        if _vision_proc is None:
            return False, 'not recording', _vision_path
        if _vision_proc.poll() is not None:
            path = _vision_path
            _vision_proc = None
            _vision_path = None
            _vision_started_at = None
            return True, f'recorder already exited; bag at {path}', path
        try:
            os.killpg(os.getpgid(_vision_proc.pid), signal.SIGINT)
        except Exception as e:
            return False, f'SIGINT failed: {e}', _vision_path
        try:
            _vision_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(_vision_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        path = _vision_path
        _vision_proc = None
        _vision_path = None
        _vision_started_at = None
        return True, f'saved → {path}', path


def _vision_status():
    with _vision_lock:
        running = (_vision_proc is not None
                   and _vision_proc.poll() is None)
        elapsed = (time.time() - _vision_started_at) \
            if (_vision_started_at and running) else None
        return {
            'recording': running,
            'path': _vision_path,
            'elapsed_s': elapsed,
        }


def _push_ball_track_gains_to_git():
    """git add stewart_bringup/config/ball_track_gains.yaml +
    commit + pull --rebase + push. Auto-generates a commit
    message embedding the headline gains so git log is
    grep-able for "tuning at Kp=0.030 Kd=0.040".

    Used by the Gains & Control Parameters panel's Push button —
    captures the live tuned gains in version control so a
    subsequent tuning round can revert via `git checkout` if it
    goes worse.
    """
    repo_dir = os.path.dirname(_iva_bags_root())  # ~/stable_bot_repo
    gains_rel = 'stewart_bringup/config/ball_track_gains.yaml'
    gains_path = os.path.join(repo_dir, gains_rel)
    if not os.path.isfile(gains_path):
        return False, f'no file at {gains_path}'
    if not os.path.isdir(os.path.join(repo_dir, '.git')):
        return False, f'not a git repo: {repo_dir}'
    # Read the headline gains for the commit message.
    try:
        import yaml as _yaml
        with open(gains_path) as f:
            g = _yaml.safe_load(f) or {}
        kp = g.get('kp', '?')
        kd = g.get('kd', '?')
        ki = g.get('ki', '?')
        mt = g.get('max_tilt_deg', '?')
        algo = g.get('algorithm', 'pid')
        msg = (f'Tuning: ball_track {algo} '
               f'Kp={kp} Kd={kd} Ki={ki} max_tilt={mt}°')
    except Exception:
        msg = 'Tuning: ball_track_gains.yaml update'
    out_lines = []

    def _run(cmd):
        try:
            r = subprocess.run(cmd, cwd=repo_dir,
                               capture_output=True, text=True,
                               timeout=60)
            line = ' '.join(cmd)
            out_lines.append(f'$ {line}')
            if r.stdout.strip():
                out_lines.append(r.stdout.strip())
            if r.stderr.strip():
                out_lines.append(r.stderr.strip())
            return r.returncode == 0
        except Exception as e:
            out_lines.append(f'FAILED: {" ".join(cmd)}: {e}')
            return False

    if not _run(['git', 'add', gains_rel]):
        return False, '\n'.join(out_lines)
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'],
                          cwd=repo_dir)
    nothing_to_commit = (diff.returncode == 0)
    if nothing_to_commit:
        out_lines.append('(yaml unchanged from HEAD — nothing to commit)')
    else:
        if not _run(['git', 'commit', '-m', msg]):
            return False, '\n'.join(out_lines)
    # Pull --rebase before push to handle concurrent pushes
    # (same race-handling as the bag push helpers).
    if not _run(['git', 'pull', '--rebase']):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append(
        '✓ pushed' if nothing_to_commit
        else f'✓ pushed: {msg}')
    return True, '\n'.join(out_lines)


def _push_vision_bag_to_git(name):
    """Commit + push the DIGEST artifacts only — digest.png and
    digest.summary.json. The raw .mcap (typically tens of MB once
    RGB and the depth-blob debug overlay are flowing) stays on the
    Pi. Calls _digest_vision_bag first if no digest exists yet so
    the user can hit one button instead of two."""
    bag_dir = _resolve_vision_bag_path(name)
    if bag_dir is None:
        return False, f'no such vision bag: {name}'
    repo_dir = os.path.dirname(_iva_bags_root())
    if not os.path.isdir(os.path.join(repo_dir, '.git')):
        return False, f'{repo_dir} is not a git repo'

    out_lines = []

    # Auto-digest if missing.
    png = os.path.join(bag_dir, 'digest.png')
    js  = os.path.join(bag_dir, 'digest.summary.json')
    if not (os.path.isfile(png) and os.path.isfile(js)):
        out_lines.append('(running digest first)')
        ok, msg = _digest_vision_bag(name)
        out_lines.append(msg or '')
        if not ok:
            return False, '\n'.join(out_lines)

    rel_png = os.path.relpath(png, repo_dir)
    rel_js  = os.path.relpath(js,  repo_dir)

    # Headline stats for the commit message body.
    headline = ''
    try:
        with open(js) as f:
            s = json.load(f)
        v0 = s.get('v0', {})
        dp = s.get('depth_blob', {})
        po = (dp.get('plane_offset_mm') or {})
        md = ((s.get('platform_frame_mm') or {}).get(
            'detector_disagreement_mm') or {})
        headline = (
            f"\nduration={s.get('duration_s', 0):.1f}s "
            f"V0={v0.get('detections_n', 0)} "
            f"depth={dp.get('detections_n', 0)}\n"
            f"depth-rate mean={dp.get('detection_rate_hz_mean', 0):.1f}Hz "
            f"max-silence={dp.get('largest_silence_s', 0):.1f}s\n"
            f"plane_offset mean={po.get('mean', 0):+.1f}mm "
            f"max|·|={po.get('max_abs', 0):.1f}mm\n"
            f"V0↔depth p95={md.get('p95', 0):.1f}mm")
    except Exception:
        pass

    msg = (f'Vision-debug digest {name}\n'
           + headline + '\n\n'
           + 'Digest only (digest.png + digest.summary.json). Raw '
             'rosbag2 mcap stays on the Pi — \n'
             'too large to commit per-bag (RGB + depth-blob overlay '
             'is tens of MB).\n')

    def _run(cmd):
        try:
            r = subprocess.run(cmd, cwd=repo_dir,
                               capture_output=True, text=True,
                               timeout=120)
            line = ' '.join(cmd)
            if r.returncode != 0:
                out_lines.append(f"FAILED: {line}\n{r.stderr}")
            else:
                out_lines.append(f"OK: {line}")
                if r.stdout.strip():
                    out_lines.append(r.stdout.strip())
            return r.returncode == 0
        except Exception as e:
            out_lines.append(f"FAILED: {' '.join(cmd)}: {e}")
            return False

    if not _run(['git', 'add', '-f', rel_png, rel_js]):
        return False, '\n'.join(out_lines)
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'],
                          cwd=repo_dir)
    nothing_to_commit = (diff.returncode == 0)
    if nothing_to_commit:
        out_lines.append('(nothing to commit — digest already in HEAD)')
    else:
        if not _run(['git', 'commit', '-m', msg]):
            return False, '\n'.join(out_lines)
    # Pull --rebase before push to handle concurrent pushes from
    # other sources (laptop, other GUI sessions). Without this, a
    # "fetch first" rejection surfaces as a confusing GUI error
    # any time someone else has pushed in the seconds before this
    # call (observed 2026-05-01 with an IVA-sweep collision).
    if not _run(['git', 'pull', '--rebase']):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append(
        '✓ pushed' if nothing_to_commit
        else '✓ pushed to origin (digest only)')
    return True, '\n'.join(out_lines)


def _delete_vision_bag(name):
    """Remove a vision-debug bag directory. Mirrors _delete_iva_bag."""
    bag_dir = _resolve_vision_bag_path(name)
    if bag_dir is None:
        return False, f'no such vision bag: {name}'
    try:
        import shutil
        shutil.rmtree(bag_dir)
    except Exception as e:
        return False, f'rmtree failed: {e}'
    return True, f'deleted {bag_dir}'


def _digest_vision_bag(name):
    """Run digest_vision_bag.py against the named bag. Writes
    digest.png + digest.summary.json into the bag directory."""
    full = _resolve_vision_bag_path(name)
    if full is None:
        return False, "bad name"
    analyzer = os.path.join(_BRINGUP_DIR, 'scripts/digest_vision_bag.py')
    if not os.path.isfile(analyzer):
        return False, f"analyzer not found at {analyzer}"
    cmd = (
        'source /opt/ros/kilted/setup.bash && '
        'source ~/ros2_ws/install/local_setup.bash && '
        f'python3 {analyzer!r} {full!r}'
    )
    try:
        r = subprocess.run(['bash', '-c', cmd],
                           capture_output=True, text=True, timeout=180)
        ok = (r.returncode == 0)
        out = (r.stdout or '').strip()
        err = (r.stderr or '').strip()
        return ok, (out + ('\n' + err if err else ''))[-4000:]
    except subprocess.TimeoutExpired:
        return False, "digest timed out after 180 s"
    except Exception as e:
        return False, f"digest failed: {e}"


def _vision_bag_png_path(name):
    """Path to digest.png inside a named vision-debug bag, or None."""
    full = _resolve_vision_bag_path(name)
    if full is None:
        return None
    p = os.path.join(full, 'digest.png')
    return p if os.path.isfile(p) else None


# ---- Demo-run bag recorder (Demos 1 / 2 / 3 tuning runs) ----

def _list_demo_bags():
    """Enumerate demo-run bags. Bag dirs are named
    <UTC>Z_demo_<label>, where label is 'demo1', 'demo2', etc.
    Returns dicts shaped like _list_iva_bags so the GUI list can
    render with a shared template."""
    root = _iva_bags_root()
    out = []
    if not os.path.isdir(root):
        return out
    try:
        entries = os.listdir(root)
    except OSError:
        return out
    for name in entries:
        # Match the demo-bag naming pattern. We accept any folder
        # that contains '_demo' in the name AND isn't a vision_debug
        # bag (which can also contain 'demo' if someone names it
        # weirdly).
        if 'vision_debug' in name:
            continue
        if not ('_demo1' in name or '_demo2' in name
                or '_demo3' in name or '_demo_run' in name
                or '_demo_untagged' in name):
            continue
        full = os.path.join(root, name)
        if not os.path.isdir(full):
            continue
        try:
            st = os.stat(full)
        except OSError:
            continue
        png = os.path.join(full, 'digest.png')
        js  = os.path.join(full, 'digest.summary.json')
        entry = {
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
            'has_png': os.path.isfile(png),
            'has_summary': os.path.isfile(js),
        }
        if entry['has_summary']:
            try:
                with open(js) as f:
                    s = json.load(f)
                entry['summary'] = {
                    'duration_s': s.get('duration_s'),
                    'demo_label': s.get('demo_label'),
                    'demo_mode': s.get('demo_mode'),
                    'demo_params': s.get('demo_params'),
                    'gains_at_record': s.get('gains_at_record'),
                    'p95_error_mm': (s.get('error_mm') or {}).get('p95'),
                    'rms_error_mm': (s.get('error_mm') or {}).get('rms'),
                    'settling_time_s': s.get('settling_time_s'),
                }
            except Exception:
                pass
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _resolve_demo_bag_path(name):
    if '/' in name or '\\' in name or name.startswith('.'):
        return None
    full = os.path.realpath(os.path.join(_iva_bags_root(), name))
    root = os.path.realpath(_iva_bags_root())
    if not full.startswith(root + os.sep):
        return None
    if not os.path.isdir(full):
        return None
    return full


def _demo_start(label='demo_run'):
    """Spawn `ros2 bag record` for the demo-run topic set. `label`
    is folded into the directory name so the operator can tell at a
    glance which demo a bag was for."""
    global _demo_proc, _demo_path, _demo_started_at, _demo_label
    # Sanitize label so it can't escape the directory.
    safe_label = re.sub(r'[^a-zA-Z0-9_]', '_', str(label) or 'demo_run')[:32]
    if not safe_label:
        safe_label = 'demo_run'
    with _demo_lock:
        if _demo_proc is not None and _demo_proc.poll() is None:
            return False, 'already recording', _demo_path
        ts = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        out_dir = os.path.join(_iva_bags_root(),
                               f'{ts}_{safe_label}')
        os.makedirs(os.path.dirname(out_dir), exist_ok=True)
        cmd = ('source /opt/ros/kilted/setup.bash && '
               'source /home/sorak/ros2_ws/install/local_setup.bash && '
               f'exec ros2 bag record -s mcap -o {out_dir} '
               + ' '.join(DEMO_TOPICS))
        try:
            _demo_proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            _demo_path = out_dir
            _demo_started_at = time.time()
            _demo_label = safe_label
            return True, f'recording → {out_dir}', out_dir
        except Exception as e:
            _demo_proc = None
            _demo_path = None
            _demo_started_at = None
            _demo_label = None
            return False, f'failed to spawn: {e}', None


def _demo_stop():
    global _demo_proc, _demo_path, _demo_started_at, _demo_label
    with _demo_lock:
        if _demo_proc is None:
            return False, 'not recording', _demo_path
        if _demo_proc.poll() is not None:
            path = _demo_path
            _demo_proc = None
            _demo_path = None
            _demo_started_at = None
            _demo_label = None
            return True, f'recorder already exited; bag at {path}', path
        try:
            os.killpg(os.getpgid(_demo_proc.pid), signal.SIGINT)
        except Exception as e:
            return False, f'SIGINT failed: {e}', _demo_path
        try:
            _demo_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(_demo_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        path = _demo_path
        _demo_proc = None
        _demo_path = None
        _demo_started_at = None
        _demo_label = None
        return True, f'saved → {path}', path


def _demo_status():
    with _demo_lock:
        running = (_demo_proc is not None
                   and _demo_proc.poll() is None)
        elapsed = (time.time() - _demo_started_at) \
            if (_demo_started_at and running) else None
        return {
            'recording': running,
            'path': _demo_path,
            'label': _demo_label,
            'elapsed_s': elapsed,
        }


def _digest_demo_bag(name):
    full = _resolve_demo_bag_path(name)
    if full is None:
        return False, "bad name"
    analyzer = os.path.join(_BRINGUP_DIR, 'scripts/digest_demo_bag.py')
    if not os.path.isfile(analyzer):
        return False, f"analyzer not found at {analyzer}"
    cmd = (
        'source /opt/ros/kilted/setup.bash && '
        'source ~/ros2_ws/install/local_setup.bash && '
        f'python3 {analyzer!r} {full!r}'
    )
    try:
        r = subprocess.run(['bash', '-c', cmd],
                           capture_output=True, text=True, timeout=180)
        ok = (r.returncode == 0)
        out = (r.stdout or '').strip()
        err = (r.stderr or '').strip()
        return ok, (out + ('\n' + err if err else ''))[-4000:]
    except subprocess.TimeoutExpired:
        return False, "digest timed out after 180 s"
    except Exception as e:
        return False, f"digest failed: {e}"


def _delete_demo_bag(name):
    bag_dir = _resolve_demo_bag_path(name)
    if bag_dir is None:
        return False, f'no such demo bag: {name}'
    try:
        import shutil
        shutil.rmtree(bag_dir)
    except Exception as e:
        return False, f'rmtree failed: {e}'
    return True, f'deleted {bag_dir}'


def _demo_bag_png_path(name):
    full = _resolve_demo_bag_path(name)
    if full is None:
        return None
    p = os.path.join(full, 'digest.png')
    return p if os.path.isfile(p) else None


def _push_demo_bag_to_git(name):
    """Push only digest.png + digest.summary.json. Raw bag stays
    on the Pi (gitignored — too large per-run)."""
    bag_dir = _resolve_demo_bag_path(name)
    if bag_dir is None:
        return False, f'no such demo bag: {name}'
    repo_dir = os.path.dirname(_iva_bags_root())
    if not os.path.isdir(os.path.join(repo_dir, '.git')):
        return False, f'{repo_dir} is not a git repo'

    out_lines = []
    png = os.path.join(bag_dir, 'digest.png')
    js  = os.path.join(bag_dir, 'digest.summary.json')
    if not (os.path.isfile(png) and os.path.isfile(js)):
        out_lines.append('(running digest first)')
        ok, msg = _digest_demo_bag(name)
        out_lines.append(msg or '')
        if not ok:
            return False, '\n'.join(out_lines)

    rel_png = os.path.relpath(png, repo_dir)
    rel_js  = os.path.relpath(js,  repo_dir)

    headline = ''
    try:
        with open(js) as f:
            s = json.load(f)
        em = s.get('error_mm') or {}
        gn = s.get('gains_at_record') or {}
        dp = s.get('demo_params') or {}
        dm = s.get('demo_mode') or ''
        headline_lines = [
            f"label={s.get('demo_label', '?')}  "
            f"mode={dm}  "
            f"duration={s.get('duration_s', 0):.1f}s  "
            f"settling={s.get('settling_time_s', 'n/a')}",
            f"error_mm: rms={em.get('rms', 0):.1f} "
            f"p95={em.get('p95', 0):.1f} "
            f"max={em.get('max', 0):.1f}",
            f"gains: kp={gn.get('kp', '?')} "
            f"kd={gn.get('kd', '?')} "
            f"ki={gn.get('ki', '?')} "
            f"max_tilt={gn.get('max_tilt_deg', '?')}deg "
            f"signs(p/r)={gn.get('pitch_sign', '?')}/"
            f"{gn.get('roll_sign', '?')}",
        ]
        if dp:
            param_str = ' '.join(f"{k}={v}" for k, v in dp.items()
                                 if k != 'ball')
            headline_lines.append(f"demo_params: {param_str}")
        headline = '\n' + '\n'.join(headline_lines)
    except Exception:
        pass

    msg = (f'Demo digest {name}\n' + headline + '\n\n'
           'Digest only (digest.png + digest.summary.json). Raw '
           'rosbag2 mcap stays on the Pi.\n')

    def _run(cmd):
        try:
            r = subprocess.run(cmd, cwd=repo_dir,
                               capture_output=True, text=True,
                               timeout=120)
            line = ' '.join(cmd)
            if r.returncode != 0:
                out_lines.append(f"FAILED: {line}\n{r.stderr}")
            else:
                out_lines.append(f"OK: {line}")
                if r.stdout.strip():
                    out_lines.append(r.stdout.strip())
            return r.returncode == 0
        except Exception as e:
            out_lines.append(f"FAILED: {' '.join(cmd)}: {e}")
            return False

    if not _run(['git', 'add', '-f', rel_png, rel_js]):
        return False, '\n'.join(out_lines)
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'],
                          cwd=repo_dir)
    nothing_to_commit = (diff.returncode == 0)
    if nothing_to_commit:
        out_lines.append('(nothing to commit — digest already in HEAD)')
    else:
        if not _run(['git', 'commit', '-m', msg]):
            return False, '\n'.join(out_lines)
    # Pull --rebase before push to handle concurrent pushes from
    # other sources (laptop, other GUI sessions). Without this, a
    # "fetch first" rejection surfaces as a confusing GUI error
    # any time someone else has pushed in the seconds before this
    # call (observed 2026-05-01 with an IVA-sweep collision).
    if not _run(['git', 'pull', '--rebase']):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append(
        '✓ pushed' if nothing_to_commit
        else '✓ pushed to origin (digest only)')
    return True, '\n'.join(out_lines)


def _rosbridge_already_running():
    """True if anything is listening on TCP 9090 (rosbridge default port).
    Catches stacks started outside this gui_server (notably systemd on Pi)."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.3)
    try:
        s.connect(('127.0.0.1', 9090))
        return True
    except Exception:
        return False
    finally:
        s.close()


def _run_launch():
    """Spawn `ros2 launch stewart_bringup stewart_gui_launch.py` detached.
    Output goes to LAUNCH_LOG. Returns the Popen or raises."""
    global _launch_proc
    with _launch_lock:
        if _launch_proc is not None and _launch_proc.poll() is None:
            return _launch_proc   # already running
        if _rosbridge_already_running():
            raise RuntimeError(
                "rosbridge already listening on :9090 (probably started by "
                "systemd). Refusing to spawn a duplicate stack — use "
                "`sudo systemctl restart stable_bot.service` instead.")
        os.makedirs(os.path.dirname(LAUNCH_LOG), exist_ok=True)
        log_f = open(LAUNCH_LOG, 'w')
        # Use bash -c so we can source ROS before launching. PATH may not
        # include ros2 if the user didn't source in the env that started
        # this HTTP server.
        cmd = (
            'source /opt/ros/kilted/setup.bash && '
            'source ~/ros2_ws/install/local_setup.bash && '
            'exec ros2 launch stewart_bringup stewart_gui_launch.py'
        )
        _launch_proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=log_f, stderr=subprocess.STDOUT,
            start_new_session=True,  # so Ctrl-C on this server doesn't kill it
        )
        return _launch_proc


def _stop_launch():
    global _launch_proc
    with _launch_lock:
        if _launch_proc is None or _launch_proc.poll() is not None:
            return False, "no launch process tracked"
        try:
            os.killpg(os.getpgid(_launch_proc.pid), signal.SIGINT)
            try:
                _launch_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(_launch_proc.pid), signal.SIGKILL)
                _launch_proc.wait(timeout=2)
            pid = _launch_proc.pid
            _launch_proc = None
            return True, f"stopped pid {pid}"
        except Exception as e:
            return False, f"stop failed: {e}"


def _dir_size(path):
    total = 0
    try:
        for root, _dirs, files in os.walk(path):
            for fn in files:
                try:
                    total += os.path.getsize(os.path.join(root, fn))
                except OSError:
                    pass
    except Exception:
        pass
    return total


def _list_bags():
    """Enumerate ~/stable_bot_bags/. Returns a list of dicts ordered by
    mtime descending. Single bags and sweep dirs are flagged via 'kind'.
    Sweep entries include their child bags as a nested 'children' list."""
    out = []
    if not os.path.isdir(BAGS_DIR):
        return out
    try:
        entries = os.listdir(BAGS_DIR)
    except OSError:
        return out
    for name in entries:
        full = os.path.join(BAGS_DIR, name)
        try:
            st = os.stat(full)
        except OSError:
            continue
        if name.endswith('_notes.json'):
            continue   # sidecars are surfaced under their bag entry
        kind = 'sweep' if name.startswith('sweep_') and os.path.isdir(full) else 'bag'
        entry = {
            'name': name,
            'path': full,
            'kind': kind,
            'mtime': st.st_mtime,
            'size_bytes': _dir_size(full) if os.path.isdir(full)
                          else st.st_size,
        }
        # Sidecar / manifest pickup.
        if kind == 'bag':
            sidecar = full + '_notes.json'
            if os.path.isfile(sidecar):
                try:
                    with open(sidecar) as f:
                        entry['notes'] = json.load(f)
                except Exception:
                    pass
        else:
            manifest = os.path.join(full, 'manifest.json')
            if os.path.isfile(manifest):
                try:
                    with open(manifest) as f:
                        entry['manifest'] = json.load(f)
                except Exception:
                    pass
            # Child bags inside the sweep dir.
            children = []
            try:
                for cn in sorted(os.listdir(full)):
                    cf = os.path.join(full, cn)
                    if not os.path.isdir(cf) or cn.endswith('_notes.json'):
                        continue
                    try:
                        cst = os.stat(cf)
                    except OSError:
                        continue
                    cnotes = None
                    side = cf + '_notes.json'
                    if os.path.isfile(side):
                        try:
                            with open(side) as f:
                                cnotes = json.load(f)
                        except Exception:
                            pass
                    children.append({
                        'name': cn, 'path': cf,
                        'mtime': cst.st_mtime,
                        'size_bytes': _dir_size(cf),
                        'notes': cnotes,
                    })
            except OSError:
                pass
            entry['children'] = children
        out.append(entry)
    out.sort(key=lambda e: e['mtime'], reverse=True)
    return out


def _resolve_bag_path(name):
    """Resolve a user-supplied basename to an absolute path under BAGS_DIR.
    Rejects anything that escapes BAGS_DIR (path-traversal guard).
    Returns the abspath or None."""
    if not name or '/' in name or name in ('.', '..'):
        return None
    full = os.path.realpath(os.path.join(BAGS_DIR, name))
    base = os.path.realpath(BAGS_DIR)
    if not full.startswith(base + os.sep):
        return None
    if not os.path.exists(full):
        return None
    return full


def _delete_bag(name):
    import shutil
    full = _resolve_bag_path(name)
    if full is None:
        return False, "bad name"
    sidecar = full + '_notes.json'
    try:
        if os.path.isdir(full):
            shutil.rmtree(full)
        else:
            os.remove(full)
        if os.path.isfile(sidecar):
            os.remove(sidecar)
        return True, f"deleted {name}"
    except Exception as e:
        return False, f"delete failed: {e}"


def _digest_bag(name):
    """Run analyze_level_bag.py against the named bag/sweep dir.
    Output lands in <repo>/tuning_data/. Path-traversal guarded via
    _resolve_bag_path."""
    full = _resolve_bag_path(name)
    if full is None:
        return False, "bad name"
    analyzer = os.path.join(_BRINGUP_DIR, 'scripts/analyze_level_bag.py')
    if not os.path.isfile(analyzer):
        return False, f"analyzer not found at {analyzer}"
    # Must source ROS for rosbag2_py + jugglebot_interfaces. Match the
    # pattern used by /reset and /launch elsewhere in this server.
    cmd = (
        'source /opt/ros/kilted/setup.bash && '
        'source ~/ros2_ws/install/local_setup.bash && '
        f'python3 {analyzer!r} {full!r}'
    )
    try:
        r = subprocess.run(['bash', '-c', cmd],
                           capture_output=True, text=True, timeout=300)
        ok = (r.returncode == 0)
        out = (r.stdout or '').strip()
        err = (r.stderr or '').strip()
        return ok, (out + ('\n' + err if err else ''))[-4000:]
    except subprocess.TimeoutExpired:
        return False, "digest timed out after 300 s"
    except Exception as e:
        return False, f"digest failed: {e}"


def _launch_status():
    pid = None
    running = False
    with _launch_lock:
        if _launch_proc is not None and _launch_proc.poll() is None:
            running = True
            pid = _launch_proc.pid
    log_tail = ""
    try:
        with open(LAUNCH_LOG, 'r') as f:
            log_tail = f.read()[-4000:]
    except Exception:
        pass
    return {'running': running, 'pid': pid, 'log_tail': log_tail}


class Handler(http.server.SimpleHTTPRequestHandler):
    web_dir = DEFAULT_WEB_DIR

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.web_dir, **kwargs)

    def _send_json(self, obj, status=200):
        # NaN / +Inf / -Inf are valid Python floats but invalid JSON
        # per RFC 8259, and JS's JSON.parse throws on them. The
        # `inner_loop_config` SDO snapshot can legitimately read e.g.
        # vel_integrator_limit = +inf, so we sanitize at the wire.
        data = json.dumps(_finite_scrub(obj), allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def end_headers(self):
        # Disable browser caching for everything we serve. Without this,
        # SimpleHTTPRequestHandler returns 304 Not Modified based on
        # If-Modified-Since, which on systems where index.html is served
        # via a symlink (Pi deployment, repo-clone layout) keeps the
        # browser pinned to a stale copy after a `git pull`. Cost: ~1
        # extra index.html fetch per page load (~50 KB). Worth it.
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        if self.path == '/launch_status':
            self._send_json(_launch_status())
            return
        if self.path == '/bags':
            self._send_json({'bags_dir': BAGS_DIR, 'entries': _list_bags()})
            return
        if self.path == '/iva/status':
            self._send_json(_iva_status())
            return
        if self.path == '/iva/bags':
            self._send_json({'iva_dir': _iva_bags_root(),
                             'entries': _list_iva_bags()})
            return
        if self.path == '/auto_tune/bags':
            self._send_json({'tuning_dir': _iva_bags_root(),
                             'entries': _list_auto_tune_sessions()})
            return
        if self.path == '/step_id/bags':
            self._send_json({'tuning_dir': _iva_bags_root(),
                             'entries': _list_step_id_sessions()})
            return
        if self.path.startswith('/step_id/bags/png'):
            # /step_id/bags/png?name=<session> → plant_gain_fit.png
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            name = (q.get('name') or [''])[0]
            full = _resolve_step_id_session(name)
            if not full:
                self._send_json({'error': 'session not found'}, 404)
                return
            png = os.path.join(full, 'plant_gain_fit.png')
            if not os.path.isfile(png):
                self._send_json(
                    {'error': 'no plant_gain_fit.png — run digest first'},
                    404)
                return
            try:
                with open(png, 'rb') as f:
                    data = f.read()
            except OSError as e:
                self._send_json({'error': str(e)}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith('/auto_tune/bags/png'):
            # /auto_tune/bags/png?name=<session>  →  fitness_curve.png
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            name = (q.get('name') or [''])[0]
            full = _resolve_auto_tune_session(name)
            if not full:
                self._send_json({'error': 'session not found'}, 404)
                return
            png = os.path.join(full, 'fitness_curve.png')
            if not os.path.isfile(png):
                self._send_json(
                    {'error': 'no fitness_curve.png — run digest first'},
                    404)
                return
            try:
                with open(png, 'rb') as f:
                    data = f.read()
            except OSError as e:
                self._send_json({'error': str(e)}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path == '/iva/alignment':
            # Live ArUco→IMU alignment, read directly from the active
            # config file. The GUI uses this to invert the rotation so
            # /ball_state and /ball_ref (both in IMU frame post-§0)
            # render in platform frame on the SVG — operator's click
            # appears under the ball's actual physical position.
            path = os.path.expanduser(
                '~/stable_bot_repo/stewart_vision/config/'
                'aruco_imu_alignment.yaml')
            try:
                import yaml as _yaml
                with open(path) as f:
                    d = _yaml.safe_load(f) or {}
                self._send_json({
                    'matrix': d.get('matrix'),
                    'rotation_deg': d.get('rotation_deg'),
                    'rms_deg': d.get('post_alignment_rms_deg'),
                    'source_bag': d.get('source_bag'),
                })
            except Exception as e:
                self._send_json(
                    {'error': str(e), 'matrix': None}, status=200)
            return
        if self.path == '/v0/weights':
            # Current NN color-score weights. Read from the JSON if
            # present (operator's tuned values), otherwise from the
            # module defaults baked into build_v0_blob.py.
            path = os.path.expanduser(
                '~/stable_bot_repo/stewart_vision/blobs/v0_weights.json')
            try:
                if os.path.isfile(path):
                    with open(path) as f:
                        self._send_json(json.load(f) or {})
                else:
                    # Fallback to defaults so the GUI sliders show
                    # something even before the JSON is created.
                    self._send_json({
                        'w_b': -1.50, 'w_g': 0.40, 'w_r': 1.00,
                        'bias': -0.50, 'score_floor': 0.30,
                        'density_floor': 0.05,
                        'nn_conf_min': 0.0001,
                        'note': 'defaults (no JSON yet)',
                    })
            except Exception as e:
                self._send_json({'error': str(e)}, status=200)
            return
        if self.path == '/vision/status':
            self._send_json(_vision_status())
            return
        if self.path == '/vision/bags':
            self._send_json({'vision_dir': _iva_bags_root(),
                             'entries': _list_vision_bags()})
            return
        if self.path == '/demo/status':
            self._send_json(_demo_status())
            return
        if self.path == '/demo/bags':
            self._send_json({'demo_dir': _iva_bags_root(),
                             'entries': _list_demo_bags()})
            return
        if self.path.startswith('/demo/bags/png'):
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            name = (q.get('name') or [''])[0]
            png = _demo_bag_png_path(name)
            if not png:
                self._send_json(
                    {'error': 'png not found — run digest first'}, 404)
                return
            try:
                with open(png, 'rb') as f:
                    data = f.read()
            except OSError as e:
                self._send_json({'error': str(e)}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith('/vision/bags/png'):
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            name = (q.get('name') or [''])[0]
            png = _vision_bag_png_path(name)
            if not png:
                self._send_json(
                    {'error': 'png not found — run digest first'}, 404)
                return
            try:
                with open(png, 'rb') as f:
                    data = f.read()
            except OSError as e:
                self._send_json({'error': str(e)}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path.startswith('/iva/bags/png'):
            # /iva/bags/png?name=<urlencoded bag name>
            from urllib.parse import urlsplit, parse_qs
            q = parse_qs(urlsplit(self.path).query)
            name = (q.get('name') or [''])[0]
            png = _iva_bag_png_path(name)
            if not png:
                self._send_json({'error': 'png not found'}, 404)
                return
            try:
                with open(png, 'rb') as f:
                    data = f.read()
            except OSError as e:
                self._send_json({'error': str(e)}, 500)
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        return super().do_GET()

    def do_POST(self):
        if self.path == '/reset':
            try:
                r = subprocess.run(
                    ['python3', RESET_SCRIPT],
                    capture_output=True, text=True, timeout=90)
                self._send_json({
                    'rc': r.returncode,
                    'stdout': r.stdout[-4000:],
                    'stderr': r.stderr[-1000:],
                })
            except subprocess.TimeoutExpired:
                self._send_json({'rc': -1, 'stdout': '',
                                 'stderr': 'reset timed out after 30 s'}, 500)
            except Exception as e:
                self._send_json({'rc': -2, 'stdout': '',
                                 'stderr': f'error: {e}'}, 500)
            return
        if self.path == '/launch':
            try:
                p = _run_launch()
                self._send_json({'pid': p.pid, 'log': LAUNCH_LOG})
            except Exception as e:
                self._send_json({'error': str(e)}, 500)
            return
        if self.path == '/stop_launch':
            ok, msg = _stop_launch()
            self._send_json({'ok': ok, 'message': msg})
            return
        if self.path == '/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _delete_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 400)
            return
        if self.path == '/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _digest_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/iva/start':
            ok, msg, path = _iva_start()
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/iva/stop':
            ok, msg, path = _iva_stop()
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/iva/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _digest_iva_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/iva/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _delete_iva_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 400)
            return
        if self.path == '/iva/bags/push':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _git_push_iva_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/iva/bags/apply_alignment':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _apply_iva_alignment(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 400)
            return
        # ----- Vision-debug bag recorder endpoints -----
        if self.path == '/vision/start':
            ok, msg, path = _vision_start()
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/vision/stop':
            ok, msg, path = _vision_stop()
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/vision/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _delete_vision_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 400)
            return
        if self.path == '/vision/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _digest_vision_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/v0/weights':
            # Save NN color-score weights to JSON. Body is JSON object
            # with float fields w_b, w_g, w_r, bias, score_floor.
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            try:
                d = {
                    'w_b':  float(body.get('w_b', -1.50)),
                    'w_g':  float(body.get('w_g', 0.40)),
                    'w_r':  float(body.get('w_r', 1.00)),
                    'bias': float(body.get('bias', -0.50)),
                    'score_floor': float(body.get('score_floor', 0.30)),
                    'density_floor': float(body.get('density_floor', 0.05)),
                    'nn_conf_min': float(body.get('nn_conf_min', 0.0001)),
                    'saved_at_utc': time.strftime(
                        '%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                }
                # Clamp to sane ranges so a runaway slider can't
                # produce a NaN-y blob on the next rebuild.
                d['w_b']  = max(-3.0, min(3.0, d['w_b']))
                d['w_g']  = max(-3.0, min(3.0, d['w_g']))
                d['w_r']  = max(-3.0, min(3.0, d['w_r']))
                d['bias'] = max(-2.0, min(2.0, d['bias']))
                d['score_floor'] = max(0.0, min(1.0, d['score_floor']))
                # density_floor: a 5×5 avg over m=0.46 maxes at ~0.46;
                # values above 0.5 reject everything.
                d['density_floor'] = max(0.0, min(0.5, d['density_floor']))
                # nn_conf_min is host-side only — clamp matches the
                # node's _on_nn_conf_min_cmd clamp [0, 0.05].
                d['nn_conf_min'] = max(0.0, min(0.05, d['nn_conf_min']))
                path = os.path.expanduser(
                    '~/stable_bot_repo/stewart_vision/blobs/'
                    'v0_weights.json')
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    json.dump(d, f, indent=2)
                self._send_json({
                    'ok': True,
                    'message': f'saved {os.path.basename(path)}',
                    'weights': d,
                })
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'save failed: {e}'}, 500)
            return
        if self.path == '/v0/weights/rebuild':
            # Run stewart_vision/scripts/build_v0_blob.py on the Pi.
            # The script reads stewart_vision/blobs/v0_weights.json
            # (which the operator just saved via /v0/weights), builds
            # the ONNX with onnx.helper, and uploads it to the Luxonis
            # cloud blobconverter for the Myriad X compile. Result is
            # the new stewart_vision/blobs/v0_320x180.blob, committed
            # alongside the JSON. No torch dependency — onnx +
            # blobconverter are tiny compared to PyTorch.
            #
            # Rebuilding takes ~30 s (most of it is the cloud round
            # trip). The new blob is loaded by oak_driver_node only
            # at startup, so for the change to take effect at the
            # camera the operator must also restart stable_bot.service.
            try:
                repo = os.path.expanduser('~/stable_bot_repo')
                script = os.path.join(
                    repo, 'stewart_vision', 'scripts', 'build_v0_blob.py')
                if not os.path.isfile(script):
                    self._send_json({
                        'ok': False,
                        'message': f'build script missing: {script}',
                    }, 500)
                    return
                r = subprocess.run(
                    ['python3', script],
                    capture_output=True, text=True,
                    timeout=120, cwd=repo)
                ok = (r.returncode == 0)
                tail = (r.stdout or '').strip().split('\n')[-1] \
                    if (r.stdout or '').strip() else ''
                err = (r.stderr or '').strip()
                msg = tail
                if not ok:
                    msg = f'build failed (exit {r.returncode}): ' + \
                          (err.split(chr(10))[0] if err else 'no stderr')
                self._send_json({
                    'ok': ok,
                    'message': msg,
                    'stdout': (r.stdout or '')[-2000:],
                    'stderr': err[-2000:],
                }, status=200 if ok else 500)
            except subprocess.TimeoutExpired:
                self._send_json({
                    'ok': False,
                    'message': 'rebuild timed out (>120 s) — '
                               'cloud blobconverter may be slow',
                }, 500)
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'rebuild failed: {e}'},
                    500)
            return
        if self.path == '/v0/weights/push':
            # git add + commit + push the weights JSON, the rebuilt
            # .blob, and the intermediate .onnx (useful for inspection).
            # Skips files that don't exist (e.g., before first Rebuild).
            # Failures (no remote, no creds, nothing to commit) are
            # reported but not fatal.
            try:
                repo = os.path.expanduser('~/stable_bot_repo')
                env = os.environ.copy()
                # Artifacts produced by the Save / Rebuild flow.
                # Glob the .blob/.onnx paths so this still works after
                # NN_W/NN_H changes (e.g., 320x180 → 640x360 bumps).
                # The JSON is fixed; everything else discovered live.
                import glob as _glob
                rels = ['stewart_vision/blobs/v0_weights.json']
                blobs_dir = os.path.join(repo, 'stewart_vision', 'blobs')
                for pattern in ('v0_*.blob', 'v0_*.onnx'):
                    for p in sorted(_glob.glob(os.path.join(blobs_dir, pattern))):
                        rels.append(os.path.relpath(p, repo))
                rels = [r for r in rels
                        if os.path.isfile(os.path.join(repo, r))]
                if not rels:
                    self._send_json({
                        'ok': False,
                        'message': 'no v0 artifacts on disk to push',
                    }, 500)
                    return
                add = subprocess.run(
                    ['git', '-C', repo, 'add'] + rels,
                    capture_output=True, text=True, timeout=10)
                if add.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'git add failed: {add.stderr.strip()}',
                    }, 500)
                    return
                # Skip commit if nothing staged.
                diff = subprocess.run(
                    ['git', '-C', repo, 'diff', '--cached', '--quiet'],
                    capture_output=True, text=True)
                if diff.returncode == 0:
                    self._send_json({
                        'ok': True,
                        'message': 'no changes to commit',
                    })
                    return
                ts = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
                # Mention which artifacts went in the commit so future
                # diffs are easy to interpret.
                what = ' + '.join(
                    os.path.basename(r) for r in rels)
                commit = subprocess.run(
                    ['git', '-C', repo, 'commit', '-m',
                     f'NN weights: GUI tune {ts} ({what})'],
                    capture_output=True, text=True, timeout=15)
                if commit.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'commit failed: {commit.stderr.strip()}',
                    }, 500)
                    return
                push = subprocess.run(
                    ['git', '-C', repo, 'push'],
                    capture_output=True, text=True, timeout=30)
                if push.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'commit ok, push failed: '
                            f'{push.stderr.strip()}'),
                    }, 500)
                    return
                self._send_json({
                    'ok': True,
                    'message': (
                        commit.stdout.strip().split('\n')[0]
                        + ' (pushed)'),
                })
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'push failed: {e}'}, 500)
            return
        # ----- Auto-tune per-session endpoints (IVA-style) -----
        if self.path == '/auto_tune/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_auto_tune_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                script = os.path.expanduser(
                    '~/stable_bot_repo/stewart_bringup/scripts/'
                    'digest_auto_tune_bag.py')
                r = subprocess.run(
                    ['python3', script, full],
                    capture_output=True, text=True, timeout=30)
                ok = (r.returncode == 0)
                self._send_json({
                    'ok': ok,
                    'message': (r.stdout or r.stderr).strip(),
                }, status=200 if ok else 500)
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'digest failed: {e}'},
                    500)
            return
        if self.path == '/auto_tune/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_auto_tune_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                import shutil as _shutil
                _shutil.rmtree(full)
                self._send_json({'ok': True,
                                 'message': f'deleted {name}'})
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'delete failed: {e}'},
                    500)
            return
        if self.path == '/auto_tune/bags/push':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_auto_tune_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                repo = os.path.expanduser('~/stable_bot_repo')
                rel = os.path.join('tuning_data', name)
                # Plain `git add` — NOT `-f`. The bag/ subdir is
                # gitignored (multi-MB mcap files exceed GitHub's
                # 100 MB single-file limit). What gets pushed: the
                # log.jsonl + summary.json + digest PNGs. The bag
                # stays on the Pi for offline replay.
                add = subprocess.run(
                    ['git', '-C', repo, 'add', rel],
                    capture_output=True, text=True, timeout=10)
                if add.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'git add: {add.stderr.strip()}',
                    }, 500)
                    return
                diff = subprocess.run(
                    ['git', '-C', repo, 'diff', '--cached', '--quiet'],
                    capture_output=True, text=True)
                if diff.returncode == 0:
                    self._send_json({
                        'ok': True,
                        'message': 'no changes to commit (digest may '
                                   'not be run yet — try Digest first)'})
                    return
                try:
                    with open(os.path.join(full, 'summary.json')) as f:
                        sd = json.load(f) or {}
                    fmsg = (f"f={sd.get('best_fitness', 0):.3f}, "
                            f"n={sd.get('n_trials', 0)} trials")
                except Exception:
                    fmsg = 'auto-tune session'
                msg = f'auto_tune: {name} ({fmsg})'
                commit = subprocess.run(
                    ['git', '-C', repo, 'commit', '-m', msg],
                    capture_output=True, text=True, timeout=15)
                if commit.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'commit: {commit.stderr.strip()}'),
                    }, 500)
                    return
                pull = subprocess.run(
                    ['git', '-C', repo, 'pull', '--rebase'],
                    capture_output=True, text=True, timeout=30)
                if pull.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'pull --rebase: {pull.stderr.strip()}'),
                    }, 500)
                    return
                push = subprocess.run(
                    ['git', '-C', repo, 'push'],
                    capture_output=True, text=True, timeout=30)
                if push.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'push: {push.stderr.strip()}'),
                    }, 500)
                    return
                self._send_json({
                    'ok': True, 'message': f'pushed {name}'})
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'push failed: {e}'},
                    500)
            return
        # ----- STEP_ID per-session endpoints (mirror auto_tune/bags/*) -----
        if self.path == '/step_id/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_step_id_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                script = os.path.expanduser(
                    '~/stable_bot_repo/stewart_bringup/scripts/'
                    'digest_step_id_bag.py')
                r = subprocess.run(
                    ['python3', script, full],
                    capture_output=True, text=True, timeout=30)
                ok = (r.returncode == 0)
                self._send_json({
                    'ok': ok,
                    'message': (r.stdout or r.stderr).strip(),
                }, status=200 if ok else 500)
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'digest failed: {e}'},
                    500)
            return
        if self.path == '/step_id/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_step_id_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                import shutil as _shutil
                _shutil.rmtree(full)
                self._send_json({'ok': True,
                                 'message': f'deleted {name}'})
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'delete failed: {e}'},
                    500)
            return
        if self.path == '/step_id/bags/push':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            name = body.get('name', '').strip()
            full = _resolve_step_id_session(name)
            if not full:
                self._send_json(
                    {'ok': False,
                     'message': f'session not found: {name!r}'}, 404)
                return
            try:
                repo = os.path.expanduser('~/stable_bot_repo')
                rel = os.path.join('tuning_data', name)
                # Plain `git add` — bag/ stays gitignored. We push the
                # JSON summaries + the three digest PNGs. Same recipe
                # as /auto_tune/bags/push.
                add = subprocess.run(
                    ['git', '-C', repo, 'add', rel],
                    capture_output=True, text=True, timeout=10)
                if add.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'git add: {add.stderr.strip()}',
                    }, 500)
                    return
                diff = subprocess.run(
                    ['git', '-C', repo, 'diff', '--cached', '--quiet'],
                    capture_output=True, text=True)
                if diff.returncode == 0:
                    self._send_json({
                        'ok': True,
                        'message': 'no changes to commit (digest may '
                                   'not be run yet — try Digest first)'})
                    return
                try:
                    with open(os.path.join(full, 'summary.json')) as f:
                        sd = json.load(f) or {}
                    rec = (sd.get('phases', {}) or {}).get(
                        'recommendation', {}) or {}
                    fmsg = (f"G_eff={rec.get('g_eff_used', 0):.0f} "
                            f"mm/s²/°, ωn={rec.get('omega_n_rad_s', 0):.2f}")
                except Exception:
                    fmsg = 'step_id session'
                msg = f'step_id: {name} ({fmsg})'
                commit = subprocess.run(
                    ['git', '-C', repo, 'commit', '-m', msg],
                    capture_output=True, text=True, timeout=15)
                if commit.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'commit: {commit.stderr.strip()}',
                    }, 500)
                    return
                pull = subprocess.run(
                    ['git', '-C', repo, 'pull', '--rebase'],
                    capture_output=True, text=True, timeout=30)
                if pull.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'pull --rebase: {pull.stderr.strip()}'),
                    }, 500)
                    return
                push = subprocess.run(
                    ['git', '-C', repo, 'push'],
                    capture_output=True, text=True, timeout=30)
                if push.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'push: {push.stderr.strip()}',
                    }, 500)
                    return
                self._send_json({
                    'ok': True, 'message': f'pushed {name}'})
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'push failed: {e}'},
                    500)
            return
        # ----- Auto-tune legacy single-session endpoints -----
        if self.path == '/auto_tune/digest':
            # Body: {"session": "<dirname under tuning_data/>"}.
            # Defaults to the latest session if omitted. Runs
            # digest_auto_tune_bag.py on the session — produces
            # fitness_curve.png + gain_trajectory.png +
            # target_coverage.png + digest.summary.json next to
            # log.jsonl.
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = (json.loads(self.rfile.read(length) or b'{}')
                        if length else {})
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            session = body.get('session', '').strip()
            try:
                root = os.path.expanduser('~/stable_bot_repo/tuning_data')
                if not session:
                    sessions = sorted(
                        d for d in os.listdir(root)
                        if d.startswith('auto_tune_'))
                    if not sessions:
                        self._send_json(
                            {'ok': False,
                             'message': 'no auto_tune sessions found'},
                            404)
                        return
                    session = sessions[-1]
                session_dir = os.path.join(root, session)
                if not os.path.isdir(session_dir):
                    self._send_json(
                        {'ok': False,
                         'message': f'session not found: {session}'},
                        404)
                    return
                script = os.path.expanduser(
                    '~/stable_bot_repo/stewart_bringup/scripts/'
                    'digest_auto_tune_bag.py')
                r = subprocess.run(
                    ['python3', script, session_dir],
                    capture_output=True, text=True, timeout=30)
                ok = (r.returncode == 0)
                self._send_json({
                    'ok': ok,
                    'message': (r.stdout or r.stderr).strip().split('\n')[-1],
                    'session': session,
                    'stdout': (r.stdout or '')[-2000:],
                    'stderr': (r.stderr or '')[-2000:],
                }, status=200 if ok else 500)
            except Exception as e:
                self._send_json(
                    {'ok': False, 'message': f'digest failed: {e}'},
                    500)
            return
        if self.path == '/auto_tune/push':
            # git add + commit + push the latest auto_tune_<UTC>/
            # session directory (log.jsonl + summary.json + digest
            # PNGs + the bag mcap if it exists). Per-session push
            # with a commit message that includes the best fitness.
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = (json.loads(self.rfile.read(length) or b'{}')
                        if length else {})
            except Exception:
                self._send_json({'ok': False,
                                 'message': 'bad JSON'}, 400)
                return
            session = body.get('session', '').strip()
            try:
                repo = os.path.expanduser('~/stable_bot_repo')
                root = os.path.join(repo, 'tuning_data')
                if not session:
                    sessions = sorted(
                        d for d in os.listdir(root)
                        if d.startswith('auto_tune_'))
                    if not sessions:
                        self._send_json(
                            {'ok': False,
                             'message': 'no auto_tune sessions found'},
                            404)
                        return
                    session = sessions[-1]
                rel = os.path.join('tuning_data', session)
                # Plain `git add` so .gitignore (which excludes the
                # bag/*.mcap from this dir) is respected — bag is
                # too big for GitHub's 100 MB file limit.
                add = subprocess.run(
                    ['git', '-C', repo, 'add', rel],
                    capture_output=True, text=True, timeout=10)
                if add.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': f'git add failed: {add.stderr.strip()}',
                    }, 500)
                    return
                diff = subprocess.run(
                    ['git', '-C', repo, 'diff', '--cached', '--quiet'],
                    capture_output=True, text=True)
                if diff.returncode == 0:
                    self._send_json({
                        'ok': True,
                        'message': 'no changes to commit'})
                    return
                # Read summary.json for a meaningful commit message.
                try:
                    with open(os.path.join(root, session,
                                           'summary.json')) as f:
                        sd = json.load(f) or {}
                    fmsg = (f"f={sd.get('best_fitness', 0):.3f}, "
                            f"n={sd.get('n_trials', 0)} trials")
                except Exception:
                    fmsg = 'auto-tune session'
                msg = f'auto_tune: {session} ({fmsg})'
                commit = subprocess.run(
                    ['git', '-C', repo, 'commit', '-m', msg],
                    capture_output=True, text=True, timeout=15)
                if commit.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'commit failed: '
                            f'{commit.stderr.strip()}'),
                    }, 500)
                    return
                # pull --rebase first in case remote is ahead — long
                # tuning sessions on the Pi are exactly the situation
                # where work has been pushed from elsewhere in the
                # meantime. The just-staged auto_tune session lives
                # only in the index, so a rebase is non-destructive.
                pull = subprocess.run(
                    ['git', '-C', repo, 'pull', '--rebase'],
                    capture_output=True, text=True, timeout=30)
                if pull.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'pull --rebase failed: '
                            f'{pull.stderr.strip()}'),
                    }, 500)
                    return
                push = subprocess.run(
                    ['git', '-C', repo, 'push'],
                    capture_output=True, text=True, timeout=30)
                if push.returncode != 0:
                    self._send_json({
                        'ok': False,
                        'message': (
                            f'push failed: {push.stderr.strip()}'),
                    }, 500)
                    return
                self._send_json({
                    'ok': True,
                    'message': f'pushed {session}'})
            except Exception as e:
                self._send_json(
                    {'ok': False,
                     'message': f'push failed: {e}'},
                    500)
            return
        # ----- Demo-run bag recorder endpoints -----
        if self.path == '/demo/start':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = (json.loads(self.rfile.read(length) or b'{}')
                        if length else {})
                label = str(body.get('label', 'demo_run'))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg, path = _demo_start(label=label)
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/demo/stop':
            ok, msg, path = _demo_stop()
            self._send_json(
                {'ok': ok, 'message': msg, 'path': path},
                status=200 if ok else 409)
            return
        if self.path == '/demo/bags/delete':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _delete_demo_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 400)
            return
        if self.path == '/demo/bags/digest':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _digest_demo_bag(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/demo/bags/push':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _push_demo_bag_to_git(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/vision/bags/push':
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                name = str(body.get('name', ''))
            except Exception:
                self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
                return
            ok, msg = _push_vision_bag_to_git(name)
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        if self.path == '/gains/push':
            ok, msg = _push_ball_track_gains_to_git()
            self._send_json({'ok': ok, 'message': msg},
                            status=200 if ok else 500)
            return
        self.send_error(404)

    # Quieter logging
    def log_message(self, fmt, *args):
        sys.stderr.write(f"[{self.log_date_time_string()}] {fmt % args}\n")


class ReusingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=8080)
    p.add_argument('--host', default='127.0.0.1',
                   help='bind address (default localhost only — do NOT expose '
                        'this to the network, the endpoints can run arbitrary '
                        'commands)')
    p.add_argument('--web-dir', default=DEFAULT_WEB_DIR)
    args = p.parse_args()

    Handler.web_dir = args.web_dir
    srv = ReusingServer((args.host, args.port), Handler)
    print(f"serving {args.web_dir} on http://{args.host}:{args.port}")
    print(f"endpoints: GET /launch_status, GET /bags, POST /reset, POST /launch, "
          f"POST /stop_launch, POST /bags/delete, POST /bags/digest")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        srv.server_close()


if __name__ == '__main__':
    main()
