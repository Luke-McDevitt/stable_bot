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
    if diff.returncode == 0:
        out_lines.append('(nothing to commit — bag already in HEAD)')
        # Still attempt push in case earlier commit didn't reach origin.
        if _run(['git', 'push']):
            out_lines.append('✓ pushed')
            return True, '\n'.join(out_lines)
        return False, '\n'.join(out_lines)

    if not _run(['git', 'commit', '-m', msg]):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append('✓ pushed to origin')
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
        out.append({
            'name': name,
            'path': full,
            'mtime': st.st_mtime,
            'size_bytes': _iva_dir_size(full),
        })
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


def _push_vision_bag_to_git(name):
    """git add + commit + push of a vision-debug bag, mirroring the
    IVA push flow but with a different commit message body."""
    bag_dir = _resolve_vision_bag_path(name)
    if bag_dir is None:
        return False, f'no such vision bag: {name}'
    repo_dir = os.path.dirname(_iva_bags_root())
    if not os.path.isdir(os.path.join(repo_dir, '.git')):
        return False, f'{repo_dir} is not a git repo'
    rel_bag = os.path.relpath(bag_dir, repo_dir)
    msg = (f'Vision-debug bag {name}\n\n'
           f'Captured by gui_server vision-debug recorder. Topics:\n'
           + ''.join(f'  - {t}\n' for t in VISION_DEBUG_TOPICS))

    out_lines = []

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

    if not _run(['git', 'add', rel_bag]):
        return False, '\n'.join(out_lines)
    diff = subprocess.run(['git', 'diff', '--cached', '--quiet'],
                          cwd=repo_dir)
    if diff.returncode == 0:
        out_lines.append('(nothing to commit — bag already in HEAD)')
        if _run(['git', 'push']):
            out_lines.append('✓ pushed')
            return True, '\n'.join(out_lines)
        return False, '\n'.join(out_lines)
    if not _run(['git', 'commit', '-m', msg]):
        return False, '\n'.join(out_lines)
    if not _run(['git', 'push']):
        return False, '\n'.join(out_lines)
    out_lines.append('✓ pushed to origin')
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
        if self.path == '/vision/status':
            self._send_json(_vision_status())
            return
        if self.path == '/vision/bags':
            self._send_json({'vision_dir': _iva_bags_root(),
                             'entries': _list_vision_bags()})
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
