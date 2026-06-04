#!/usr/bin/env python3
"""z_focus_map.py — empirical Z->focus map builder (engine for the GUI panel).

Drives the platform through a range of Z heights; at each Z, sweeps the OAK
lens focus, measures sharpness (Tenengrad / variance-of-Laplacian), and
records the focus_pos that maximizes it. Assembles a z_focus_map so the
camera can use a fixed, Z-scheduled MANUAL focus instead of autofocus
(which pauses the sensor to hunt — the documented FPS killer).

Spec: ../../stewart_bringup/docs/oak_focus_exposure_autocal.md.

SAFETY (this command MOVES the platform autonomously):
  - Z is hard-clamped to [Z_SAFE_MIN, Z_SAFE_MAX] = [0, 80] mm (the
    validated ball-demo range); the control node + level loop also clamp to
    leg limits.
  - Reuses the proven 'hold level at Z' path: enables the PI level loop and
    sets Z via set_pose (the loop tracks to it, IMU-leveled), then settles.
  - `finally`: returns to rest (Z=0), restores the prior level state, and
    restores the original focus — even on Ctrl-C / error.
  - Record-only: writes the map + data; does NOT switch the live camera off
    autofocus. Adopting the map (promoting it to config/ + wiring
    Z-scheduled focus into oak_driver) is a separate, deliberate step.

Run on the Pi (platform homed + armed; oak_driver up; a stationary textured
target — the platform + ArUco ring + a parked ball — in view). Do a tiny
2-point range FIRST to confirm the motion is safe:
    source ~/ros2_ws/install/local_setup.bash
    python3 ~/stable_bot_repo/stewart_vision/scripts/z_focus_map.py \
        --z-start 50 --z-stop 60 --z-step 10
    # then the full map:
    python3 ~/stable_bot_repo/stewart_vision/scripts/z_focus_map.py \
        --z-start 0 --z-stop 80 --z-step 20
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    print("ERROR: opencv (cv2) required.", file=sys.stderr)
    raise

try:
    import yaml
except ImportError:
    yaml = None

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from jugglebot_interfaces.srv import SetPose, ActivateOrDeactivate

try:
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS

# Hard Z envelope — confirmed safe operating range (matches the Demo-2 Z
# slider). The control node clamps to leg limits as a backstop.
Z_SAFE_MIN = 0.0
Z_SAFE_MAX = 80.0

_FOCUS_KEYS = ('tenengrad', 'variance_of_laplacian',
               'sum_modified_laplacian', 'normalized_variance')


def _tuning_data_dir() -> str:
    for cand in (os.environ.get('STABLE_BOT_REPO'),
                 os.path.expanduser('~/stable_bot_repo')):
        if cand and os.path.isdir(cand):
            return os.path.join(cand, 'tuning_data')
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(d, '.git')):
            return os.path.join(d, 'tuning_data')
        nd = os.path.dirname(d)
        if nd == d:
            break
        d = nd
    return os.path.join(os.path.expanduser('~'), 'tuning_data')


def _center_roi(img, frac=0.4):
    h, w = img.shape[:2]
    ch, cw = int(h * frac), int(w * frac)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    return img[y0:y0 + ch, x0:x0 + cw]


class ZFocusMapNode(Node):
    def __init__(self):
        super().__init__('z_focus_map')
        self.frames = []
        self.config = None
        self.status = None
        self.pub_focus = self.create_publisher(String, '/oak/cmd_focus', 1)
        self.cli_pose = self.create_client(SetPose, 'set_pose')
        self.cli_level = self.create_client(ActivateOrDeactivate, 'enable_level')
        self.create_subscription(
            CompressedImage, '/oak/rgb/image_compressed', self._on_img,
            qos_profile_sensor_data)
        self.create_subscription(String, '/oak/config', self._on_config, 5)
        self.create_subscription(String, '/status', self._on_status, 5)

    def _on_img(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frames.append(img)
            if len(self.frames) > 60:
                self.frames = self.frames[-60:]

    def _on_config(self, msg):
        try:
            self.config = json.loads(msg.data)
        except Exception:
            pass

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except Exception:
            pass

    def wait_config(self, timeout=12.0):
        t = time.monotonic() + timeout
        while rclpy.ok() and self.config is None and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.config

    def wait_status(self, timeout=8.0):
        t = time.monotonic() + timeout
        while rclpy.ok() and self.status is None and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.status

    def set_focus(self, payload):
        self.pub_focus.publish(String(data=str(payload)))

    def _call(self, client, req, timeout):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def set_level(self, activate, timeout=15.0):
        """Enable/disable the PI level loop — the proven 'hold level at Z'
        path. Returns (ok, msg)."""
        req = ActivateOrDeactivate.Request()
        req.command = 'activate' if activate else 'deactivate'
        r = self._call(self.cli_level, req, timeout)
        if r is None:
            return False, 'enable_level service unavailable/timed out'
        return bool(r.success), str(r.message)

    def set_z(self, z, timeout=10.0):
        """Set the platform target to (0,0,z,0,0,0). With the level loop ON
        this returns immediately and the loop tracks to Z (hold-level), so
        the caller MUST settle-wait. Returns (ok, msg)."""
        req = SetPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = float(z)
        req.roll = 0.0
        req.pitch = 0.0
        req.yaw = 0.0
        req.blocking = False
        r = self._call(self.cli_pose, req, timeout)
        if r is None:
            return False, 'set_pose service unavailable/timed out'
        return bool(r.success), str(r.message)

    def spin_for(self, seconds):
        t = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def collect(self, n, settle_s, timeout=4.0):
        self.spin_for(settle_s)
        self.frames = []
        t_to = time.monotonic() + timeout
        while rclpy.ok() and len(self.frames) < n and time.monotonic() < t_to:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.frames)


def sweep_focus(node, positions, frames, settle, roi_frac, metric):
    """Sweep focus at the CURRENT platform pose. Returns (rows, keyframes,
    (peak_pos, peak_val))."""
    rows = []
    keyframes = {}
    for pos in positions:
        node.set_focus(pos)
        fr = node.collect(frames, settle)
        if not fr:
            node.get_logger().warn(f"    focus {pos}: no frames")
            continue
        reps = [image_quality_report(_center_roi(f, roi_frac)) for f in fr]
        agg = {k: float(np.mean([r[k] for r in reps if k in r]))
               for k in METRIC_KEYS if any(k in r for r in reps)}
        rows.append((pos, agg))
        keyframes[pos] = fr[len(fr) // 2]
    vals = [(p, a.get(metric, float('nan'))) for p, a in rows]
    vals = [(p, v) for p, v in vals if np.isfinite(v)]
    peak = max(vals, key=lambda pv: pv[1]) if vals else (None, None)
    return rows, keyframes, peak


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--z-start', type=float, default=0.0)
    ap.add_argument('--z-stop', type=float, default=80.0)
    ap.add_argument('--z-step', type=float, default=20.0)
    ap.add_argument('--settle-z', type=float, default=2.0,
                    help='settle (s) after setting Z, for the level loop to '
                         'converge + vibration to die')
    ap.add_argument('--focus-min', type=int, default=0)
    ap.add_argument('--focus-max', type=int, default=255)
    ap.add_argument('--focus-step', type=int, default=5)
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--settle', type=float, default=0.7,
                    help='settle (s) after each focus change')
    ap.add_argument('--roi-frac', type=float, default=0.4)
    ap.add_argument('--metric', default='tenengrad', choices=_FOCUS_KEYS)
    ap.add_argument('--out-dir', default=None)
    args = ap.parse_args()

    # --- Z safety clamp ---
    z0 = max(Z_SAFE_MIN, min(Z_SAFE_MAX, args.z_start))
    z1 = max(Z_SAFE_MIN, min(Z_SAFE_MAX, args.z_stop))
    if (z0, z1) != (args.z_start, args.z_stop):
        print(f"[safety] Z clamped to [{Z_SAFE_MIN:.0f}, {Z_SAFE_MAX:.0f}] mm: "
              f"start {args.z_start}->{z0}, stop {args.z_stop}->{z1}")
    if z1 < z0:
        z0, z1 = z1, z0
    step = max(1.0, abs(args.z_step))
    z_ladder = list(np.arange(z0, z1 + 0.5 * step, step))
    z_ladder = [float(min(Z_SAFE_MAX, max(Z_SAFE_MIN, z))) for z in z_ladder]

    focus_positions = list(range(args.focus_min, args.focus_max + 1, args.focus_step))

    rclpy.init()
    node = ZFocusMapNode()

    cfg = node.wait_config()
    if cfg is None:
        node.get_logger().error(
            "no /oak/config — is oak_driver running? Aborting (nothing moved).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)
    orig_mode = cfg.get('focus_mode', 'auto')
    orig_pos = int(cfg.get('focus_pos', 145))

    # Read armed + current level state (to restore it afterwards).
    st = node.wait_status()
    orig_level = bool(st.get('level_enabled', False)) if st else False
    armed = st.get('armed') if st else None
    if armed is False:
        node.get_logger().error(
            "platform NOT armed — arm + home + capture level in the GUI first. "
            "Aborting (nothing moved).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    # Reuse the proven 'hold level at Z' path: enable the PI level loop, then
    # set Z (the loop tracks to it, IMU-leveled). Requires armed + a saved
    # level calibration (platform_level.yaml).
    ok, msg = node.set_level(True)
    if not ok:
        node.get_logger().error(
            f"could not enable level loop: {msg}. Fix in the GUI (arm + "
            "'Level ON' must work), then retry. Aborting (nothing moved).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)
    node.get_logger().info(
        f"level loop ON ({msg}); was {'ON' if orig_level else 'OFF'} before. "
        f"Z ladder {z_ladder} mm; focus {args.focus_min}-{args.focus_max} "
        f"step {args.focus_step}; original focus {orig_pos} ({orig_mode}).")

    results = []      # dict per Z
    aborted = False
    try:
        for i, z in enumerate(z_ladder):
            node.get_logger().info(
                f"[{i+1}/{len(z_ladder)}] set Z={z:.0f} mm (level loop holding)…")
            ok, msg = node.set_z(z)
            if not ok:
                node.get_logger().error(f"  set_pose failed: {msg}")
                if i == 0:
                    node.get_logger().error(
                        "  first move failed — aborting (rest + restore in finally).")
                    aborted = True
                    break
                node.get_logger().warn("  skipping this Z.")
                continue
            node.spin_for(args.settle_z)   # let the level loop converge + vibration die
            rows, keyframes, (peak_pos, peak_val) = sweep_focus(
                node, focus_positions, args.frames, args.settle,
                args.roi_frac, args.metric)
            if peak_pos is None:
                node.get_logger().warn(f"  Z={z:.0f}: no usable sharpness data")
                continue
            results.append({'z_mm': float(z), 'clamped': False,
                            'peak_focus_pos': int(peak_pos),
                            'peak_value': float(peak_val),
                            'rows': rows, 'keyframe': keyframes.get(peak_pos)})
            node.get_logger().info(
                f"  Z={z:.0f}: peak focus {peak_pos} ({args.metric}={peak_val:.0f})")
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted — returning to rest + restoring state.")
        aborted = True
    finally:
        # --- SAFETY: rest, restore level state, restore focus ---
        try:
            node.get_logger().info("returning to rest (Z=0)…")
            node.set_z(0.0)
            node.spin_for(args.settle_z)
        except Exception as e:
            node.get_logger().error(f"  rest move error: {e}")
        if not orig_level:
            try:
                node.set_level(False)
            except Exception as e:
                node.get_logger().error(f"  level-restore error: {e}")
        if orig_mode == 'manual':
            node.set_focus(orig_pos)
        else:
            node.set_focus('auto' if orig_mode == 'auto' else 'auto_one_shot')
        node.spin_for(0.5)
        node.get_logger().info(
            f"focus restored to {orig_pos} ({orig_mode}); "
            f"level restored to {'ON' if orig_level else 'OFF'}.")

    if not results:
        node.get_logger().error("no Z points captured.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(3)

    # ---- write outputs ----
    ts = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
    out_dir = args.out_dir or os.path.join(_tuning_data_dir(), f'{ts}_zfocusmap')
    os.makedirs(out_dir, exist_ok=True)

    # per-Z sweep CSVs + the peak keyframe at each Z
    for r in results:
        ztag = f"z{int(round(r['z_mm'])):03d}"
        with open(os.path.join(out_dir, f'focus_sweep_{ztag}.csv'), 'w') as f:
            f.write('focus_pos,' + ','.join(METRIC_KEYS) + '\n')
            for pos, a in r['rows']:
                f.write(f'{pos},' + ','.join(f'{a.get(k, ""):}' for k in METRIC_KEYS) + '\n')
        if r['keyframe'] is not None:
            cv2.imwrite(os.path.join(out_dir, f'frame_{ztag}_peak_{r["peak_focus_pos"]}.jpg'),
                        r['keyframe'])

    # the map itself
    zs = [r['z_mm'] for r in results]
    foci = [r['peak_focus_pos'] for r in results]
    vals = [r['peak_value'] for r in results]

    # combined plot: peak focus_pos vs Z (+ sharpness vs Z)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True)
    ax1.plot(zs, foci, 'o-', color='tab:blue')
    for r in results:
        if r['clamped']:
            ax1.plot(r['z_mm'], r['peak_focus_pos'], 'rx', markersize=11)
    ax1.set_ylabel('peak focus_pos'); ax1.grid(True, alpha=0.3)
    ax1.set_title(f'Z -> focus map ({args.metric} peak; red x = leg-limit clamped)')
    ax2.plot(zs, vals, 's-', color='tab:green')
    ax2.set_xlabel('platform Z (mm)'); ax2.set_ylabel(f'peak {args.metric}')
    ax2.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'z_focus_map.png'), dpi=110)
    plt.close(fig)

    map_doc = {
        'kind': 'z_focus_map',
        'utc': ts,
        'metric': args.metric,
        'z_safe_range_mm': [Z_SAFE_MIN, Z_SAFE_MAX],
        'oak_config_at_start': cfg,
        'original_focus_pos': orig_pos,
        'original_focus_mode': orig_mode,
        'aborted': aborted,
        # The map: focus_pos to use at each Z. NOT yet adopted by the live
        # camera — promote to stewart_vision/config/z_focus_map.yaml + wire
        # Z-scheduled focus into oak_driver as a separate step.
        'map': [{'z_mm': r['z_mm'], 'focus_pos': r['peak_focus_pos'],
                 'peak_value': r['peak_value'], 'clamped': r['clamped']}
                for r in results],
        'notes': ('Record-only. Platform returned to rest and focus restored. '
                  'Repeat / refine before adopting. Red-x points were clamped '
                  'by leg limits (Z not fully reached).'),
    }
    with open(os.path.join(out_dir, 'z_focus_map.yaml'), 'w') as f:
        if yaml is not None:
            yaml.safe_dump(map_doc, f, sort_keys=False)
        else:
            json.dump(map_doc, f, indent=2)

    print("\n=== Z -> FOCUS MAP ===")
    for r in results:
        flag = '  (CLAMPED)' if r['clamped'] else ''
        print(f"  Z={r['z_mm']:5.1f} mm  ->  focus {r['peak_focus_pos']:3d}  "
              f"({args.metric}={r['peak_value']:.0f}){flag}")
    print(f"  written -> {out_dir}")
    print("  commit/push: git add tuning_data && git commit -m 'z-focus map' "
          "&& git pull --rebase && git push\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
