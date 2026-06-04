#!/usr/bin/env python3
"""focus_sweep.py — empirical focus calibration for one platform Z.

Steps the OAK lens through its focus range, measures image sharpness
(Tenengrad / variance-of-Laplacian, etc.) at each position, finds the
sharpness peak, and writes the curve + a recommended focus_pos to
tuning_data/. Repeat at several Z values to build the empirical
`z_focus_map.yaml` (see oak_focus_exposure_autocal.md).

SAFETY — can't make things worse:
  - Reads the CURRENT focus from /oak/config first and **restores it on
    exit** (normal finish, Ctrl-C, or error) via a finally block.
  - Pure measurement otherwise; the sweep is an operator-run calibration,
    not something that runs during a demo.
  - Recommends a focus but does NOT apply it — you commit the map and
    wire it up deliberately later.

Run on the Pi (oak_driver must be up; keep a stationary, textured target
— the platform with the ArUco ring + a parked ball — in view):
    source ~/ros2_ws/install/local_setup.bash
    python3 ~/stable_bot_repo/stewart_vision/scripts/focus_sweep.py --z-mm 300
    # → tuning_data/<UTC>_focuscal/{focus_sweep.csv,focus_curve.png,summary.yaml,...}

Spec: ../../stewart_bringup/docs/oak_focus_exposure_autocal.md.
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

try:
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS

# focus measures only (higher = sharper) — what we peak on.
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


def _center_roi(img: np.ndarray, frac: float = 0.4) -> np.ndarray:
    h, w = img.shape[:2]
    ch, cw = int(h * frac), int(w * frac)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    return img[y0:y0 + ch, x0:x0 + cw]


class FocusSweepNode(Node):
    def __init__(self, roi_frac: float):
        super().__init__('focus_sweep')
        self.roi_frac = roi_frac
        self.frames = []
        self.config = None
        self.pub = self.create_publisher(String, '/oak/cmd_focus', 1)
        self.create_subscription(
            CompressedImage, '/oak/rgb/image_compressed', self._on_img,
            qos_profile_sensor_data)
        self.create_subscription(String, '/oak/config', self._on_config, 5)

    def _on_img(self, msg: CompressedImage):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frames.append(img)
            if len(self.frames) > 60:
                self.frames = self.frames[-60:]

    def _on_config(self, msg: String):
        try:
            self.config = json.loads(msg.data)
        except Exception:
            pass

    def wait_config(self, timeout=10.0):
        t = time.monotonic() + timeout
        while rclpy.ok() and self.config is None and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.config

    def set_focus(self, payload: str):
        self.pub.publish(String(data=str(payload)))

    def collect(self, n: int, settle_s: float, timeout=4.0):
        """Wait settle_s, then gather up to n fresh frames."""
        t_end = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.frames = []
        t_to = time.monotonic() + timeout
        while rclpy.ok() and len(self.frames) < n and time.monotonic() < t_to:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.frames)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--focus-min', type=int, default=0)
    ap.add_argument('--focus-max', type=int, default=255)
    ap.add_argument('--step', type=int, default=5)
    ap.add_argument('--frames', type=int, default=8,
                    help='frames averaged per focus position')
    ap.add_argument('--settle', type=float, default=0.4,
                    help='seconds to wait after a focus change')
    ap.add_argument('--roi-frac', type=float, default=0.4,
                    help='central fraction used for the metric (0-1)')
    ap.add_argument('--z-mm', type=float, default=None,
                    help='platform Z for this sweep (LABEL only — tag so '
                         'multiple sweeps build the z->focus map)')
    ap.add_argument('--metric', default='tenengrad', choices=_FOCUS_KEYS)
    ap.add_argument('--out-dir', default=None)
    args = ap.parse_args()

    rclpy.init()
    node = FocusSweepNode(args.roi_frac)

    cfg = node.wait_config()
    if cfg is None:
        node.get_logger().error(
            "no /oak/config — is oak_driver running? Aborting (nothing changed).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    orig_mode = cfg.get('focus_mode', 'auto')
    orig_pos = int(cfg.get('focus_pos', 145))
    node.get_logger().info(
        f"original focus: {orig_pos} ({orig_mode}) — will restore on exit.")

    positions = list(range(args.focus_min, args.focus_max + 1, args.step))
    rows = []           # (pos, report_dict)
    keyframes = {}      # pos -> frame (min / peak / max)
    try:
        for pos in positions:
            node.set_focus(pos)
            frames = node.collect(args.frames, args.settle)
            if not frames:
                node.get_logger().warn(f"  focus {pos}: no frames, skipping")
                continue
            reps = [image_quality_report(_center_roi(f, args.roi_frac))
                    for f in frames]
            agg = {k: float(np.mean([r[k] for r in reps if k in r]))
                   for k in METRIC_KEYS
                   if any(k in r for r in reps)}
            rows.append((pos, agg))
            keyframes[pos] = frames[len(frames) // 2]
            node.get_logger().info(
                f"  focus {pos:3d}: {args.metric}={agg.get(args.metric, 0):.4g} "
                f"clip+={agg.get('over_clip_frac', 0):.3f}")
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted — restoring focus and saving partial.")
    finally:
        # --- SAFETY: always restore the original focus ---
        if orig_mode == 'manual':
            node.set_focus(orig_pos)
        else:
            node.set_focus('auto' if orig_mode == 'auto' else 'auto_one_shot')
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f"focus restored to {orig_pos} ({orig_mode}).")

    if not rows:
        node.get_logger().error("no usable rows captured.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(3)

    # peak per focus metric
    peaks = {}
    for k in _FOCUS_KEYS:
        vals = [(p, a.get(k, float('nan'))) for p, a in rows]
        vals = [(p, v) for p, v in vals if np.isfinite(v)]
        if vals:
            peaks[k] = max(vals, key=lambda pv: pv[1])  # (pos, value)
    recommended_pos = peaks.get(args.metric, (orig_pos, 0.0))[0]

    ts = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out_dir = args.out_dir or os.path.join(_tuning_data_dir(), f'{ts}_focuscal')
    os.makedirs(out_dir, exist_ok=True)

    # CSV
    with open(os.path.join(out_dir, 'focus_sweep.csv'), 'w') as f:
        f.write('focus_pos,' + ','.join(METRIC_KEYS) + '\n')
        for pos, a in rows:
            f.write(f'{pos},' + ','.join(f'{a.get(k, ""):}' for k in METRIC_KEYS) + '\n')

    # plot (focus measures, normalized to their own max for overlay)
    fig, ax = plt.subplots(figsize=(8, 5))
    xs = [p for p, _ in rows]
    for k in _FOCUS_KEYS:
        ys = np.array([a.get(k, np.nan) for _, a in rows], dtype=float)
        m = np.nanmax(ys)
        if m and np.isfinite(m):
            ax.plot(xs, ys / m, marker='.', label=k)
    ax.axvline(recommended_pos, color='k', ls='--',
               label=f'{args.metric} peak = {recommended_pos}')
    ax.axvline(orig_pos, color='r', ls=':', label=f'original = {orig_pos}')
    zlabel = f' (Z={args.z_mm:.0f} mm)' if args.z_mm is not None else ''
    ax.set_title(f'Focus sweep{zlabel}')
    ax.set_xlabel('focus_pos (0=inf, 255=macro)')
    ax.set_ylabel('sharpness (normalized per metric)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, 'focus_curve.png'), dpi=110)
    plt.close(fig)

    # keyframes at min / peak / original
    for tag, p in (('min', positions[0]), ('peak', recommended_pos),
                   ('orig', orig_pos)):
        # nearest captured position
        if keyframes:
            near = min(keyframes, key=lambda q: abs(q - p))
            cv2.imwrite(os.path.join(out_dir, f'frame_{tag}_{near}.jpg'),
                        keyframes[near])

    summary = {
        'kind': 'focus_sweep',
        'utc': ts,
        'z_mm': args.z_mm,
        'oak_config_at_start': cfg,
        'sweep': {'focus_min': args.focus_min, 'focus_max': args.focus_max,
                  'step': args.step, 'frames_per_pos': args.frames,
                  'roi_frac': args.roi_frac, 'metric': args.metric},
        'original_focus_pos': orig_pos,
        'original_focus_mode': orig_mode,
        'peak_per_metric': {k: {'focus_pos': int(p), 'value': float(v)}
                            for k, (p, v) in peaks.items()},
        'recommended_focus_pos': int(recommended_pos),
        'restored_to': orig_pos,
        'notes': ('Recommended focus is the sharpness peak for THIS Z; it '
                  'was NOT applied. Repeat at several --z-mm to build '
                  'z_focus_map.yaml. Focus was restored to the pre-sweep '
                  'value, so the live system is unchanged.'),
    }
    with open(os.path.join(out_dir, 'summary.yaml'), 'w') as f:
        if yaml is not None:
            yaml.safe_dump(summary, f, sort_keys=False)
        else:
            json.dump(summary, f, indent=2)

    print(f"\n=== FOCUS SWEEP {('Z=%.0fmm' % args.z_mm) if args.z_mm is not None else ''} ===")
    print(f"  recommended focus_pos ({args.metric} peak): {recommended_pos}")
    print(f"  original focus_pos (restored):              {orig_pos}")
    print(f"  written → {out_dir}")
    print("  commit/push tuning_data/ (pi_deploy.sh) to share for analysis.\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
