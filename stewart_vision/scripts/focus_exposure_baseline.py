#!/usr/bin/env python3
"""focus_exposure_baseline.py — capture the CURRENT focus/exposure state.

**Observe-only.** Subscribes to the live OAK streams, records the current
camera config and the focus/exposure metrics on the current image, and
writes a baseline snapshot under tuning_data/. It NEVER sends a camera
command, so it cannot change or degrade anything — its only job is to
pin down "where we are now" so every later tweak can be checked against
it (don't-make-it-worse gate).

Run on the Pi (the OAK is owned by the running oak_driver; this just
listens):
    source ~/ros2_ws/install/local_setup.bash
    python3 ~/stable_bot_repo/stewart_vision/scripts/focus_exposure_baseline.py
    # → tuning_data/<UTC>_baseline/{baseline.yaml,metrics.csv,frame_*.jpg}

Then run pi_deploy.sh (or git add tuning_data/) to push it to GitHub.

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
    print("ERROR: opencv (cv2) required to decode JPEG frames.", file=sys.stderr)
    raise

try:
    import yaml
except ImportError:
    yaml = None

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Float32, Float32MultiArray

# Metric math lives in the package; add the package parent to the path if
# the workspace overlay isn't sourced.
try:
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS


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


class BaselineNode(Node):
    def __init__(self, n_frames: int):
        super().__init__('focus_exposure_baseline')
        self.n_frames = n_frames
        self.frames = []            # decoded BGR frames
        self.config = None          # latest /oak/config dict
        self.latency_ms = []        # latest /oak/latency_ms samples
        self.health = None          # latest /oak/health array
        self.create_subscription(
            CompressedImage, '/oak/rgb/image_compressed', self._on_img,
            qos_profile_sensor_data)
        self.create_subscription(String, '/oak/config', self._on_config, 5)
        self.create_subscription(Float32, '/oak/latency_ms', self._on_lat, 10)
        self.create_subscription(
            Float32MultiArray, '/oak/health', self._on_health, 5)

    def _on_img(self, msg: CompressedImage):
        if len(self.frames) >= self.n_frames:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frames.append(img)

    def _on_config(self, msg: String):
        try:
            self.config = json.loads(msg.data)
        except Exception:
            pass

    def _on_lat(self, msg: Float32):
        self.latency_ms.append(float(msg.data))

    def _on_health(self, msg: Float32MultiArray):
        self.health = [float(v) for v in msg.data]


def _aggregate(reports):
    """mean/std/min/max for each metric across frames."""
    out = {}
    for k in METRIC_KEYS:
        vals = [r[k] for r in reports if k in r]
        if not vals:
            continue
        a = np.asarray(vals, dtype=np.float64)
        out[k] = {'mean': float(a.mean()), 'std': float(a.std()),
                  'min': float(a.min()), 'max': float(a.max())}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--frames', type=int, default=20,
                    help='how many frames to average (default 20)')
    ap.add_argument('--timeout', type=float, default=20.0,
                    help='max seconds to wait for frames')
    ap.add_argument('--out-dir', default=None,
                    help='output session dir (default tuning_data/<UTC>_baseline)')
    args = ap.parse_args()

    rclpy.init()
    node = BaselineNode(args.frames)
    node.get_logger().info(
        f"listening for {args.frames} frames on /oak/rgb/image_compressed "
        "(observe-only; no commands sent)…")

    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and len(node.frames) < args.frames and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    frames = list(node.frames)
    if not frames:
        node.get_logger().error(
            "no frames received — is oak_driver running and publishing "
            "/oak/rgb/image_compressed? (Open the GUI to confirm a live feed.)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    full_reports = [image_quality_report(f) for f in frames]
    roi_reports = [image_quality_report(_center_roi(f)) for f in frames]

    ts = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
    out_dir = args.out_dir or os.path.join(_tuning_data_dir(), f'{ts}_baseline')
    os.makedirs(out_dir, exist_ok=True)

    snapshot = {
        'kind': 'focus_exposure_baseline',
        'utc': ts,
        'n_frames': len(frames),
        'oak_config': node.config,
        'latency_ms_mean': (float(np.mean(node.latency_ms))
                            if node.latency_ms else None),
        'oak_health': node.health,
        'metrics_full_frame': _aggregate(full_reports),
        'metrics_center_roi': _aggregate(roi_reports),
        'notes': ('Baseline of the CURRENT camera state. Compare later '
                  'focus/exposure tweaks against metrics_center_roi — '
                  'sharpness (tenengrad / variance_of_laplacian) should not '
                  'drop and over/under_clip_frac should not rise.'),
    }

    # baseline.yaml (human + tool readable)
    base_path = os.path.join(out_dir, 'baseline.yaml')
    with open(base_path, 'w') as f:
        if yaml is not None:
            yaml.safe_dump(snapshot, f, sort_keys=False, default_flow_style=False)
        else:
            json.dump(snapshot, f, indent=2)

    # per-frame CSV for full traceability
    csv_path = os.path.join(out_dir, 'metrics.csv')
    with open(csv_path, 'w') as f:
        f.write('frame,roi,' + ','.join(METRIC_KEYS) + '\n')
        for i, (fr, rr) in enumerate(zip(full_reports, roi_reports)):
            f.write(f'{i},full,' + ','.join(f'{fr.get(k, ""):}' for k in METRIC_KEYS) + '\n')
            f.write(f'{i},center,' + ','.join(f'{rr.get(k, ""):}' for k in METRIC_KEYS) + '\n')

    # a few keyframes so the actual image quality is inspectable
    for i in (0, len(frames) // 2, len(frames) - 1):
        cv2.imwrite(os.path.join(out_dir, f'frame_{i:02d}.jpg'), frames[i])

    roi = snapshot['metrics_center_roi']
    node.get_logger().info(f"baseline written → {out_dir}")
    print("\n=== BASELINE (center ROI, mean over frames) ===")
    cfg = node.config or {}
    print(f"  exposure: {cfg.get('exp_us')} us   iso: {cfg.get('iso')}   "
          f"focus: {cfg.get('focus_pos')} ({cfg.get('focus_mode')})")
    for k in ('tenengrad', 'variance_of_laplacian', 'over_clip_frac',
              'under_clip_frac', 'sat_mean', 'p50'):
        if k in roi:
            print(f"  {k:24s} {roi[k]['mean']:.4g}")
    if node.latency_ms:
        print(f"  latency_ms (mean)        {np.mean(node.latency_ms):.1f}")
    print("\nCommit/push tuning_data/ (pi_deploy.sh does this) to share it.\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
