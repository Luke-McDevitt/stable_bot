#!/usr/bin/env python3
"""exposure_sweep.py — find the shortest exposure that still detects the
ball reliably, and profile latency vs exposure along the way.

Sweeps the OAK manual exposure (via /oak/cmd_exposure) at a fixed ISO; at
each setting it measures, over a window:
  - cv2 detection rate (count of /oak/ball/v0/rgb_pixel)  ← the floor metric
  - ball-region saturation / value + clipping (HSV detection needs S/V high)
  - /oak/latency_ms (mean / std / p95)                    ← latency profile

The shortest exposure that keeps detection near its max (and saturation
above the HSV gate) is the recommendation — shorter exposure = less motion
blur AND lower latency (exposure time is part of the capture latency).

SAFETY / hygiene:
  - Reads the current exposure first and RESTORES it on exit (finally),
    even on Ctrl-C / error. Record-only.
  - Writes each exposure's row incrementally (SSH-drop safe).
  - Static test: put a stationary ball + the platform in view. (Motion
    blur itself needs a moving-ball test; this finds the detection floor.)

Spec: ../../stewart_bringup/docs/oak_focus_exposure_autocal.md +
      ../../stewart_bringup/docs/oak_latency_map.md.

Run on the Pi (oak_driver up, cv2 backend, ball in view):
    source ~/ros2_ws/install/local_setup.bash
    python3 ~/stable_bot_repo/stewart_vision/scripts/exposure_sweep.py --iso 1600
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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String, Float32, Float32MultiArray

try:
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_vision._image_quality import image_quality_report, METRIC_KEYS

DEFAULT_EXP_LIST = "8000,6000,4000,3000,2000,1500,1000,750,500"


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


# columns recorded per exposure
ROW_KEYS = ('exp_us', 'iso', 'detect_hz', 'frame_hz', 'detect_frac',
            'latency_mean', 'latency_std', 'latency_p95',
            'sat_mean', 'sat_p50', 'val_mean', 'over_clip_frac',
            'under_clip_frac', 'mean', 'tenengrad')


class ExposureSweepNode(Node):
    def __init__(self):
        super().__init__('exposure_sweep')
        self.frames = []
        self.config = None
        self._det_count = 0
        self._lat = []
        self.pub_exp = self.create_publisher(
            Float32MultiArray, '/oak/cmd_exposure', 1)
        self.create_subscription(
            CompressedImage, '/oak/rgb/image_compressed', self._on_img,
            qos_profile_sensor_data)
        self.create_subscription(String, '/oak/config', self._on_config, 5)
        self.create_subscription(
            PointStamped, '/oak/ball/v0/rgb_pixel', self._on_det, 10)
        self.create_subscription(Float32, '/oak/latency_ms', self._on_lat, 10)

    def _on_img(self, msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            self.frames.append(img)
            if len(self.frames) > 120:
                self.frames = self.frames[-120:]

    def _on_config(self, msg):
        try:
            self.config = json.loads(msg.data)
        except Exception:
            pass

    def _on_det(self, msg):
        self._det_count += 1

    def _on_lat(self, msg):
        self._lat.append(float(msg.data))

    def wait_config(self, timeout=12.0):
        t = time.monotonic() + timeout
        while rclpy.ok() and self.config is None and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.config

    def set_exposure(self, exp_us, iso):
        m = Float32MultiArray()
        m.data = [float(exp_us), float(iso)]
        self.pub_exp.publish(m)

    def measure(self, settle_s, window_s):
        """Settle, then collect frames + count detections + latency over a
        window. Returns (frames, det_count, det_window_s, latencies)."""
        t = time.monotonic() + settle_s
        while rclpy.ok() and time.monotonic() < t:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.frames = []
        self._det_count = 0
        self._lat = []
        t0 = time.monotonic()
        t_end = t0 + window_s
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        dt = max(1e-3, time.monotonic() - t0)
        return list(self.frames), self._det_count, dt, list(self._lat)


def _write_csv(out_dir, rows):
    with open(os.path.join(out_dir, 'exposure_sweep.csv'), 'w') as f:
        f.write(','.join(ROW_KEYS) + '\n')
        for r in rows:
            f.write(','.join(f'{r.get(k, "")}' for k in ROW_KEYS) + '\n')


def _write_plot(out_dir, rows):
    if not rows:
        return
    xs = [r['exp_us'] for r in rows]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(7, 9), sharex=True)
    ax1.plot(xs, [r['detect_hz'] for r in rows], 'o-', color='tab:blue', label='detect Hz')
    ax1.plot(xs, [r['frame_hz'] for r in rows], '.--', color='tab:gray', label='frame Hz')
    ax1.set_ylabel('rate (Hz)'); ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)
    ax1.set_title('exposure sweep — detection, saturation, latency vs exposure')
    ax2.plot(xs, [r['sat_mean'] for r in rows], 's-', color='tab:orange', label='sat mean')
    ax2.plot(xs, [r['under_clip_frac'] for r in rows], 'v-', color='tab:purple', label='under-clip frac')
    ax2.set_ylabel('saturation / clip'); ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)
    ax3.plot(xs, [r['latency_mean'] for r in rows], 'D-', color='tab:green', label='latency mean')
    ax3.fill_between(xs,
                     [r['latency_mean'] - r['latency_std'] for r in rows],
                     [r['latency_mean'] + r['latency_std'] for r in rows],
                     color='tab:green', alpha=0.15)
    ax3.set_xlabel('exposure (µs)'); ax3.set_ylabel('latency (ms)')
    ax3.grid(True, alpha=0.3); ax3.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, 'exposure_sweep.png'), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exp-list', default=DEFAULT_EXP_LIST,
                    help='comma-separated exposure times in µs (high→low)')
    ap.add_argument('--iso', type=int, default=None,
                    help='fixed ISO for the sweep (default: current/config or 1600)')
    ap.add_argument('--settle', type=float, default=1.0,
                    help='settle (s) after each exposure change')
    ap.add_argument('--window', type=float, default=4.0,
                    help='measurement window (s) per exposure')
    ap.add_argument('--roi-frac', type=float, default=0.4)
    ap.add_argument('--detect-floor-frac', type=float, default=0.9,
                    help='recommend the shortest exposure whose detect_hz '
                         'stays >= this fraction of the best detect_hz')
    ap.add_argument('--out-dir', default=None)
    args = ap.parse_args()

    try:
        exposures = [int(x) for x in args.exp_list.split(',') if x.strip()]
    except ValueError:
        print("--exp-list must be comma-separated integers (µs).", file=sys.stderr)
        sys.exit(1)
    exposures = [max(500, min(33000, e)) for e in exposures]

    rclpy.init()
    node = ExposureSweepNode()
    cfg = node.wait_config()
    if cfg is None:
        node.get_logger().error("no /oak/config — is oak_driver running? Aborting.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)
    orig_exp = int(cfg.get('exp_us', 8000))
    orig_iso = int(cfg.get('iso', 1600))
    iso = args.iso if args.iso is not None else orig_iso
    backend = cfg.get('v0_backend')
    node.get_logger().info(
        f"exposure sweep {exposures} µs @ ISO {iso}; original {orig_exp} µs/"
        f"ISO {orig_iso}; v0_backend={backend} (need cv2 for detect_hz).")

    ts = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out_dir = args.out_dir or os.path.join(_tuning_data_dir(), f'{ts}_expsweep')
    os.makedirs(out_dir, exist_ok=True)
    node.get_logger().info(f"writing to {out_dir} (incrementally).")

    rows = []
    try:
        for exp in exposures:
            node.set_exposure(exp, iso)
            frames, det, dt, lats = node.measure(args.settle, args.window)
            frame_hz = len(frames) / dt
            detect_hz = det / dt
            det_frac = (detect_hz / frame_hz) if frame_hz > 1e-6 else 0.0
            if frames:
                reps = [image_quality_report(_center_roi(f, args.roi_frac))
                        for f in frames]
                agg = {k: float(np.mean([r[k] for r in reps if k in r]))
                       for k in ('sat_mean', 'sat_p50', 'val_mean',
                                 'over_clip_frac', 'under_clip_frac', 'mean',
                                 'tenengrad') if any(k in r for r in reps)}
                cv2.imwrite(os.path.join(out_dir, f'frame_exp{exp:05d}.jpg'),
                            frames[len(frames) // 2])
            else:
                agg = {}
            lat = np.asarray(lats, dtype=np.float64) if lats else np.array([np.nan])
            row = {'exp_us': exp, 'iso': iso,
                   'detect_hz': round(detect_hz, 2), 'frame_hz': round(frame_hz, 2),
                   'detect_frac': round(det_frac, 3),
                   'latency_mean': round(float(np.nanmean(lat)), 1),
                   'latency_std': round(float(np.nanstd(lat)), 1),
                   'latency_p95': round(float(np.nanpercentile(lat, 95)), 1)
                   if lats else float('nan')}
            for k in ('sat_mean', 'sat_p50', 'val_mean', 'over_clip_frac',
                      'under_clip_frac', 'mean', 'tenengrad'):
                row[k] = round(agg.get(k, float('nan')), 4)
            rows.append(row)
            _write_csv(out_dir, rows)      # persist now (SSH-drop safe)
            _write_plot(out_dir, rows)
            node.get_logger().info(
                f"  {exp:5d}µs: detect {detect_hz:.1f}/{frame_hz:.1f} Hz "
                f"({det_frac*100:.0f}%) sat {row['sat_mean']:.0f} "
                f"lat {row['latency_mean']:.0f}±{row['latency_std']:.0f} ms [saved]")
    except KeyboardInterrupt:
        node.get_logger().warn("interrupted — restoring exposure + saving partial.")
    finally:
        node.set_exposure(orig_exp, orig_iso)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f"exposure restored to {orig_exp} µs / ISO {orig_iso}.")

    if not rows:
        node.get_logger().error("no rows captured.")
        node.destroy_node(); rclpy.shutdown(); sys.exit(3)

    # recommendation: shortest exposure whose detect_hz >= floor*best
    best = max(r['detect_hz'] for r in rows)
    floor = args.detect_floor_frac * best
    ok_rows = [r for r in rows if r['detect_hz'] >= floor]
    rec = min(ok_rows, key=lambda r: r['exp_us']) if ok_rows else None

    summary = {
        'kind': 'exposure_sweep', 'utc': ts,
        'iso': iso, 'exposures_us': exposures,
        'oak_config_at_start': cfg,
        'original_exp_us': orig_exp, 'original_iso': orig_iso,
        'best_detect_hz': best,
        'recommended_exp_us': (rec['exp_us'] if rec else None),
        'recommendation_note': (
            f"shortest exposure holding detect_hz >= {args.detect_floor_frac:.0%} "
            f"of best ({best:.1f} Hz). Verify saturation stays above the HSV "
            "gate and re-test detection under a MOVING ball before adopting."),
        'rows': rows,
    }
    with open(os.path.join(out_dir, 'exposure_sweep_summary.yaml'), 'w') as f:
        if yaml is not None:
            yaml.safe_dump(summary, f, sort_keys=False)
        else:
            json.dump(summary, f, indent=2)

    print("\n=== EXPOSURE SWEEP ===")
    for r in rows:
        print(f"  {r['exp_us']:5d}µs ISO{r['iso']}: detect {r['detect_hz']:5.1f} Hz "
              f"({r['detect_frac']*100:3.0f}%)  sat {r['sat_mean']:5.0f}  "
              f"lat {r['latency_mean']:.0f}±{r['latency_std']:.0f} ms")
    if rec:
        print(f"  → recommended: {rec['exp_us']} µs (detect {rec['detect_hz']:.1f} Hz, "
              f"lat {rec['latency_mean']:.0f} ms)")
    print(f"  written → {out_dir}\n")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
