#!/usr/bin/env python3
"""digest_vision_bag.py — analyze a vision-debug bag.

Reads a rosbag2 mcap directory written by gui_server's /vision/start
endpoint, then produces:

  <bag>/digest.png          — keyframe mosaic + diagnostic plots
  <bag>/digest.summary.json — detection-rate + plane-offset stats

The push flow (gui_server._push_vision_bag_to_git) ships ONLY these
two files — the raw .mcap stays on the Pi. A 30 s vision-debug
recording with the RGB stream + the depth_blob debug overlay is
70+ MB; the digest is ~1 MB, which is what we actually care about
for diagnosis.

Topics consumed:

  /oak/depth_blob/debug_image   CompressedImage   (annotated overlay)
  /oak/ball/v0/diagnostic       Float32MultiArray [cx, cy, r, conf]
  /oak/ball/depth/diagnostic    Float32MultiArray
                                  [cx, cy, area, conf, plane_offset_mm]
  /oak/ball/v0/rgb_pixel        PointStamped      (V0 detection)
  /oak/ball/depth/rgb_pixel     PointStamped      (depth-blob detection)
  /ball_xy_mono                 PointStamped      (V0 → platform mm)
  /ball_xy_depth                PointStamped      (depth → platform mm)
  /platform_pose/markers_visible Int32

Other topics in the bag (RGB stream, IMU, control_*, status, etc.)
are listed in the topic-message-count summary but not plotted —
they're there for replay in foxglove if needed.

Usage:
  python3 digest_vision_bag.py <bag_directory>

Run on the Pi (or any machine with ROS 2 Kilted sourced — needs
rosbag2_py + the message types). matplotlib + opencv-python are
required for the PNG output.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
except ImportError as e:
    print(f"ERROR: rosbag2_py / rclpy unavailable: {e}", file=sys.stderr)
    print("       Source ROS first: source /opt/ros/kilted/setup.bash",
          file=sys.stderr)
    sys.exit(2)

try:
    from std_msgs.msg import Float32MultiArray, String, Int32, Float32
    from geometry_msgs.msg import PoseStamped, PointStamped
    from sensor_msgs.msg import CompressedImage
    try:
        from sensor_msgs.msg import Imu
    except ImportError:
        Imu = None
except ImportError as e:
    print(f"ERROR: ROS message types missing: {e}", file=sys.stderr)
    sys.exit(2)


def _open_bag(bag_dir: str):
    """Open a rosbag2 SequentialReader, falling back to explicit
    .mcap file URIs if the directory-mode auto-detect fails. Same
    pattern as digest_iva_bag.py."""
    storage = StorageOptions(uri=bag_dir, storage_id='')
    converter = ConverterOptions('', '')
    reader = SequentialReader()
    try:
        reader.open(storage, converter)
        return reader
    except Exception as e_auto:
        mcap_files = sorted(p for p in os.listdir(bag_dir)
                            if p.endswith('.mcap'))
        last_err = e_auto
        for mf in mcap_files:
            try:
                storage2 = StorageOptions(
                    uri=os.path.join(bag_dir, mf), storage_id='mcap')
                reader = SequentialReader()
                reader.open(storage2, converter)
                return reader
            except Exception as e_file:
                last_err = e_file
                continue
        raise RuntimeError(
            f"could not open bag at {bag_dir} (auto: {e_auto}; "
            f"file fallback: {last_err})")


# Topic → ROS message class.  Float-array, point, image, status etc.
_TYPE_CLASS = {
    'std_msgs/msg/Float32MultiArray': Float32MultiArray,
    'std_msgs/msg/Float32':           Float32,
    'std_msgs/msg/String':            String,
    'std_msgs/msg/Int32':             Int32,
    'geometry_msgs/msg/PoseStamped':  PoseStamped,
    'geometry_msgs/msg/PointStamped': PointStamped,
    'sensor_msgs/msg/CompressedImage': CompressedImage,
}
if Imu is not None:
    _TYPE_CLASS['sensor_msgs/msg/Imu'] = Imu


def _read_bag(bag_dir: str):
    reader = _open_bag(bag_dir)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    # Per-topic accumulators we'll need for plots / stats.
    counts = {n: 0 for n in topic_types}
    debug_imgs: List[Tuple[int, bytes]] = []      # (t_ns, jpeg bytes)
    v0_diag_t, v0_diag = [], []                   # cx, cy, r, conf
    dp_diag_t, dp_diag = [], []                   # cx, cy, area, conf, plane_offset
    v0_pix_t, v0_pix   = [], []                   # cx, cy
    dp_pix_t, dp_pix   = [], []
    bxy_mono_t, bxy_mono = [], []                 # x, y mm
    bxy_depth_t, bxy_depth = [], []
    mark_t, mark_n = [], []
    pose_t = []                                   # just stamps (for window)

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        ttype = topic_types.get(topic)
        cls = _TYPE_CLASS.get(ttype)
        if cls is None:
            continue
        try:
            msg = deserialize_message(raw, cls)
        except Exception:
            continue

        if topic == '/oak/depth_blob/debug_image':
            # Keep only the raw bytes — decode lazily when sampling.
            debug_imgs.append((t_ns, bytes(msg.data)))
        elif topic == '/oak/ball/v0/diagnostic':
            d = list(msg.data)
            if len(d) >= 4:
                v0_diag_t.append(t_ns)
                v0_diag.append([float(d[0]), float(d[1]),
                                float(d[2]), float(d[3])])
        elif topic == '/oak/ball/depth/diagnostic':
            d = list(msg.data)
            if len(d) >= 4:
                # plane_offset is the 5th field (added 2026-04-29);
                # older bags only have 4 fields. Fill in NaN so the
                # plot just shows gaps.
                po = float(d[4]) if len(d) >= 5 else float('nan')
                dp_diag_t.append(t_ns)
                dp_diag.append([float(d[0]), float(d[1]),
                                float(d[2]), float(d[3]), po])
        elif topic == '/oak/ball/v0/rgb_pixel':
            v0_pix_t.append(t_ns)
            v0_pix.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/oak/ball/depth/rgb_pixel':
            dp_pix_t.append(t_ns)
            dp_pix.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/ball_xy_mono':
            bxy_mono_t.append(t_ns)
            bxy_mono.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/ball_xy_depth':
            bxy_depth_t.append(t_ns)
            bxy_depth.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/platform_pose/markers_visible':
            mark_t.append(t_ns)
            mark_n.append(int(msg.data))
        elif topic == '/platform_pose':
            pose_t.append(t_ns)

    return {
        'topic_types': topic_types,
        'counts': counts,
        'debug_imgs': debug_imgs,
        'v0_diag': (np.array(v0_diag_t,  dtype=np.int64),
                    np.array(v0_diag) if v0_diag else np.zeros((0, 4))),
        'dp_diag': (np.array(dp_diag_t,  dtype=np.int64),
                    np.array(dp_diag) if dp_diag else np.zeros((0, 5))),
        'v0_pix':  (np.array(v0_pix_t,   dtype=np.int64),
                    np.array(v0_pix)  if v0_pix  else np.zeros((0, 2))),
        'dp_pix':  (np.array(dp_pix_t,   dtype=np.int64),
                    np.array(dp_pix)  if dp_pix  else np.zeros((0, 2))),
        'bxy_mono':  (np.array(bxy_mono_t,  dtype=np.int64),
                      np.array(bxy_mono)  if bxy_mono  else np.zeros((0, 2))),
        'bxy_depth': (np.array(bxy_depth_t, dtype=np.int64),
                      np.array(bxy_depth) if bxy_depth else np.zeros((0, 2))),
        'mark': (np.array(mark_t, dtype=np.int64),
                 np.array(mark_n, dtype=np.int32) if mark_n else np.zeros((0,), dtype=np.int32)),
        'pose_t': np.array(pose_t, dtype=np.int64) if pose_t else np.zeros((0,), dtype=np.int64),
    }


def _stats(arr: np.ndarray):
    arr = np.asarray(arr, dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {'n': 0}
    return {
        'n':       int(arr.size),
        'mean':    float(np.mean(arr)),
        'std':     float(np.std(arr)),
        'p50':     float(np.percentile(arr, 50)),
        'p95':     float(np.percentile(np.abs(arr), 95)),
        'min':     float(np.min(arr)),
        'max':     float(np.max(arr)),
        'max_abs': float(np.max(np.abs(arr))),
    }


def _detection_rate_hz(t_ns: np.ndarray, t0_ns: int, t1_ns: int,
                       window_s: float = 1.0):
    """Compute a rolling-window detection rate (Hz) at 10 Hz output
    granularity over the bag's duration. Returns (t_seconds, rate_hz)."""
    if t1_ns <= t0_ns or t_ns.size == 0:
        return np.zeros((0,)), np.zeros((0,))
    duration = (t1_ns - t0_ns) * 1e-9
    n_grid = max(1, int(duration * 10.0))
    grid_s = np.linspace(0.0, duration, n_grid)
    t_rel_s = (t_ns - t0_ns) * 1e-9
    half = window_s * 0.5
    rate = np.zeros_like(grid_s)
    j_lo = 0
    j_hi = 0
    sorted_t = np.sort(t_rel_s)
    for k, g in enumerate(grid_s):
        lo = g - half
        hi = g + half
        while j_lo < sorted_t.size and sorted_t[j_lo] < lo:
            j_lo += 1
        while j_hi < sorted_t.size and sorted_t[j_hi] < hi:
            j_hi += 1
        rate[k] = (j_hi - j_lo) / window_s
    return grid_s, rate


def _largest_gap_s(t_ns: np.ndarray):
    if t_ns.size < 2:
        return 0.0
    diffs = np.diff(np.sort(t_ns)) * 1e-9
    return float(np.max(diffs))


def digest(bag_dir: str):
    print(f"[vision-digest] reading {bag_dir}")
    data = _read_bag(bag_dir)

    counts: dict = data['counts']
    debug_imgs: List[Tuple[int, bytes]] = data['debug_imgs']
    v0_diag_t, v0_diag = data['v0_diag']
    dp_diag_t, dp_diag = data['dp_diag']
    v0_pix_t,  v0_pix  = data['v0_pix']
    dp_pix_t,  dp_pix  = data['dp_pix']
    bxy_mono_t, bxy_mono   = data['bxy_mono']
    bxy_depth_t, bxy_depth = data['bxy_depth']
    mark_t, mark_n = data['mark']
    pose_t = data['pose_t']

    # Bag-relative time origin: earliest stamp seen across the
    # topics we care about. Falls back to debug_imgs / pose_t if the
    # detector channels are silent throughout.
    t0_candidates = []
    for arr in (v0_diag_t, dp_diag_t, v0_pix_t, dp_pix_t,
                bxy_mono_t, bxy_depth_t, mark_t, pose_t):
        if arr.size:
            t0_candidates.append(int(arr[0]))
    if debug_imgs:
        t0_candidates.append(int(debug_imgs[0][0]))
    if not t0_candidates:
        raise RuntimeError(
            "bag is empty for all expected topics — cannot digest.")
    t0 = min(t0_candidates)

    t1_candidates = []
    for arr in (v0_diag_t, dp_diag_t, v0_pix_t, dp_pix_t,
                bxy_mono_t, bxy_depth_t, mark_t, pose_t):
        if arr.size:
            t1_candidates.append(int(arr[-1]))
    if debug_imgs:
        t1_candidates.append(int(debug_imgs[-1][0]))
    t1 = max(t1_candidates) if t1_candidates else t0
    duration_s = (t1 - t0) * 1e-9

    # Detection rate (Hz) per detector over the whole duration.
    v0_t_grid, v0_rate = _detection_rate_hz(v0_pix_t, t0, t1)
    dp_t_grid, dp_rate = _detection_rate_hz(dp_pix_t, t0, t1)

    # V0 and depth pixel agreement, when both fire near-simultaneously
    # (within 100 ms). We linearly interpolate the depth pixel onto the
    # V0 timestamps for a fair comparison.
    pixel_disagreement_px = np.zeros((0,), dtype=np.float64)
    if v0_pix_t.size and dp_pix_t.size:
        dp_x = np.interp((v0_pix_t - t0) * 1e-9,
                         (dp_pix_t - t0) * 1e-9, dp_pix[:, 0],
                         left=np.nan, right=np.nan)
        dp_y = np.interp((v0_pix_t - t0) * 1e-9,
                         (dp_pix_t - t0) * 1e-9, dp_pix[:, 1],
                         left=np.nan, right=np.nan)
        dx = v0_pix[:, 0] - dp_x
        dy = v0_pix[:, 1] - dp_y
        d = np.sqrt(dx * dx + dy * dy)
        d = d[np.isfinite(d)]
        pixel_disagreement_px = d

    # Same for platform-frame mm.
    mm_disagreement_mm = np.zeros((0,), dtype=np.float64)
    if bxy_mono_t.size and bxy_depth_t.size:
        dx = np.interp((bxy_mono_t - t0) * 1e-9,
                       (bxy_depth_t - t0) * 1e-9, bxy_depth[:, 0],
                       left=np.nan, right=np.nan)
        dy = np.interp((bxy_mono_t - t0) * 1e-9,
                       (bxy_depth_t - t0) * 1e-9, bxy_depth[:, 1],
                       left=np.nan, right=np.nan)
        d = np.sqrt((bxy_mono[:, 0] - dx) ** 2
                    + (bxy_mono[:, 1] - dy) ** 2)
        d = d[np.isfinite(d)]
        mm_disagreement_mm = d

    summary = {
        'bag': os.path.abspath(bag_dir),
        'duration_s': duration_s,
        'topic_counts': dict(sorted(counts.items())),
        'v0': {
            'detections_n': int(v0_pix_t.size),
            'detection_rate_hz_mean': float(v0_rate.mean()) if v0_rate.size else 0.0,
            'detection_rate_hz_min':  float(v0_rate.min())  if v0_rate.size else 0.0,
            'detection_rate_hz_max':  float(v0_rate.max())  if v0_rate.size else 0.0,
            'largest_silence_s': _largest_gap_s(v0_pix_t),
        },
        'depth_blob': {
            'detections_n': int(dp_pix_t.size),
            'detection_rate_hz_mean': float(dp_rate.mean()) if dp_rate.size else 0.0,
            'detection_rate_hz_min':  float(dp_rate.min())  if dp_rate.size else 0.0,
            'detection_rate_hz_max':  float(dp_rate.max())  if dp_rate.size else 0.0,
            'largest_silence_s': _largest_gap_s(dp_pix_t),
            # plane_offset_mm column: index 4 in the diagnostic
            'plane_offset_mm': _stats(dp_diag[:, 4] if dp_diag.size else
                                      np.zeros((0,))),
            # area_px column: index 2
            'area_px': _stats(dp_diag[:, 2] if dp_diag.size else
                              np.zeros((0,))),
            'confidence': _stats(dp_diag[:, 3] if dp_diag.size else
                                 np.zeros((0,))),
        },
        'platform_frame_mm': {
            'v0_xy_n': int(bxy_mono_t.size),
            'depth_xy_n': int(bxy_depth_t.size),
            'detector_disagreement_mm': _stats(mm_disagreement_mm),
        },
        'pixel_space': {
            'detector_disagreement_px': _stats(pixel_disagreement_px),
        },
        'markers_visible': {
            'n': int(mark_t.size),
            'mean': float(np.mean(mark_n)) if mark_n.size else None,
            'min':  int(np.min(mark_n))    if mark_n.size else None,
            'max':  int(np.max(mark_n))    if mark_n.size else None,
        },
        'debug_image_frames': len(debug_imgs),
    }

    print(f"[vision-digest] duration={duration_s:.1f}s "
          f"V0={summary['v0']['detections_n']} "
          f"depth={summary['depth_blob']['detections_n']}")
    if dp_diag.size:
        po = summary['depth_blob']['plane_offset_mm']
        print(f"  depth plane_offset_mm: mean={po.get('mean', 0):+.1f} "
              f"std={po.get('std', 0):.1f} max|·|={po.get('max_abs', 0):.1f}")
    md = summary['platform_frame_mm']['detector_disagreement_mm']
    if md.get('n', 0):
        print(f"  V0 vs depth (platform mm): mean={md['mean']:.1f} "
              f"std={md['std']:.1f} p95={md['p95']:.1f}")

    # ---- Plot ---------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
    except ImportError:
        print("[vision-digest] matplotlib not installed; skipping PNG.",
              file=sys.stderr)
        plt = None
    try:
        import cv2
    except ImportError:
        cv2 = None

    if plt is not None:
        # 9-frame mosaic of debug overlay if available; else
        # 3-row diagnostic plots only.
        n_keyframes = min(9, len(debug_imgs))
        if n_keyframes >= 1 and cv2 is not None:
            fig = plt.figure(figsize=(13, 14))
            gs = GridSpec(5, 3, figure=fig,
                          height_ratios=[2.0, 2.0, 2.0, 1.4, 1.4],
                          hspace=0.4, wspace=0.15)
            # Mosaic occupies rows 0–1.
            idxs = np.linspace(0, len(debug_imgs) - 1,
                               n_keyframes).astype(int)
            for k, idx in enumerate(idxs):
                ax = fig.add_subplot(gs[k // 3, k % 3])
                t_ns, jpg = debug_imgs[idx]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_title(f't={(t_ns - t0) * 1e-9:.1f}s',
                             fontsize=8)
            ts_row = 2
        else:
            fig = plt.figure(figsize=(13, 9))
            gs = GridSpec(3, 1, figure=fig, hspace=0.45)
            ts_row = 0

        # Detection rate (Hz)
        ax = fig.add_subplot(gs[ts_row, :] if n_keyframes >= 1 else gs[0])
        if v0_rate.size:
            ax.plot(v0_t_grid, v0_rate, color='#fb923c', lw=1.2,
                    label='V0 (color)')
        if dp_rate.size:
            ax.plot(dp_t_grid, dp_rate, color='#22d3ee', lw=1.2,
                    label='depth-blob')
        ax.set_ylabel('rate [Hz]')
        ax.set_title('Detection rate (1 s rolling window)', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)

        # Plane offset (mm) over time + above_px
        ax = fig.add_subplot(gs[ts_row + 1, :] if n_keyframes >= 1 else gs[1])
        if dp_diag.size:
            ax.plot((dp_diag_t - t0) * 1e-9, dp_diag[:, 4],
                    color='#a78bfa', lw=1.0, label='plane_offset_mm')
            ax.set_ylabel('mm', color='#a78bfa')
            ax2 = ax.twinx()
            ax2.plot((dp_diag_t - t0) * 1e-9, dp_diag[:, 2],
                     color='#f87171', lw=0.7, label='area_px')
            ax2.set_ylabel('blob area [px]', color='#f87171')
            ax.legend(loc='upper left', fontsize=8)
            ax2.legend(loc='upper right', fontsize=8)
        else:
            ax.text(0.5, 0.5, '(no depth-blob diagnostics)',
                    ha='center', va='center', transform=ax.transAxes,
                    color='#888')
        ax.set_title(
            'Depth-blob plane offset & blob area (per detection)',
            fontsize=10)
        ax.grid(alpha=0.3)

        # Pixel-space scatter (V0 vs depth)
        ax = fig.add_subplot(gs[ts_row + 2, 0:2] if n_keyframes >= 1
                             else gs[2])
        if v0_pix.size:
            ax.scatter(v0_pix[:, 0], v0_pix[:, 1], s=4,
                       color='#fb923c', alpha=0.4, label='V0')
        if dp_pix.size:
            ax.scatter(dp_pix[:, 0], dp_pix[:, 1], s=4,
                       color='#22d3ee', alpha=0.4, label='depth')
        # 960×540 RGB-ISP frame; flip Y so image-up is plot-up.
        ax.set_xlim(0, 960)
        ax.set_ylim(540, 0)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Detected pixels (image space)', fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

        # Platform-frame scatter (mm)
        if n_keyframes >= 1:
            ax = fig.add_subplot(gs[ts_row + 2, 2])
        else:
            # If we used the simpler 3-row layout, reuse the spot below
            # — we only have 3 rows in that branch, so plot side-by-side
            # would need a different setup. Skip in that case.
            ax = None
        if ax is not None:
            if bxy_mono.size:
                ax.scatter(bxy_mono[:, 0], bxy_mono[:, 1], s=4,
                           color='#fb923c', alpha=0.4, label='V0')
            if bxy_depth.size:
                ax.scatter(bxy_depth[:, 0], bxy_depth[:, 1], s=4,
                           color='#22d3ee', alpha=0.4, label='depth')
            # Platform disk for reference
            theta = np.linspace(0, 2 * np.pi, 200)
            ax.plot(220 * np.cos(theta), 220 * np.sin(theta),
                    color='#475569', lw=0.7, label='gate r=220mm')
            ax.plot(200 * np.cos(theta), 200 * np.sin(theta),
                    color='#94a3b8', lw=0.5, ls='--', alpha=0.6)
            ax.set_aspect('equal', adjustable='box')
            ax.set_xlim(-260, 260)
            ax.set_ylim(-260, 260)
            ax.set_title('Platform-frame ball xy (mm)', fontsize=10)
            ax.legend(fontsize=8)
            ax.grid(alpha=0.3)

        title = f"vision-debug — {os.path.basename(bag_dir)} ({duration_s:.1f}s)"
        sub = (f"V0={summary['v0']['detections_n']}  "
               f"depth={summary['depth_blob']['detections_n']}")
        po = summary['depth_blob']['plane_offset_mm']
        if po.get('n'):
            sub += (f"  |  plane_offset mean={po.get('mean', 0):+.1f}mm "
                    f"max|·|={po.get('max_abs', 0):.1f}mm")
        md = summary['platform_frame_mm']['detector_disagreement_mm']
        if md.get('n', 0):
            sub += f"  |  V0↔depth p95={md['p95']:.1f}mm"
        fig.suptitle(title + '\n' + sub, fontsize=10)
        png = os.path.join(bag_dir, 'digest.png')
        fig.savefig(png, dpi=110, bbox_inches='tight')
        plt.close(fig)
        print(f"[vision-digest] wrote {png}")

    js = os.path.join(bag_dir, 'digest.summary.json')
    with open(js, 'w') as f:
        json.dump(summary, f, indent=2)
        f.write('\n')
    print(f"[vision-digest] wrote {js}")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag', help='Path to the bag directory')
    args = p.parse_args()
    if not os.path.isdir(args.bag):
        print(f"ERROR: not a directory: {args.bag}", file=sys.stderr)
        return 2
    digest(args.bag)
    return 0


if __name__ == '__main__':
    sys.exit(main())
