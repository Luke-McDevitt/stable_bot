#!/usr/bin/env python3
"""digest_vision_bag.py — analyze a vision-debug bag.

Reads a rosbag2 mcap directory written by gui_server's /vision/start
endpoint and produces:

  <bag>/digest.png          — keyframe mosaic + multi-panel diagnostic plots
  <bag>/digest.summary.json — aggregate stats per signal

Only the digest artifacts are committed to git (push button in the GUI
forces them). The raw bag stays on the Pi — typically tens of MB once
RGB and the depth-blob debug overlay are flowing.

Topics consumed:

  /oak/depth_blob/debug_image   CompressedImage   (annotated overlay)
  /oak/ball/v0/diagnostic       Float32MultiArray [cx, cy, r, conf]
  /oak/ball/depth/diagnostic    Float32MultiArray (RICH — see below)
  /oak/ball/v0/rgb_pixel        PointStamped
  /oak/ball/depth/rgb_pixel     PointStamped
  /ball_xy_mono                 PointStamped      (V0 → platform mm)
  /ball_xy_depth                PointStamped      (depth → platform mm)
  /platform_pose                PoseStamped       (ArUco)
  /platform_pose/markers_visible Int32

The rich 20-field /oak/ball/depth/diagnostic schema (see
oak_driver_node._tick depth-blob block, 2026-04-29):

   0..4   detection-only fields (NaN/0 when no detection):
            cx, cy, area_px, confidence, plane_offset_mm
   5..10  algorithm internals (every depth frame):
            mask_pixels, eroded_pixels, valid_in_eroded, n_above,
            max_height_mm, median_height_in_mask_mm
  11..12  largest_blob_area, fail_code
            (fail_code 0=ok, 1=no_pixels_above, 2=no_components,
                       3=blob_below_min_area, 4=cv2_missing, 9=other)
  13..15  probe-style: pose_z_mm, exp_center_mm, meas_center_mm
  16..21  frame-wide depth distribution (mm + %):
            depth_min, depth_p25, depth_median, depth_p75, depth_max,
            depth_invalid_pct

Older bags use the 5-field legacy schema; the digest detects the
length and degrades gracefully (legacy fields plotted, new panels
labelled "(legacy bag)").
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


# Map fail_code int → name for plot legends + summary.
FAIL_CODE_NAMES = {
    0: 'ok',
    1: 'no_pixels_above',
    2: 'no_components',
    3: 'blob_below_min_area',
    4: 'cv2_missing',
    9: 'other',
}
FAIL_CODE_COLORS = {
    0: '#10b981',   # emerald
    1: '#f87171',   # red
    2: '#fbbf24',   # amber
    3: '#a78bfa',   # purple
    4: '#94a3b8',   # slate
    9: '#475569',
}


def _open_bag(bag_dir: str):
    """Open rosbag2 reader with directory→file fallback."""
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
    counts = {n: 0 for n in topic_types}

    debug_imgs: List[Tuple[int, bytes]] = []
    v0_diag_t, v0_diag = [], []           # [cx, cy, r, conf]
    dp_diag_t, dp_diag_rows = [], []      # rich row, length 5..22
    v0_pix_t, v0_pix = [], []
    dp_pix_t, dp_pix = [], []
    bxy_mono_t, bxy_mono = [], []
    bxy_depth_t, bxy_depth = [], []
    mark_t, mark_n = [], []
    pose_t, pose_z = [], []

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
            debug_imgs.append((t_ns, bytes(msg.data)))
        elif topic == '/oak/ball/v0/diagnostic':
            d = list(msg.data)
            if len(d) >= 4:
                v0_diag_t.append(t_ns)
                v0_diag.append([float(d[0]), float(d[1]),
                                float(d[2]), float(d[3])])
        elif topic == '/oak/ball/depth/diagnostic':
            d = list(msg.data)
            if len(d) >= 5:
                dp_diag_t.append(t_ns)
                dp_diag_rows.append([float(x) for x in d])
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
            pose_z.append(float(msg.pose.position.z) * 1000.0)

    # Pad rich-diag rows to the max width seen so np.array works.
    if dp_diag_rows:
        maxw = max(len(r) for r in dp_diag_rows)
        padded = [r + [float('nan')] * (maxw - len(r)) for r in dp_diag_rows]
        dp_diag = np.array(padded, dtype=np.float64)
    else:
        dp_diag = np.zeros((0, 22), dtype=np.float64)

    return {
        'topic_types': topic_types,
        'counts': counts,
        'debug_imgs': debug_imgs,
        'v0_diag': (np.array(v0_diag_t, dtype=np.int64),
                    np.array(v0_diag) if v0_diag else np.zeros((0, 4))),
        'dp_diag': (np.array(dp_diag_t, dtype=np.int64), dp_diag),
        'v0_pix': (np.array(v0_pix_t, dtype=np.int64),
                   np.array(v0_pix) if v0_pix else np.zeros((0, 2))),
        'dp_pix': (np.array(dp_pix_t, dtype=np.int64),
                   np.array(dp_pix) if dp_pix else np.zeros((0, 2))),
        'bxy_mono':  (np.array(bxy_mono_t, dtype=np.int64),
                      np.array(bxy_mono) if bxy_mono else np.zeros((0, 2))),
        'bxy_depth': (np.array(bxy_depth_t, dtype=np.int64),
                      np.array(bxy_depth) if bxy_depth else np.zeros((0, 2))),
        'mark': (np.array(mark_t, dtype=np.int64),
                 np.array(mark_n, dtype=np.int32) if mark_n
                 else np.zeros((0,), dtype=np.int32)),
        'pose': (np.array(pose_t, dtype=np.int64),
                 np.array(pose_z, dtype=np.float64)),
    }


def _stats(arr) -> dict:
    """Stats summary, robust to NaN."""
    a = np.asarray(arr, dtype=np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return {'n': 0}
    return {
        'n':       int(a.size),
        'mean':    float(np.mean(a)),
        'std':     float(np.std(a)),
        'p05':     float(np.percentile(a, 5)),
        'p50':     float(np.percentile(a, 50)),
        'p95':     float(np.percentile(a, 95)),
        'min':     float(np.min(a)),
        'max':     float(np.max(a)),
    }


def _rate_hz(t_ns, t0_ns, t1_ns, window_s=1.0):
    if t1_ns <= t0_ns or len(t_ns) == 0:
        return np.zeros((0,)), np.zeros((0,))
    duration = (t1_ns - t0_ns) * 1e-9
    n_grid = max(1, int(duration * 10.0))
    grid_s = np.linspace(0.0, duration, n_grid)
    t_rel_s = (np.asarray(t_ns) - t0_ns) * 1e-9
    half = window_s * 0.5
    rate = np.zeros_like(grid_s)
    sorted_t = np.sort(t_rel_s)
    j_lo = j_hi = 0
    for k, g in enumerate(grid_s):
        lo, hi = g - half, g + half
        while j_lo < sorted_t.size and sorted_t[j_lo] < lo:
            j_lo += 1
        while j_hi < sorted_t.size and sorted_t[j_hi] < hi:
            j_hi += 1
        rate[k] = (j_hi - j_lo) / window_s
    return grid_s, rate


def _largest_gap_s(t_ns):
    if len(t_ns) < 2:
        return 0.0
    return float(np.max(np.diff(np.sort(t_ns))) * 1e-9)


def _fail_histogram(dp_diag: np.ndarray) -> dict:
    """Count occurrences of each fail_code value."""
    if dp_diag.shape[0] == 0 or dp_diag.shape[1] < 13:
        return {}
    codes = dp_diag[:, 12].astype(int)
    out = {}
    for c in np.unique(codes):
        c = int(c)
        n = int(np.sum(codes == c))
        out[FAIL_CODE_NAMES.get(c, f'code_{c}')] = n
    return out


def digest(bag_dir: str):
    print(f"[vision-digest] reading {bag_dir}")
    data = _read_bag(bag_dir)

    counts = data['counts']
    debug_imgs = data['debug_imgs']
    v0_diag_t, v0_diag = data['v0_diag']
    dp_diag_t, dp_diag = data['dp_diag']
    v0_pix_t, v0_pix = data['v0_pix']
    dp_pix_t, dp_pix = data['dp_pix']
    bxy_mono_t, bxy_mono = data['bxy_mono']
    bxy_depth_t, bxy_depth = data['bxy_depth']
    mark_t, mark_n = data['mark']
    pose_t, pose_z = data['pose']

    # Bag-relative time origin.
    t0_cands = []
    for arr in (v0_diag_t, dp_diag_t, v0_pix_t, dp_pix_t,
                bxy_mono_t, bxy_depth_t, mark_t, pose_t):
        if len(arr):
            t0_cands.append(int(arr[0]))
    if debug_imgs:
        t0_cands.append(int(debug_imgs[0][0]))
    if not t0_cands:
        raise RuntimeError("bag is empty for all expected topics")
    t0 = min(t0_cands)

    t1_cands = []
    for arr in (v0_diag_t, dp_diag_t, v0_pix_t, dp_pix_t,
                bxy_mono_t, bxy_depth_t, mark_t, pose_t):
        if len(arr):
            t1_cands.append(int(arr[-1]))
    if debug_imgs:
        t1_cands.append(int(debug_imgs[-1][0]))
    t1 = max(t1_cands) if t1_cands else t0
    duration_s = (t1 - t0) * 1e-9

    rich = (dp_diag.shape[1] >= 22) if dp_diag.shape[0] else False

    # ---- Detection-rate stats ----
    v0_t_grid, v0_rate = _rate_hz(v0_pix_t, t0, t1)
    dp_t_grid, dp_rate = _rate_hz(dp_pix_t, t0, t1)
    df_t_grid, df_rate = _rate_hz(dp_diag_t, t0, t1)

    # ---- V0↔depth disagreement (when both fire near-simultaneously) ----
    pixel_dis = np.zeros((0,), dtype=np.float64)
    if v0_pix_t.size and dp_pix_t.size:
        dpx = np.interp((v0_pix_t - t0) * 1e-9,
                        (dp_pix_t - t0) * 1e-9, dp_pix[:, 0],
                        left=np.nan, right=np.nan)
        dpy = np.interp((v0_pix_t - t0) * 1e-9,
                        (dp_pix_t - t0) * 1e-9, dp_pix[:, 1],
                        left=np.nan, right=np.nan)
        d = np.sqrt((v0_pix[:, 0] - dpx) ** 2
                    + (v0_pix[:, 1] - dpy) ** 2)
        pixel_dis = d[np.isfinite(d)]

    mm_dis = np.zeros((0,), dtype=np.float64)
    if bxy_mono_t.size and bxy_depth_t.size:
        dx = np.interp((bxy_mono_t - t0) * 1e-9,
                       (bxy_depth_t - t0) * 1e-9, bxy_depth[:, 0],
                       left=np.nan, right=np.nan)
        dy = np.interp((bxy_mono_t - t0) * 1e-9,
                       (bxy_depth_t - t0) * 1e-9, bxy_depth[:, 1],
                       left=np.nan, right=np.nan)
        d = np.sqrt((bxy_mono[:, 0] - dx) ** 2
                    + (bxy_mono[:, 1] - dy) ** 2)
        mm_dis = d[np.isfinite(d)]

    # Per-rich-diag-field stats.
    rich_stats = {}
    if rich:
        # Field index → name
        field_names = [
            'cx', 'cy', 'area_px', 'confidence', 'plane_offset_mm',
            'mask_pixels', 'eroded_pixels', 'valid_in_eroded', 'n_above',
            'max_height_mm', 'median_height_in_mask_mm',
            'largest_blob_area', 'fail_code',
            'pose_z_mm', 'exp_center_mm', 'meas_center_mm',
            'depth_min_mm', 'depth_p25_mm', 'depth_median_mm',
            'depth_p75_mm', 'depth_max_mm', 'depth_invalid_pct',
        ]
        for i, name in enumerate(field_names):
            if i < dp_diag.shape[1]:
                rich_stats[name] = _stats(dp_diag[:, i])

    summary = {
        'bag': os.path.abspath(bag_dir),
        'duration_s': duration_s,
        'topic_counts': dict(sorted(counts.items())),
        'rich_diagnostic_present': rich,
        'v0': {
            'detections_n': int(v0_pix_t.size),
            'rate_hz_mean': float(v0_rate.mean()) if v0_rate.size else 0.0,
            'rate_hz_max':  float(v0_rate.max())  if v0_rate.size else 0.0,
            'largest_silence_s': _largest_gap_s(v0_pix_t),
        },
        'depth_blob': {
            'detections_n': int(dp_pix_t.size),
            'rate_hz_mean': float(dp_rate.mean()) if dp_rate.size else 0.0,
            'rate_hz_max':  float(dp_rate.max())  if dp_rate.size else 0.0,
            'largest_silence_s': _largest_gap_s(dp_pix_t),
            'frame_diag_n': int(dp_diag_t.size),
            'fail_histogram': _fail_histogram(dp_diag),
        },
        'platform_frame_mm': {
            'v0_xy_n': int(bxy_mono_t.size),
            'depth_xy_n': int(bxy_depth_t.size),
            'detector_disagreement_mm': _stats(mm_dis),
        },
        'pixel_space': {
            'detector_disagreement_px': _stats(pixel_dis),
        },
        'markers_visible': {
            'n': int(mark_t.size),
            'mean': float(np.mean(mark_n)) if mark_n.size else None,
            'min':  int(np.min(mark_n)) if mark_n.size else None,
            'max':  int(np.max(mark_n)) if mark_n.size else None,
        },
        'platform_pose_z_mm': _stats(pose_z),
        'rich_diagnostic_stats': rich_stats,
        'debug_image_frames': len(debug_imgs),
    }

    print(f"[vision-digest] duration={duration_s:.1f}s "
          f"V0={summary['v0']['detections_n']} "
          f"depth={summary['depth_blob']['detections_n']} "
          f"frame_diag={summary['depth_blob']['frame_diag_n']}")
    if rich and 'fail_histogram' in summary['depth_blob']:
        fh = summary['depth_blob']['fail_histogram']
        print(f"  fail histogram: {fh}")
        po = rich_stats.get('plane_offset_mm', {})
        if po.get('n'):
            print(f"  plane_offset_mm: mean={po['mean']:+.1f} "
                  f"std={po['std']:.1f} max|·|={max(abs(po['min']), abs(po['max'])):.1f}")
        mh = rich_stats.get('max_height_mm', {})
        if mh.get('n'):
            print(f"  max_height_mm: mean={mh['mean']:+.1f} "
                  f"max={mh['max']:.1f}")
        for axis in ('pose_z_mm', 'exp_center_mm', 'meas_center_mm'):
            s = rich_stats.get(axis, {})
            if s.get('n'):
                print(f"  {axis}: mean={s['mean']:.0f} std={s['std']:.0f}")
    md = summary['platform_frame_mm']['detector_disagreement_mm']
    if md.get('n', 0):
        print(f"  V0 vs depth (mm): mean={md['mean']:.1f} "
              f"std={md['std']:.1f} p95={md['p95']:.1f}")

    # ----- Plot -----
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
        n_keyframes = min(9, len(debug_imgs))
        # Layout: keyframe mosaic (3 rows, 3 cols if available) + 6
        # full-width time-series rows + scatter pair on a final row.
        # Heights tuned to keep mosaic frames readable.
        n_mosaic_rows = 3 if n_keyframes >= 7 else (
            2 if n_keyframes >= 4 else (1 if n_keyframes >= 1 else 0))
        n_ts_rows = 6 if rich else 2  # rich = more panels
        n_total_rows = n_mosaic_rows + n_ts_rows + 1  # + scatter row
        h_per_row = 2.2
        fig = plt.figure(figsize=(14, h_per_row * n_total_rows))
        height_ratios = ([2.2] * n_mosaic_rows
                         + [1.2] * n_ts_rows
                         + [2.4])
        gs = GridSpec(n_total_rows, 6, figure=fig,
                      height_ratios=height_ratios,
                      hspace=0.55, wspace=0.25)

        # --- Mosaic ---
        if n_keyframes >= 1 and cv2 is not None:
            idxs = np.linspace(0, len(debug_imgs) - 1,
                               n_keyframes).astype(int)
            for k, idx in enumerate(idxs):
                row = k // 3
                col = k % 3
                if row >= n_mosaic_rows:
                    break
                ax = fig.add_subplot(gs[row, col * 2:col * 2 + 2])
                t_ns, jpg = debug_imgs[idx]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                ax.set_xticks([])
                ax.set_yticks([])
                ax.set_title(f't={(t_ns - t0) * 1e-9:.1f}s', fontsize=8)

        ts_row = n_mosaic_rows

        # --- 1. Detection rate ---
        ax = fig.add_subplot(gs[ts_row, :])
        if v0_rate.size:
            ax.plot(v0_t_grid, v0_rate, color='#fb923c', lw=1.2,
                    label='V0 (color)')
        if dp_rate.size:
            ax.plot(dp_t_grid, dp_rate, color='#22d3ee', lw=1.2,
                    label='depth-blob')
        if df_rate.size:
            ax.plot(df_t_grid, df_rate, color='#94a3b8', lw=0.8,
                    alpha=0.6, label='depth-frame_diag')
        ax.set_ylabel('rate [Hz]')
        ax.set_title('Detection / diagnostic rate (1s rolling window)',
                     fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(loc='upper right', fontsize=8)
        ts_row += 1

        if rich:
            t_rel = (dp_diag_t - t0) * 1e-9

            # --- 2. fail_code over time + histogram ---
            ax = fig.add_subplot(gs[ts_row, :])
            codes = dp_diag[:, 12].astype(int)
            for c in np.unique(codes):
                m = codes == c
                ax.scatter(t_rel[m],
                           np.full(int(m.sum()), int(c)),
                           s=6,
                           color=FAIL_CODE_COLORS.get(int(c), '#475569'),
                           label=f"{FAIL_CODE_NAMES.get(int(c), '?')} ({int(m.sum())})")
            ax.set_yticks(list(FAIL_CODE_NAMES.keys()))
            ax.set_yticklabels([FAIL_CODE_NAMES[k] for k
                                in FAIL_CODE_NAMES.keys()],
                               fontsize=7)
            ax.set_ylabel('fail_reason')
            ax.set_title('Detector outcome per depth frame', fontsize=10)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right', fontsize=7, ncol=3)
            ts_row += 1

            # --- 3. heights: max_h / med_h / plane_offset ---
            ax = fig.add_subplot(gs[ts_row, :])
            ax.plot(t_rel, dp_diag[:, 9],  color='#22d3ee', lw=1.0,
                    label='max_height_mm')
            ax.plot(t_rel, dp_diag[:, 10], color='#a78bfa', lw=1.0,
                    label='median_height_in_mask_mm')
            ax.plot(t_rel, dp_diag[:, 4],  color='#fbbf24', lw=0.8,
                    alpha=0.7, label='plane_offset_mm')
            ax.axhline(0, color='#475569', lw=0.5, ls=':')
            ax.set_ylabel('height [mm]')
            ax.set_title('Per-frame height stats inside platform mask',
                         fontsize=10)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right', fontsize=8)
            ts_row += 1

            # --- 4. pose_z vs measured_center vs expected_center ---
            ax = fig.add_subplot(gs[ts_row, :])
            ax.plot(t_rel, dp_diag[:, 13], color='#10b981', lw=1.0,
                    label='pose_z_mm (ArUco)')
            ax.plot(t_rel, dp_diag[:, 14], color='#3b82f6', lw=0.8,
                    alpha=0.8, label='exp_center_mm (projected plane)')
            ax.plot(t_rel, dp_diag[:, 15], color='#f87171', lw=1.0,
                    label='meas_center_mm (stereo)')
            ax.set_ylabel('depth [mm]')
            ax.set_title(
                'Pose-derived plane vs stereo at image center '
                '(should agree on a flat deck)', fontsize=10)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right', fontsize=8)
            ts_row += 1

            # --- 5. valid pixel counts (mask vs eroded vs valid_in_mask) ---
            ax = fig.add_subplot(gs[ts_row, :])
            ax.plot(t_rel, dp_diag[:, 5], color='#94a3b8', lw=0.8,
                    label='mask_pixels')
            ax.plot(t_rel, dp_diag[:, 6], color='#64748b', lw=0.8,
                    label='eroded_pixels')
            ax.plot(t_rel, dp_diag[:, 7], color='#22d3ee', lw=1.0,
                    label='valid_in_eroded')
            ax2 = ax.twinx()
            ax2.plot(t_rel, dp_diag[:, 8], color='#fbbf24', lw=1.0,
                     label='n_above')
            ax2.set_ylabel('n_above [px]', color='#fbbf24')
            ax.set_ylabel('mask area [px]')
            ax.set_title(
                'Mask occupancy and pixel survival through the gate',
                fontsize=10)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper left', fontsize=8)
            ax2.legend(loc='upper right', fontsize=8)
            ts_row += 1

            # --- 6. frame depth distribution + invalid % ---
            ax = fig.add_subplot(gs[ts_row, :])
            # Translucent band between p25 and p75
            ax.fill_between(t_rel, dp_diag[:, 17], dp_diag[:, 19],
                            color='#3b82f6', alpha=0.15,
                            label='depth p25..p75')
            ax.plot(t_rel, dp_diag[:, 18], color='#3b82f6', lw=1.0,
                    label='depth median')
            ax.plot(t_rel, dp_diag[:, 16], color='#22d3ee', lw=0.7,
                    alpha=0.7, label='depth min')
            # Cap depth max so the y-axis stays readable; OAK reports
            # 60000+ for far/no-match.
            ax.plot(t_rel, np.clip(dp_diag[:, 20], 0, 5000),
                    color='#a78bfa', lw=0.7, alpha=0.7,
                    label='depth max (clipped 5 m)')
            ax2 = ax.twinx()
            ax2.plot(t_rel, dp_diag[:, 21], color='#f87171', lw=1.0,
                     label='invalid %')
            ax2.set_ylim(0, 100)
            ax2.set_ylabel('invalid %', color='#f87171')
            ax.set_ylabel('depth [mm]')
            ax.set_title(
                'Stereo health: full-frame depth distribution + '
                'invalid-pixel %', fontsize=10)
            ax.grid(alpha=0.3)
            ax.legend(loc='upper left', fontsize=8)
            ax2.legend(loc='upper right', fontsize=8)
            ts_row += 1
        else:
            # Legacy bag — single panel showing what we have.
            ax = fig.add_subplot(gs[ts_row, :])
            ax.text(0.5, 0.5, '(legacy bag — rich diagnostic absent)',
                    ha='center', va='center', transform=ax.transAxes,
                    color='#888')
            ax.set_xticks([])
            ax.set_yticks([])
            ts_row += 1

        # --- Final row: pixel scatter + platform-frame scatter ---
        ax = fig.add_subplot(gs[ts_row, 0:3])
        if v0_pix.size:
            ax.scatter(v0_pix[:, 0], v0_pix[:, 1], s=4,
                       color='#fb923c', alpha=0.4, label='V0')
        if dp_pix.size:
            ax.scatter(dp_pix[:, 0], dp_pix[:, 1], s=4,
                       color='#22d3ee', alpha=0.4, label='depth')
        ax.set_xlim(0, 960)
        ax.set_ylim(540, 0)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title('Detected pixels (image space)', fontsize=10)
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)

        ax = fig.add_subplot(gs[ts_row, 3:6])
        if bxy_mono.size:
            ax.scatter(bxy_mono[:, 0], bxy_mono[:, 1], s=4,
                       color='#fb923c', alpha=0.4, label='V0')
        if bxy_depth.size:
            ax.scatter(bxy_depth[:, 0], bxy_depth[:, 1], s=4,
                       color='#22d3ee', alpha=0.4, label='depth')
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

        # --- Suptitle ---
        title = f"vision-debug — {os.path.basename(bag_dir)} ({duration_s:.1f}s)"
        sub = (f"V0={summary['v0']['detections_n']}  "
               f"depth={summary['depth_blob']['detections_n']}  "
               f"frame_diag={summary['depth_blob']['frame_diag_n']}")
        if rich:
            fh = summary['depth_blob'].get('fail_histogram', {})
            sub += "  |  " + ", ".join(f"{k}={v}" for k, v in fh.items())
        if md.get('n', 0):
            sub += f"  |  V0↔depth p95={md['p95']:.1f}mm"
        fig.suptitle(title + '\n' + sub, fontsize=10, y=0.995)
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
