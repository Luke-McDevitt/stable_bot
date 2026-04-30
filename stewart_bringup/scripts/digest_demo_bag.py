#!/usr/bin/env python3
"""digest_demo_bag.py — analyze a Demo-run bag.

Reads a rosbag2 mcap directory written by gui_server's /demo/start
endpoint and produces:

  <bag>/digest.png          — multi-panel tracking + control plots
  <bag>/digest.summary.json — error / settling / gain stats

Push-to-git ships only the digest artifacts (raw mcap stays on
the Pi — gitignored).

Topics consumed:

  /ball_state                  geometry_msgs/PoseStamped   (KF posterior)
  /ball_ref                    geometry_msgs/PointStamped  (target)
  /ball_xy_mono, /ball_xy_depth PointStamped               (raw localizer)
  /platform_pose               PoseStamped                 (ArUco)
  /platform_pose/markers_visible Int32
  /platform_rpy                Float32MultiArray[3]        (IMU RPY)
  /control_cmd, /control_result String                     (operator IO)
  /status                      String (JSON)
  /leg_encoders                Float64MultiArray[6]
  /leg_currents                Float32MultiArray[6]

The headline metric is **tracking error**: the Euclidean distance
between the KF ball state and the active reference, sampled at the
ball-state rate. From it we derive:

  - rms / p50 / p95 / max  →  guides Kp tuning
  - settling_time_s         →  time-to-±10mm at the end of a goto
  - per-axis bias           →  catches sign convention errors
  - reference-vs-state lag  →  guides Kd tuning

The digest also embeds the gain set in effect during the run
(parsed from /status messages) so a future digest can be compared
to a past one and you can correlate gain changes to outcomes.
"""
from __future__ import annotations

import argparse
import json
import math
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
    from std_msgs.msg import (Float32MultiArray, Float64MultiArray,
                              String, Int32, Float32)
    from geometry_msgs.msg import PoseStamped, PointStamped
    try:
        from sensor_msgs.msg import Imu
    except ImportError:
        Imu = None
except ImportError as e:
    print(f"ERROR: ROS message types missing: {e}", file=sys.stderr)
    sys.exit(2)


def _open_bag(bag_dir: str):
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
            f"could not open bag at {bag_dir}: {last_err}")


_TYPE_CLASS = {
    'std_msgs/msg/Float32MultiArray':  Float32MultiArray,
    'std_msgs/msg/Float64MultiArray':  Float64MultiArray,
    'std_msgs/msg/Float32':            Float32,
    'std_msgs/msg/String':             String,
    'std_msgs/msg/Int32':              Int32,
    'geometry_msgs/msg/PoseStamped':   PoseStamped,
    'geometry_msgs/msg/PointStamped':  PointStamped,
}
if Imu is not None:
    _TYPE_CLASS['sensor_msgs/msg/Imu'] = Imu


def _read_bag(bag_dir: str):
    reader = _open_bag(bag_dir)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts = {n: 0 for n in topic_types}

    state_t, state_xy, state_v = [], [], []  # KF: pos + vel
    ref_t,   ref_xy   = [], []
    mono_t,  mono_xy  = [], []
    depth_t, depth_xy = [], []
    pose_t,  pose_z   = [], []
    rpy_t,   rpy_data = [], []
    cmd_t,   cmd_d    = [], []     # control_cmd: list of (t, dict|str)
    res_t,   res_d    = [], []
    status_t, status_d = [], []    # parsed json
    mark_t,  mark_n   = [], []
    enc_t,   enc_data = [], []
    cur_t,   cur_data = [], []

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

        if topic == '/ball_state':
            # PoseStamped: position is xy in mm, orientation as wxyz
            # encodes velocity in (x, y, _, _) per ball_kf_node convention.
            state_t.append(t_ns)
            state_xy.append([float(msg.pose.position.x),
                             float(msg.pose.position.y)])
            # Convention: vx, vy bagged in orientation.x, orientation.y
            state_v.append([float(msg.pose.orientation.x),
                            float(msg.pose.orientation.y)])
        elif topic == '/ball_ref':
            ref_t.append(t_ns)
            ref_xy.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/ball_xy_mono':
            mono_t.append(t_ns)
            mono_xy.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/ball_xy_depth':
            depth_t.append(t_ns)
            depth_xy.append([float(msg.point.x), float(msg.point.y)])
        elif topic == '/platform_pose':
            pose_t.append(t_ns)
            pose_z.append(float(msg.pose.position.z) * 1000.0)
        elif topic == '/platform_rpy':
            d = list(msg.data)
            if len(d) >= 3:
                rpy_t.append(t_ns)
                rpy_data.append([float(d[0]), float(d[1]), float(d[2])])
        elif topic == '/control_cmd':
            cmd_t.append(t_ns)
            try:
                cmd_d.append(json.loads(msg.data))
            except Exception:
                cmd_d.append({'raw': msg.data})
        elif topic == '/control_result':
            res_t.append(t_ns)
            try:
                res_d.append(json.loads(msg.data))
            except Exception:
                res_d.append({'raw': msg.data})
        elif topic == '/status':
            try:
                s = json.loads(msg.data)
                status_t.append(t_ns)
                status_d.append(s)
            except Exception:
                continue
        elif topic == '/platform_pose/markers_visible':
            mark_t.append(t_ns)
            mark_n.append(int(msg.data))
        elif topic == '/leg_encoders':
            d = list(msg.data)
            if len(d) >= 6:
                enc_t.append(t_ns)
                enc_data.append([float(x) for x in d[:6]])
        elif topic == '/leg_currents':
            d = list(msg.data)
            if len(d) >= 6:
                cur_t.append(t_ns)
                cur_data.append([float(x) for x in d[:6]])

    return {
        'topic_types': topic_types,
        'counts': counts,
        'state': (np.array(state_t, dtype=np.int64),
                  np.array(state_xy) if state_xy else np.zeros((0, 2)),
                  np.array(state_v)  if state_v  else np.zeros((0, 2))),
        'ref':   (np.array(ref_t, dtype=np.int64),
                  np.array(ref_xy) if ref_xy else np.zeros((0, 2))),
        'mono':  (np.array(mono_t, dtype=np.int64),
                  np.array(mono_xy) if mono_xy else np.zeros((0, 2))),
        'depth': (np.array(depth_t, dtype=np.int64),
                  np.array(depth_xy) if depth_xy else np.zeros((0, 2))),
        'pose':  (np.array(pose_t, dtype=np.int64),
                  np.array(pose_z, dtype=np.float64)),
        'rpy':   (np.array(rpy_t, dtype=np.int64),
                  np.array(rpy_data) if rpy_data else np.zeros((0, 3))),
        'cmd':   (np.array(cmd_t, dtype=np.int64), cmd_d),
        'res':   (np.array(res_t, dtype=np.int64), res_d),
        'status': (np.array(status_t, dtype=np.int64), status_d),
        'mark':  (np.array(mark_t, dtype=np.int64),
                  np.array(mark_n, dtype=np.int32) if mark_n
                  else np.zeros((0,), dtype=np.int32)),
        'enc':   (np.array(enc_t, dtype=np.int64),
                  np.array(enc_data) if enc_data else np.zeros((0, 6))),
        'cur':   (np.array(cur_t, dtype=np.int64),
                  np.array(cur_data) if cur_data else np.zeros((0, 6))),
    }


def _stats(arr) -> dict:
    a = np.asarray(arr, dtype=np.float64)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return {'n': 0}
    return {
        'n':      int(a.size),
        'mean':   float(np.mean(a)),
        'std':    float(np.std(a)),
        'rms':    float(np.sqrt(np.mean(a * a))),
        'p50':    float(np.percentile(a, 50)),
        'p95':    float(np.percentile(a, 95)),
        'min':    float(np.min(a)),
        'max':    float(np.max(a)),
    }


def _interp_xy(t_query_s, t_src_s, xy_src):
    if xy_src.shape[0] == 0:
        return np.full((t_query_s.size, 2), np.nan, dtype=np.float64)
    out = np.zeros((t_query_s.size, 2), dtype=np.float64)
    out[:, 0] = np.interp(t_query_s, t_src_s, xy_src[:, 0],
                          left=np.nan, right=np.nan)
    out[:, 1] = np.interp(t_query_s, t_src_s, xy_src[:, 1],
                          left=np.nan, right=np.nan)
    return out


def _settling_time_s(err_t_s, err_mag, settle_band_mm=10.0,
                     dwell_s=1.0):
    """For each frame, find the latest time after which |err| stays
    below settle_band for at least dwell_s consecutively. Returns
    the time-from-start to that point, or None if never settled."""
    if err_t_s.size == 0:
        return None
    inside = err_mag < settle_band_mm
    # Walk backwards: find the longest tail where inside stays True.
    tail_start = None
    for i in range(err_t_s.size - 1, -1, -1):
        if not inside[i]:
            tail_start = i + 1
            break
    if tail_start is None:
        tail_start = 0
    if tail_start >= err_t_s.size:
        return None  # never inside the band
    # Need at least dwell_s of dwell.
    if err_t_s[-1] - err_t_s[tail_start] < dwell_s:
        return None
    return float(err_t_s[tail_start] - err_t_s[0])


def _gains_at_record(status_d):
    """Pull the ball_track_gains from the LAST /status message that
    carries them. Returns dict or None."""
    for s in reversed(status_d):
        g = s.get('ball_track_gains')
        if isinstance(g, dict):
            return g
    return None


def _demo_label_from_dir(bag_dir):
    name = os.path.basename(bag_dir.rstrip('/'))
    for tag in ('demo1', 'demo2', 'demo3', 'demo_untagged', 'demo_run'):
        if tag in name:
            return tag
    return 'unknown'


def digest(bag_dir: str):
    print(f"[demo-digest] reading {bag_dir}")
    data = _read_bag(bag_dir)

    state_t, state_xy, state_v = data['state']
    ref_t, ref_xy = data['ref']
    mono_t, mono_xy = data['mono']
    depth_t, depth_xy = data['depth']
    pose_t, pose_z = data['pose']
    rpy_t, rpy_data = data['rpy']
    status_t, status_d = data['status']
    mark_t, mark_n = data['mark']
    enc_t, enc_data = data['enc']
    cur_t, cur_data = data['cur']

    # Bag-relative origin.
    cands = []
    for arr in (state_t, ref_t, mono_t, depth_t, pose_t, rpy_t, status_t):
        if len(arr):
            cands.append(int(arr[0]))
    if not cands:
        raise RuntimeError("bag has no usable topics")
    t0 = min(cands)
    t_end_cands = []
    for arr in (state_t, ref_t, mono_t, depth_t, pose_t, rpy_t, status_t):
        if len(arr):
            t_end_cands.append(int(arr[-1]))
    t1 = max(t_end_cands) if t_end_cands else t0
    duration_s = (t1 - t0) * 1e-9

    # Tracking error: state vs ref interpolated to state timestamps.
    err_t_s = (state_t - t0) * 1e-9 if state_t.size else np.zeros((0,))
    err_mag = np.zeros((0,), dtype=np.float64)
    err_xy = np.zeros((0, 2), dtype=np.float64)
    if state_t.size and ref_t.size:
        ref_at_state = _interp_xy(err_t_s,
                                  (ref_t - t0) * 1e-9,
                                  ref_xy)
        err_xy = state_xy - ref_at_state
        err_mag = np.sqrt(err_xy[:, 0] ** 2 + err_xy[:, 1] ** 2)
        err_mag = err_mag[np.isfinite(err_mag)]

    settling = _settling_time_s(
        err_t_s[np.isfinite(err_xy[:, 0])] if err_xy.size else np.zeros((0,)),
        err_mag) if err_mag.size else None

    label = _demo_label_from_dir(bag_dir)
    gains_at_record = _gains_at_record(status_d)

    summary = {
        'bag': os.path.abspath(bag_dir),
        'demo_label': label,
        'duration_s': duration_s,
        'topic_counts': dict(sorted(data['counts'].items())),
        'gains_at_record': gains_at_record,
        'ball_state_n': int(state_t.size),
        'ball_ref_n':   int(ref_t.size),
        'mono_n': int(mono_t.size),
        'depth_n': int(depth_t.size),
        'error_mm': _stats(err_mag),
        'error_x_mm': _stats(err_xy[:, 0]) if err_xy.size else {'n': 0},
        'error_y_mm': _stats(err_xy[:, 1]) if err_xy.size else {'n': 0},
        'settling_time_s': settling,
        'platform_pose_z_mm': _stats(pose_z),
        'markers_visible': {
            'n': int(mark_t.size),
            'mean': float(np.mean(mark_n)) if mark_n.size else None,
            'min':  int(np.min(mark_n)) if mark_n.size else None,
            'max':  int(np.max(mark_n)) if mark_n.size else None,
        },
    }

    print(f"[demo-digest] label={label} duration={duration_s:.1f}s "
          f"state={summary['ball_state_n']} ref={summary['ball_ref_n']}")
    em = summary['error_mm']
    if em.get('n'):
        print(f"  error_mm: rms={em['rms']:.1f} p50={em['p50']:.1f} "
              f"p95={em['p95']:.1f} max={em['max']:.1f}")
    if settling is not None:
        print(f"  settling_time_s: {settling:.2f}")
    if gains_at_record:
        print(f"  gains: {gains_at_record}")

    # ----- Plot -----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
    except ImportError:
        print("[demo-digest] matplotlib not installed; skipping PNG.",
              file=sys.stderr)
        plt = None

    if plt is not None:
        fig = plt.figure(figsize=(13, 14))
        gs = GridSpec(6, 2, figure=fig,
                      height_ratios=[2.4, 1.2, 1.2, 1.2, 1.2, 1.2],
                      hspace=0.6, wspace=0.25)

        # Row 0 left: 2D ball trajectory + reference + dead-zone
        ax = fig.add_subplot(gs[0, 0])
        if state_xy.size:
            ax.plot(state_xy[:, 0], state_xy[:, 1],
                    color='#3b82f6', lw=1.0, alpha=0.7,
                    label='ball state (KF)')
        if ref_xy.size:
            ax.plot(ref_xy[:, 0], ref_xy[:, 1],
                    color='#10b981', lw=1.5,
                    label='reference')
        # Platform disk + dead-zone
        theta = np.linspace(0, 2 * np.pi, 200)
        ax.plot(220 * np.cos(theta), 220 * np.sin(theta),
                color='#475569', lw=0.5, ls='--', alpha=0.6,
                label='gate r=220')
        ax.plot(35 * np.cos(theta), 35 * np.sin(theta),
                color='#dc2626', lw=0.7, ls=':', alpha=0.6,
                label='dead-zone')
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlim(-260, 260)
        ax.set_ylim(-260, 260)
        ax.grid(alpha=0.3)
        ax.set_title('Trajectory (platform frame, mm)', fontsize=10)
        ax.legend(fontsize=7, loc='upper right')

        # Row 0 right: error magnitude over time + settling band
        ax = fig.add_subplot(gs[0, 1])
        if err_mag.size:
            ax.plot(err_t_s[:err_mag.size], err_mag,
                    color='#f87171', lw=1.0)
            ax.axhline(10.0, color='#10b981', ls='--', lw=0.8,
                       label='±10 mm settling band')
            if settling is not None:
                ax.axvline(settling, color='#10b981', ls=':', lw=0.8,
                           label=f'settled @ {settling:.1f}s')
        ax.set_ylabel('|err| [mm]')
        ax.set_xlabel('time [s]')
        ax.set_title('Tracking error magnitude', fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')

        # Row 1: x state vs ref over time
        ax = fig.add_subplot(gs[1, :])
        if state_xy.size:
            ax.plot((state_t - t0) * 1e-9, state_xy[:, 0],
                    color='#3b82f6', lw=1.0, label='state.x')
        if ref_xy.size:
            ax.plot((ref_t - t0) * 1e-9, ref_xy[:, 0],
                    color='#10b981', lw=1.0, label='ref.x')
        ax.set_ylabel('x [mm]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
        ax.set_title('x-axis tracking', fontsize=10)

        # Row 2: y state vs ref
        ax = fig.add_subplot(gs[2, :])
        if state_xy.size:
            ax.plot((state_t - t0) * 1e-9, state_xy[:, 1],
                    color='#3b82f6', lw=1.0, label='state.y')
        if ref_xy.size:
            ax.plot((ref_t - t0) * 1e-9, ref_xy[:, 1],
                    color='#10b981', lw=1.0, label='ref.y')
        ax.set_ylabel('y [mm]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
        ax.set_title('y-axis tracking', fontsize=10)

        # Row 3: KF velocities (catches Kd-tuning issues)
        ax = fig.add_subplot(gs[3, :])
        if state_v.size:
            ax.plot((state_t - t0) * 1e-9, state_v[:, 0],
                    color='#fb923c', lw=1.0, label='vx')
            ax.plot((state_t - t0) * 1e-9, state_v[:, 1],
                    color='#a78bfa', lw=1.0, label='vy')
        ax.set_ylabel('velocity [mm/s]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
        ax.set_title('Ball velocity (from KF)', fontsize=10)

        # Row 4: platform RPY (commanded vs measured visible)
        ax = fig.add_subplot(gs[4, :])
        if rpy_data.size:
            ax.plot((rpy_t - t0) * 1e-9, rpy_data[:, 0],
                    color='#f87171', lw=0.8, label='IMU roll')
            ax.plot((rpy_t - t0) * 1e-9, rpy_data[:, 1],
                    color='#fbbf24', lw=0.8, label='IMU pitch')
        ax.set_ylabel('platform tilt [deg]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8, loc='upper right')
        ax.set_title('IMU roll/pitch (what the platform actually did)',
                     fontsize=10)

        # Row 5: leg currents (effort proxy)
        ax = fig.add_subplot(gs[5, :])
        if cur_data.size:
            for i in range(6):
                ax.plot((cur_t - t0) * 1e-9, cur_data[:, i],
                        lw=0.7, label=f'L{i}')
        ax.set_ylabel('leg current [A]')
        ax.set_xlabel('time [s]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc='upper right', ncol=6)
        ax.set_title('Per-leg current (effort)', fontsize=10)

        # Suptitle
        title = (f"Demo digest — {os.path.basename(bag_dir)} "
                 f"({label})")
        sub_bits = [f"duration={duration_s:.1f}s"]
        if em.get('n'):
            sub_bits.append(f"|err| rms={em['rms']:.1f} "
                            f"p95={em['p95']:.1f} max={em['max']:.1f} mm")
        if settling is not None:
            sub_bits.append(f"settled={settling:.2f}s")
        if gains_at_record:
            sub_bits.append(
                f"kp={gains_at_record.get('kp')} "
                f"kd={gains_at_record.get('kd')} "
                f"ki={gains_at_record.get('ki')}")
        fig.suptitle(title + '\n' + '  |  '.join(sub_bits), fontsize=10)
        png = os.path.join(bag_dir, 'digest.png')
        fig.savefig(png, dpi=110, bbox_inches='tight')
        plt.close(fig)
        print(f"[demo-digest] wrote {png}")

    js = os.path.join(bag_dir, 'digest.summary.json')
    with open(js, 'w') as f:
        json.dump(summary, f, indent=2, default=str)
        f.write('\n')
    print(f"[demo-digest] wrote {js}")


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
