#!/usr/bin/env python3
"""digest_iva_bag.py — analyze an IMU-vs-Camera angle bag.

Reads a rosbag2 mcap directory written by gui_server's /iva/start
endpoint (manual record OR auto-record from the GUI sweep), then
produces:

  <bag>/digest.png         — time-series + residual plots
  <bag>/digest.summary.json — aggregate statistics

Topics consumed:

  /platform_rpy        Float32MultiArray[3]   roll, pitch, yaw (deg, IMU)
  /platform_pose       PoseStamped            ArUco-derived pose (RGB cam frame)
  /control_cmd         String (JSON)          set_pose commands (commanded tilt)
  /platform_pose/markers_visible  Int32       ArUco quality indicator

The vision RPY is decoded from /platform_pose's quaternion as
ZYX Euler. A "level reference" is auto-captured as the median of
the first 1.0 s of /platform_pose samples, and subsequent angles
are reported relative to that — exactly matching what the GUI
panel does when the operator clicks "Capture level reference".

Usage:
  python3 digest_iva_bag.py <bag_directory>
  python3 digest_iva_bag.py ~/stable_bot_repo/tuning_data/<UTC>Z_imu_vs_camera

Run on the Pi (or any machine with ROS 2 Kilted sourced — needs
rosbag2_py + the message types). matplotlib is required for the
PNG output.

Spec: companion to the IMU-vs-Camera Angle GUI panel
(stewart_bringup/web/index.html). Use to validate Stage C
calibration accuracy across the platform's tilt envelope.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import List, Optional, Tuple

import numpy as np

# Lazy import — these only resolve once /opt/ros/<distro>/setup.bash
# is sourced. If the user runs this without ROS, the error is
# unambiguous.
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
except ImportError as e:
    print(f"ERROR: rosbag2_py / rclpy unavailable: {e}", file=sys.stderr)
    print("       Source ROS first: source /opt/ros/kilted/setup.bash",
          file=sys.stderr)
    sys.exit(2)

try:
    from std_msgs.msg import Float32MultiArray, String, Int32
    from geometry_msgs.msg import PoseStamped
except ImportError as e:
    print(f"ERROR: ROS message types missing: {e}", file=sys.stderr)
    sys.exit(2)


# Relative to bag start (in seconds)
def _t_rel(t_ns: int, t0_ns: int) -> float:
    return (t_ns - t0_ns) * 1e-9


def _quat_to_zyx_deg(qx: float, qy: float, qz: float, qw: float):
    """Quaternion (xyzw) → ZYX Euler in degrees."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    rad = 180.0 / math.pi
    return roll * rad, pitch * rad, yaw * rad


def _read_bag(bag_dir: str):
    """Walk the bag once, deserialize the topics we care about.
    Returns dicts of stamps + decoded values per topic.
    """
    storage = StorageOptions(uri=bag_dir, storage_id='mcap')
    converter = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage, converter)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    type_class = {
        'std_msgs/msg/Float32MultiArray': Float32MultiArray,
        'std_msgs/msg/String':            String,
        'std_msgs/msg/Int32':             Int32,
        'geometry_msgs/msg/PoseStamped':  PoseStamped,
    }

    imu_t,    imu_rpy    = [], []   # (t, [roll, pitch, yaw])
    pose_t,   pose_quat  = [], []   # (t, [x,y,z,w])
    cmd_t,    cmd_pose   = [], []   # (t, dict with roll/pitch/...)
    mark_t,   mark_n     = [], []   # (t, int)

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        ttype = topic_types.get(topic)
        cls = type_class.get(ttype)
        if cls is None:
            continue
        try:
            msg = deserialize_message(raw, cls)
        except Exception:
            continue

        if topic == '/platform_rpy':
            d = list(msg.data)
            if len(d) >= 3:
                imu_t.append(t_ns)
                imu_rpy.append([float(d[0]), float(d[1]), float(d[2])])
        elif topic == '/platform_pose':
            q = msg.pose.orientation
            pose_t.append(t_ns)
            pose_quat.append([q.x, q.y, q.z, q.w])
        elif topic == '/control_cmd':
            try:
                obj = json.loads(msg.data)
            except Exception:
                continue
            if obj.get('cmd') == 'set_pose':
                cmd_t.append(t_ns)
                cmd_pose.append({
                    'roll':  float(obj.get('roll',  0.0)),
                    'pitch': float(obj.get('pitch', 0.0)),
                    'yaw':   float(obj.get('yaw',   0.0)),
                })
        elif topic == '/platform_pose/markers_visible':
            mark_t.append(t_ns)
            mark_n.append(int(msg.data))

    if not imu_t or not pose_t:
        raise RuntimeError(
            "bag has no /platform_rpy or /platform_pose messages — "
            "either it's the wrong bag or the recording missed those topics.")

    return {
        'imu':  (np.array(imu_t,  dtype=np.int64), np.array(imu_rpy)),
        'pose': (np.array(pose_t, dtype=np.int64), np.array(pose_quat)),
        'cmd':  (np.array(cmd_t,  dtype=np.int64), cmd_pose),
        'mark': (np.array(mark_t, dtype=np.int64), np.array(mark_n)),
    }


def _decode_pose_rpy(pose_quat: np.ndarray):
    """ZYX Euler in degrees from an Nx4 quaternion array."""
    out = np.zeros((pose_quat.shape[0], 3), dtype=np.float64)
    for i, q in enumerate(pose_quat):
        out[i] = _quat_to_zyx_deg(q[0], q[1], q[2], q[3])
    return out


def _level_reference(pose_t: np.ndarray, pose_rpy: np.ndarray,
                     window_s: float = 1.0):
    """Compute the level reference RPY as the median of the first
    `window_s` of /platform_pose. Returns (ref_roll, ref_pitch,
    ref_yaw) in degrees. If the bag is shorter than window_s, uses
    the whole thing.
    """
    if pose_t.size == 0:
        return 0.0, 0.0, 0.0
    t0 = pose_t[0]
    cutoff = t0 + int(window_s * 1e9)
    mask = pose_t <= cutoff
    if not mask.any():
        mask = np.ones_like(pose_t, dtype=bool)
    sub = pose_rpy[mask]
    return float(np.median(sub[:, 0])), \
           float(np.median(sub[:, 1])), \
           float(np.median(sub[:, 2]))


def _interp_imu_at_pose(pose_t: np.ndarray, imu_t: np.ndarray,
                        imu_rpy: np.ndarray):
    """Linearly interpolate IMU RPY onto the pose timestamps so the
    two streams can be subtracted point-for-point. Returns an
    (N_pose, 3) array of IMU RPY at the pose times. NaN where pose
    timestamps fall outside the IMU coverage.
    """
    out = np.full((pose_t.size, 3), np.nan, dtype=np.float64)
    if imu_t.size == 0:
        return out
    t = imu_t.astype(np.float64)
    for i in range(3):
        out[:, i] = np.interp(
            pose_t.astype(np.float64), t, imu_rpy[:, i],
            left=np.nan, right=np.nan)
    return out


def digest(bag_dir: str):
    print(f"[iva-digest] reading {bag_dir}")
    data = _read_bag(bag_dir)

    imu_t, imu_rpy = data['imu']
    pose_t, pose_quat = data['pose']
    mark_t, mark_n = data['mark']

    # Decode camera RPY relative to the level reference.
    pose_rpy_abs = _decode_pose_rpy(pose_quat)
    ref_r, ref_p, ref_y = _level_reference(pose_t, pose_rpy_abs)
    pose_rpy = pose_rpy_abs - np.array([ref_r, ref_p, ref_y])

    # Bag-relative time axes (seconds).
    t0 = min(int(imu_t[0]), int(pose_t[0]))
    imu_t_s  = (imu_t  - t0) * 1e-9
    pose_t_s = (pose_t - t0) * 1e-9
    mark_t_s = (mark_t - t0) * 1e-9 if mark_t.size else np.array([])

    # Interpolated IMU at pose timestamps for residuals.
    imu_at_pose = _interp_imu_at_pose(pose_t, imu_t, imu_rpy)
    valid = np.isfinite(imu_at_pose).all(axis=1)
    residual = pose_rpy - imu_at_pose
    residual_v = residual[valid]

    # Aggregate stats per axis.
    def _stats(arr):
        if arr.size == 0:
            return {'n': 0}
        return {
            'n': int(arr.size),
            'mean':  float(np.mean(arr)),
            'std':   float(np.std(arr)),
            'p50':   float(np.percentile(arr, 50)),
            'p95':   float(np.percentile(np.abs(arr), 95)),
            'max_abs': float(np.max(np.abs(arr))),
        }

    summary = {
        'bag': os.path.abspath(bag_dir),
        'duration_s': float(pose_t_s[-1] - pose_t_s[0]) \
            if pose_t_s.size > 1 else 0.0,
        'imu_n':  int(imu_t.size),
        'pose_n': int(pose_t.size),
        'level_reference_deg': {
            'roll':  ref_r, 'pitch': ref_p, 'yaw': ref_y,
        },
        'residual_deg': {
            'roll':  _stats(residual_v[:, 0]),
            'pitch': _stats(residual_v[:, 1]),
            'yaw':   _stats(residual_v[:, 2]),
        },
        'markers_visible': {
            'mean':    float(np.mean(mark_n)) if mark_n.size else None,
            'min':     int(np.min(mark_n))    if mark_n.size else None,
            'max':     int(np.max(mark_n))    if mark_n.size else None,
        },
    }

    print(f"[iva-digest] duration={summary['duration_s']:.1f} s, "
          f"IMU samples={summary['imu_n']}, pose samples={summary['pose_n']}")
    for axis in ('roll', 'pitch', 'yaw'):
        s = summary['residual_deg'][axis]
        if s['n']:
            print(f"  Δ {axis:>5}: mean={s['mean']:+.3f}° "
                  f"std={s['std']:.3f}° p95={s['p95']:.3f}° "
                  f"max|Δ|={s['max_abs']:.3f}°")

    # Plot.
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("[iva-digest] matplotlib not installed; "
              "skipping PNG. `pip install matplotlib --break-system-packages`")
    else:
        fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

        labels = ('roll', 'pitch', 'yaw')
        colors_imu = ('#10b981', '#10b981', '#10b981')   # emerald
        colors_cam = ('#3b82f6', '#3b82f6', '#3b82f6')   # blue
        for i, label in enumerate(labels):
            ax = axes[i]
            ax.plot(imu_t_s, imu_rpy[:, i],
                    color=colors_imu[i], lw=1.0, label='IMU')
            ax.plot(pose_t_s, pose_rpy[:, i],
                    color=colors_cam[i], lw=1.0, alpha=0.8, label='camera')
            ax.set_ylabel(f'{label} [°]')
            ax.grid(alpha=0.3)
            ax.legend(loc='upper right', fontsize=8)
            s = summary['residual_deg'][label]
            if s['n']:
                ax.text(0.01, 0.97,
                        f"Δ mean={s['mean']:+.2f}° "
                        f"std={s['std']:.2f}° "
                        f"p95={s['p95']:.2f}° "
                        f"max={s['max_abs']:.2f}°",
                        transform=ax.transAxes, va='top',
                        fontsize=8, family='monospace',
                        color=('#16a34a' if s['p95'] < 0.5
                               else '#ca8a04' if s['p95'] < 1.5
                               else '#dc2626'))

        # Marker count on the bottom panel.
        if mark_t_s.size:
            axes[3].step(mark_t_s, mark_n, where='post',
                         color='#a78bfa', lw=1.0)
            axes[3].set_ylabel('markers visible')
            axes[3].set_ylim(-0.5, 8.5)
            axes[3].grid(alpha=0.3)
        axes[3].set_xlabel('time since bag start [s]')

        fig.suptitle(
            f'IMU vs Camera angle — {os.path.basename(bag_dir)}',
            fontsize=11)
        fig.tight_layout()
        png_path = os.path.join(bag_dir, 'digest.png')
        fig.savefig(png_path, dpi=120, bbox_inches='tight')
        plt.close(fig)
        print(f"[iva-digest] wrote {png_path}")

    json_path = os.path.join(bag_dir, 'digest.summary.json')
    with open(json_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"[iva-digest] wrote {json_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag', help='Path to the bag directory '
                              '(contains metadata.yaml + .mcap)')
    args = p.parse_args()
    if not os.path.isdir(args.bag):
        print(f"ERROR: not a directory: {args.bag}", file=sys.stderr)
        return 2
    digest(args.bag)
    return 0


if __name__ == '__main__':
    sys.exit(main())
