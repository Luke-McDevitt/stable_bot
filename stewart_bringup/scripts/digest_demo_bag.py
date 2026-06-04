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

# Shared latency math (pure NumPy, no ROS) — single source of truth,
# also used by compare_demo_bags.py. Importable from the installed
# package or, as a fallback, from the source tree (this script runs
# from scripts/ which may precede a colcon install).
try:
    from stewart_bringup._latency import (
        actuation_latency, quat_to_roll_pitch_deg)
except ImportError:
    sys.path.insert(
        0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_bringup._latency import (
        actuation_latency, quat_to_roll_pitch_deg)


def _parse_control_cmd(raw: str) -> dict:
    """The /control_cmd topic carries two interleaved formats from
    the GUI:

      "<key>:<value>"            e.g. "record_images:on"
      "<key>:<MODE_NAME> <json>" e.g. "mode:BALL_TRACK_TRAJECTORY {..}"
      "{<json>}"                 e.g. "{\\"cmd\\": \\"activate\\"}"

    The digest needs to find the latest "mode:..." with its full
    JSON payload (the demo's start parameters) and turn that into
    a structured row. Failed parses are kept verbatim so nothing
    silently disappears.
    """
    if not isinstance(raw, str):
        return {'raw': raw}
    s = raw.strip()
    # JSON object form (uncommon but used for some control commands).
    if s.startswith('{'):
        try:
            return json.loads(s)
        except Exception:
            pass
    if ':' not in s:
        return {'raw': s}
    key, _, payload = s.partition(':')
    key = key.strip()
    payload = payload.strip()
    out: dict = {'cmd': key, 'raw': s}
    if key == 'mode':
        # "mode:NAME" or "mode:NAME {json}"
        parts = payload.split(maxsplit=1)
        out['mode'] = parts[0].upper() if parts else ''
        if len(parts) > 1:
            try:
                out['params'] = json.loads(parts[1])
            except Exception:
                out['params_raw'] = parts[1]
    else:
        out['payload'] = payload
    return out


def _last_demo_command(cmd_d: list) -> tuple:
    """Walk /control_cmd in reverse for the most recent
    'mode:BALL_TRACK_*' command. For BALL_TRACK_GOTO, prefer
    entries that carry x_mm/y_mm — the bare arming command (no
    target yet) is published when the operator clicks Start, before
    they click the SVG to set a goal, so naively picking the
    literal-most-recent often returns the empty arming and loses the
    click target. Fall back to the bare arming if no later
    target-bearing command appears.

    Returns (mode_name, params_dict) or (None, None).
    """
    fallback = None
    for c in reversed(cmd_d):
        m = (c.get('mode') or '').upper()
        if not m.startswith('BALL_TRACK_'):
            continue
        params = dict(c.get('params') or {})
        if m == 'BALL_TRACK_GOTO':
            # Latest GOTO with a target — best case, return immediately.
            if 'x_mm' in params and 'y_mm' in params:
                return m, params
            if fallback is None:
                fallback = (m, params)
            continue
        # TRAJECTORY / PATH / etc. — first reverse hit wins.
        return m, params
    return fallback if fallback else (None, None)


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
    bt_diag_t, bt_diag = [], []   # ball_track FSM diagnostic
    lat_t,   lat_ms   = [], []     # /oak/latency_ms (capture→Pi)
    health_t, health  = [], []     # /oak/health Float32MultiArray
    config_d          = []         # /oak/config String (JSON snapshots)
    imu_t, imu_quat   = [], []     # /platform/imu/data fused orientation

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
            cmd_d.append(_parse_control_cmd(msg.data))
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
        elif topic == '/oak/latency_ms':
            # Float32: OAK capture-to-Pi-receipt wall-clock lag in ms.
            try:
                lat_t.append(t_ns)
                lat_ms.append(float(msg.data))
            except Exception:
                pass
        elif topic == '/oak/health':
            # Float32MultiArray: [v0_arr, v0_pub, jpeg, depth,
            #   depth_pub, pose, jpeg_lat_ms, v0_lat_ms]
            d = list(msg.data)
            if len(d) >= 8:
                health_t.append(t_ns)
                health.append([float(x) for x in d[:8]])
        elif topic == '/oak/config':
            # String: JSON snapshot of the OAK-side tunables.
            try:
                config_d.append(json.loads(msg.data))
            except Exception:
                config_d.append({'raw': msg.data})
        elif topic == '/ball_track/diagnostic':
            d = list(msg.data)
            # Field layout (see stewart_control_node._ball_track_run):
            # [t_rel, phase_code, tilt_pitch, tilt_roll, ex, ey,
            #  err_mag, v_toward, vel_mag, ux, uy]
            if len(d) >= 11:
                bt_diag_t.append(t_ns)
                bt_diag.append([float(x) for x in d[:11]])
        elif topic == '/platform/imu/data' and Imu is not None:
            # sensor_msgs/Imu: fused orientation quaternion (x,y,z,w),
            # ~240 Hz — the high-rate actual platform tilt used for the
            # command->motion actuation-latency cross-correlation.
            try:
                o = msg.orientation
                imu_t.append(t_ns)
                imu_quat.append([float(o.x), float(o.y),
                                 float(o.z), float(o.w)])
            except Exception:
                pass

    if imu_quat:
        _imu_roll, _imu_pitch = quat_to_roll_pitch_deg(np.array(imu_quat))
        imu_rp = np.stack([_imu_roll, _imu_pitch], axis=1)
    else:
        imu_rp = np.zeros((0, 2))

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
        'bt_diag': (np.array(bt_diag_t, dtype=np.int64),
                    np.array(bt_diag) if bt_diag else np.zeros((0, 11))),
        'lat':   (np.array(lat_t, dtype=np.int64),
                  np.array(lat_ms, dtype=np.float64) if lat_ms
                  else np.zeros((0,), dtype=np.float64)),
        'health': (np.array(health_t, dtype=np.int64),
                   np.array(health) if health
                   else np.zeros((0, 8))),
        'imu':   (np.array(imu_t, dtype=np.int64), imu_rp),
        'oak_config': config_d,
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


def _next_gain_recommendation(err_mag: np.ndarray,
                              err_t_s: np.ndarray,
                              state_v: np.ndarray,
                              cmd_pitch: np.ndarray,
                              cmd_roll: np.ndarray,
                              gains: dict | None,
                              demo_mode: str | None) -> dict:
    """Per-demo-run gain refinement suggestion. Reads:
      - final 5 s tracking error magnitude (did it settle?)
      - peak ball velocity (vision-noise-filtered via 95th percentile)
      - fraction of trial spent at max_tilt saturation
      - whether commanded tilt oscillates above ~3 Hz (Kd-driven
        noise — would benefit from a Kd reduction)

    Returns a dict with 'suggestion' (string), 'suggested_gains'
    (dict, keys present only if changed), and a 'rationale' list.

    The suggestion is conservative: 30-50% changes per axis at most,
    so the operator can iterate fast without overshooting. Multiple
    issues → suggest the dominant fix first; the next demo run picks
    up the next one.
    """
    # Note: we used to gate on demo_mode == 'BALL_TRACK_GOTO', but
    # the auto-bag recorder has a known race where the goto command
    # lands on /control_cmd before rosbag2 finishes subscribing —
    # so demo_mode often comes back as None even though it WAS a
    # goto. Tracking error vs /ball_ref tells us all we need
    # regardless of which BALL_TRACK_* mode generated it; relax the
    # gate to "have gains + have err".
    if not gains or err_mag.size == 0:
        return {'suggestion': 'no recommendation '
                              '(no gains snapshot or no /ball_state '
                              'data in bag)',
                'suggested_gains': {}, 'rationale': []}
    kp = float(gains.get('kp', 0.015))
    kd = float(gains.get('kd', 0.030))
    ki = float(gains.get('ki', 0.001))
    max_tilt = float(gains.get('max_tilt_deg', 2.5))
    err_tol = float(gains.get('err_tol_mm', 15.0))
    # Settle window = last 5 s of trial (or last quarter, whichever
    # is bigger — short trials need a different definition).
    if err_t_s.size:
        t_end = float(err_t_s[-1])
        tail_t = max(t_end - 5.0, t_end * 0.75)
    else:
        tail_t = 0.0
    tail_mask = err_t_s >= tail_t
    tail_err = err_mag[tail_mask] if tail_mask.any() else err_mag
    settled = bool(tail_err.size and float(np.median(tail_err))
                   < err_tol * 1.5)
    median_tail_err = (float(np.median(tail_err))
                       if tail_err.size else float('inf'))
    p95_err = (float(np.percentile(err_mag, 95))
               if err_mag.size else 0.0)
    # Vision-noise-filtered peak speed (95th percentile of |v|).
    if state_v.size:
        speed = np.linalg.norm(state_v, axis=1)
        speed = speed[np.isfinite(speed)]
        peak_speed = (float(np.percentile(speed, 95))
                      if speed.size else 0.0)
    else:
        peak_speed = 0.0
    # Saturation fraction: how much of the trial was the cmd tilt
    # at max_tilt? Indicates the controller is bang-bang-like.
    sat_frac = 0.0
    if cmd_pitch.size and cmd_roll.size:
        cmd_mag = np.maximum(np.abs(cmd_pitch), np.abs(cmd_roll))
        # 95% of max_tilt counts as "saturated" (small float slack).
        sat_frac = float(np.mean(cmd_mag > 0.95 * max_tilt))
    # High-frequency oscillation in commanded tilt = Kd amplifying
    # noise. Detect via std of frame-to-frame cmd_tilt deltas.
    cmd_jitter = 0.0
    if cmd_pitch.size > 5:
        d_cmd = np.diff(cmd_pitch)
        cmd_jitter = float(np.std(d_cmd))
    rationale: list[str] = []
    sg: dict = {}
    suggestion = 'fine-tune from here'
    # Decision tree, ordered by severity.
    if median_tail_err > 100.0 and peak_speed < 80.0:
        # Ball barely moved.
        rationale.append(
            f'final-5s err={median_tail_err:.0f}mm '
            f'with peak speed {peak_speed:.0f}mm/s — ball not '
            f'breaking stiction or Kp far too low.')
        sg['kp'] = round(kp * 1.5, 4)
        sg['max_tilt_deg'] = max(max_tilt, round(sg['kp'] * 60, 1))
        suggestion = (f'Kp +50% (→{sg["kp"]:.3f}) '
                      f'and raise max_tilt to {sg["max_tilt_deg"]:.1f}° '
                      f'so the controller has enough authority to '
                      f'break stiction at this error magnitude.')
    elif (median_tail_err > 60.0 and sat_frac > 0.5
          and peak_speed > 300.0):
        # Saturated + still bouncing around → too aggressive.
        rationale.append(
            f'cmd tilt at saturation {sat_frac*100:.0f}% of trial; '
            f'peak speed {peak_speed:.0f}mm/s — likely orbital '
            f'limit cycle.')
        sg['kp'] = round(kp * 0.7, 4)
        suggestion = (f'Kp -30% (→{sg["kp"]:.3f}) — controller is '
                      f'saturated most of the trial; back off Kp '
                      f'to keep error ranges in the linear zone.')
    elif (cmd_jitter > 0.4 and median_tail_err > 30.0):
        # High-frequency cmd tilt jitter → vision-noise + Kd issue.
        rationale.append(
            f'cmd tilt jitter std={cmd_jitter:.2f}°/tick suggests '
            f'Kd is amplifying vision noise into the tilt command.')
        sg['kd'] = round(kd * 0.6, 4)
        suggestion = (f'Kd -40% (→{sg["kd"]:.3f}) — Kd is feeding '
                      f'vision noise back as tilt command. With less '
                      f'Kd the controller will be slower but smoother.')
    elif median_tail_err > 30.0 and not settled:
        # Generic "not settled, not saturated, no jitter" → modest
        # bump in Kp.
        rationale.append(
            f'final-5s err={median_tail_err:.0f}mm > '
            f'1.5×err_tol={err_tol*1.5:.0f}mm but no clear failure '
            f'mode; nudge Kp up.')
        sg['kp'] = round(kp * 1.3, 4)
        suggestion = (f'Kp +30% (→{sg["kp"]:.3f}) — ball got near '
                      f'target but didn\'t settle within tolerance.')
    elif settled and median_tail_err > err_tol:
        # Settled but with steady-state offset → bump Ki.
        rationale.append(
            f'settled at err={median_tail_err:.0f}mm > tol '
            f'{err_tol:.0f}mm; steady-state offset.')
        sg['ki'] = round(ki * 2.0, 4)
        suggestion = (f'Ki ×2 (→{sg["ki"]:.4f}) — settled with a '
                      f'steady-state offset; small Ki bump should '
                      f'integrate the bias out.')
    elif settled:
        rationale.append(
            f'settled cleanly at err={median_tail_err:.0f}mm < '
            f'{err_tol*1.5:.0f}mm; no obvious refinement needed.')
        suggestion = 'looks good — ship it or run a longer trial.'
    return {
        'suggestion': suggestion,
        'suggested_gains': sg,
        'rationale': rationale,
        'metrics': {
            'settled': settled,
            'median_tail_err_mm': median_tail_err,
            'p95_err_mm': p95_err,
            'peak_speed_mm_s': peak_speed,
            'saturation_fraction': sat_frac,
            'cmd_jitter_deg_per_tick': cmd_jitter,
        },
    }


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
    lat_t, lat_ms = data['lat']
    health_t, health = data['health']
    oak_config = data.get('oak_config', [])

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
        finite_mask = np.isfinite(err_mag)
        err_mag = err_mag[finite_mask]
        err_t_s = err_t_s[finite_mask]
        err_xy = err_xy[finite_mask]

    settling = _settling_time_s(err_t_s, err_mag) if err_mag.size else None

    label = _demo_label_from_dir(bag_dir)
    gains_at_record = _gains_at_record(status_d)

    # Iteration-actionable gain recommendation. Reads the active
    # gains, the trial outcome, and the controller's actual
    # behaviour (saturation / cmd-jitter / peak speed) and proposes
    # the single most-impactful next gain change. Saves the operator
    # from re-analysing each iteration.
    bt_diag_t_local, bt_diag_local = data['bt_diag']
    cmd_pitch_arr = (bt_diag_local[:, 2]
                     if bt_diag_local.size else np.zeros((0,)))
    cmd_roll_arr = (bt_diag_local[:, 3]
                    if bt_diag_local.size else np.zeros((0,)))

    # Actuation latency: command -> actual platform motion, measured by
    # cross-correlating commanded tilt (/ball_track/diagnostic) against
    # the platform IMU tilt (~240 Hz). This is the under-counted half of
    # the loop the vision-only map could never measure — see
    # control_path_latency.md §3. Runs on existing bags (no new
    # instrumentation): the IMU + cmd tilt are already recorded.
    imu_t_local, imu_rp_local = data['imu']
    actuation = actuation_latency(
        bt_diag_t_local, cmd_pitch_arr, cmd_roll_arr,
        imu_t_local, imu_rp_local, rpy_t, rpy_data)
    # End-to-end budget = detector latency (vision) + actuation. The
    # localizer/KF tick (~25 ms) + feeder ZOH sit between and aren't
    # separately instrumented yet (Tier-2 capture-stamp propagation).
    v0_lat_mean = (float(np.nanmean(health[:, 7]))
                   if health.size else None)
    act_ms = actuation.get('actuation_ms') if actuation else None
    latency_breakdown = {
        'vision_detector_ms': (round(v0_lat_mean, 1)
                               if v0_lat_mean is not None else None),
        'vision_source': '/oak/health.v0_lat_ms (windowed detector latency)',
        'actuation': actuation,
        'see_to_move_est_ms': (round(v0_lat_mean + act_ms, 1)
                               if (v0_lat_mean is not None
                                   and act_ms is not None) else None),
        'note': ('vision = windowed detector latency; actuation = '
                 'cmd-tilt->IMU-tilt cross-correlation (effective '
                 'command->motion delay incl. feeder ZOH + leg slew + '
                 'mechanical rise). Localizer/KF tick (~25 ms) is '
                 'between them, not yet separately instrumented '
                 '(see control_path_latency.md).'),
    }

    # Demo-side params (radius, n_waypoints, arrival_tol_mm, dwell_s,
    # max_wait_s, dir, mode='waypoint'/'continuous', period_s, ...)
    # extracted from the latest mode:BALL_TRACK_* /control_cmd in the
    # bag — that's the command that started this demo run.
    demo_mode, demo_params = _last_demo_command(data['cmd'][1])
    # /control_cmd race: when the GUI's auto-bag is on, the SVG-click
    # target command can land on /control_cmd before the rosbag2
    # recorder finishes subscribing — bag captures only the bare
    # arming (no x_mm/y_mm). For BALL_TRACK_GOTO specifically, fall
    # back to the median of /ball_ref over the run, which the
    # ref_generator publishes continuously and is therefore always
    # captured. This guarantees demo_params has x_mm/y_mm whenever
    # the controller actually had a target.
    if (demo_mode == 'BALL_TRACK_GOTO'
            and demo_params is not None
            and 'x_mm' not in demo_params
            and ref_xy.size):
        demo_params = dict(demo_params)
        demo_params['x_mm'] = float(np.median(ref_xy[:, 0]))
        demo_params['y_mm'] = float(np.median(ref_xy[:, 1]))
        demo_params['_target_source'] = 'ball_ref_median'

    next_step = _next_gain_recommendation(
        err_mag, err_t_s,
        state_v if state_v.size else np.zeros((0, 2)),
        cmd_pitch_arr, cmd_roll_arr,
        gains_at_record, demo_mode)

    summary = {
        'bag': os.path.abspath(bag_dir),
        'demo_label': label,
        'demo_mode': demo_mode,
        'demo_params': demo_params,
        'duration_s': duration_s,
        'topic_counts': dict(sorted(data['counts'].items())),
        'gains_at_record': gains_at_record,
        'next_step': next_step,
        'ball_state_n': int(state_t.size),
        'ball_ref_n':   int(ref_t.size),
        'mono_n': int(mono_t.size),
        'depth_n': int(depth_t.size),
        'error_mm': _stats(err_mag),
        'error_x_mm': _stats(err_xy[:, 0]) if err_xy.size else {'n': 0},
        'error_y_mm': _stats(err_xy[:, 1]) if err_xy.size else {'n': 0},
        'settling_time_s': settling,
        'platform_pose_z_mm': _stats(pose_z),
        'latency_breakdown': latency_breakdown,
        'oak_latency_ms': _stats(lat_ms),
        # /oak/health field summary so the digest captures whether
        # the camera was actually hitting the rates the config asked
        # for during this run. Index order matches oak_driver_node:
        # 0 v0_arr_hz, 1 v0_pub_hz, 2 jpeg_hz, 3 depth_hz,
        # 4 depth_pub_hz, 5 pose_hz, 6 jpeg_lat_ms, 7 v0_lat_ms.
        'oak_health': ({
            'n': int(health.shape[0]),
            'v0_arr_hz': _stats(health[:, 0]) if health.size else {'n': 0},
            'v0_pub_hz': _stats(health[:, 1]) if health.size else {'n': 0},
            'jpeg_hz':   _stats(health[:, 2]) if health.size else {'n': 0},
            'pose_hz':   _stats(health[:, 5]) if health.size else {'n': 0},
            'jpeg_lat_ms': _stats(health[:, 6]) if health.size else {'n': 0},
            'v0_lat_ms':   _stats(health[:, 7]) if health.size else {'n': 0},
        }),
        # Most recent /oak/config snapshot in the bag (deduplicated by
        # JSON key set since the snapshot only changes when an env or
        # toggle flips). Captures the OAK-side tunables that were live
        # for this run, so we can correlate config tweaks with outcomes.
        'oak_config': (oak_config[-1] if oak_config else None),
        'markers_visible': {
            'n': int(mark_t.size),
            'mean': float(np.mean(mark_n)) if mark_n.size else None,
            'min':  int(np.min(mark_n)) if mark_n.size else None,
            'max':  int(np.max(mark_n)) if mark_n.size else None,
        },
    }

    print(f"[demo-digest] label={label} duration={duration_s:.1f}s "
          f"state={summary['ball_state_n']} ref={summary['ball_ref_n']}")
    if demo_mode:
        print(f"  demo_mode: {demo_mode}")
    if demo_params:
        print(f"  demo_params: {demo_params}")
    em = summary['error_mm']
    ex = summary.get('error_x_mm', {})
    ey = summary.get('error_y_mm', {})
    if em.get('n'):
        print(f"  error_mm: rms={em['rms']:.1f} p50={em['p50']:.1f} "
              f"p95={em['p95']:.1f} max={em['max']:.1f}")
    if ex.get('n'):
        print(f"  error_x_mm: mean={ex['mean']:+.1f} rms={ex['rms']:.1f} "
              f"p95={ex['p95']:+.1f}")
    if ey.get('n'):
        print(f"  error_y_mm: mean={ey['mean']:+.1f} rms={ey['rms']:.1f} "
              f"p95={ey['p95']:+.1f}")
    if settling is not None:
        print(f"  settling_time_s: {settling:.2f}")
    # Vision health — what camera/detector rates were achieved during
    # this run, and what the see→Pi latency looked like. These are
    # the numbers the user (and Claude) want at a glance to decide
    # whether the demo's tracking error came from controller tuning
    # or from vision starvation.
    h = summary.get('oak_health') or {}
    if h.get('n'):
        v0a = h.get('v0_arr_hz', {})
        v0p = h.get('v0_pub_hz', {})
        v0l = h.get('v0_lat_ms', {})
        jpl = h.get('jpeg_lat_ms', {})
        if v0a.get('n'):
            print(f"  v0_arr_hz: p50={v0a.get('p50','—'):.1f} "
                  f"p95={v0a.get('p95','—'):.1f}")
        if v0p.get('n'):
            print(f"  v0_pub_hz: p50={v0p.get('p50','—'):.1f} "
                  f"p95={v0p.get('p95','—'):.1f}")
        if v0l.get('n'):
            print(f"  v0_lat_ms: p50={v0l.get('p50','—'):.0f} "
                  f"p95={v0l.get('p95','—'):.0f}")
        if jpl.get('n'):
            print(f"  jpeg_lat_ms: p50={jpl.get('p50','—'):.0f} "
                  f"p95={jpl.get('p95','—'):.0f}")
    # Actuation + end-to-end latency budget — the new headline number:
    # how long after the controller commands a tilt does the platform
    # actually move there, and the see->move total.
    lb = summary.get('latency_breakdown') or {}
    act = lb.get('actuation') or {}
    if act.get('actuation_ms') is not None:
        print(f"  actuation_ms: {act['actuation_ms']:.0f}  "
              f"(pitch={act.get('pitch_lag_ms')}ms/corr {act.get('pitch_corr')}, "
              f"roll={act.get('roll_lag_ms')}ms/corr {act.get('roll_corr')}, "
              f"src={act.get('source')})")
        if lb.get('see_to_move_est_ms') is not None:
            print(f"  see->move budget: vision "
                  f"{lb.get('vision_detector_ms'):.0f}ms + actuation "
                  f"{act['actuation_ms']:.0f}ms ≈ "
                  f"{lb['see_to_move_est_ms']:.0f}ms")
    cfg = summary.get('oak_config')
    if cfg:
        print(f"  oak_config: focus={cfg.get('focus_pos')} "
              f"exp={cfg.get('exp_us')}us iso={cfg.get('iso')} "
              f"jpeg_q={cfg.get('jpeg_quality')} "
              f"depth={cfg.get('enable_depth')}")
    if gains_at_record:
        print(f"  gains: {gains_at_record}")

    # ----- Iteration-actionable next-step recommendation -----
    # Big banner so the operator can read it at a glance after each
    # demo run. The 'next_step' block also lands in
    # digest.summary.json for tooling that wants the structured form.
    ns = summary.get('next_step') or {}
    if ns.get('suggestion'):
        print()
        print(f"  ┌─ NEXT STEP ──────────────────────────────────")
        print(f"  │ {ns['suggestion']}")
        sg = ns.get('suggested_gains') or {}
        if sg:
            parts = ', '.join(f"{k}={v}" for k, v in sg.items())
            print(f"  │ Try: {parts}")
        m = ns.get('metrics') or {}
        if m:
            settled_str = '✓' if m.get('settled') else '✗'
            print(f"  │ settled={settled_str}  "
                  f"tail_err={m.get('median_tail_err_mm', 0):.0f}mm  "
                  f"peak_v={m.get('peak_speed_mm_s', 0):.0f}mm/s  "
                  f"sat={m.get('saturation_fraction', 0)*100:.0f}%  "
                  f"jitter={m.get('cmd_jitter_deg_per_tick', 0):.2f}°")
        for r in ns.get('rationale') or []:
            print(f"  │ ({r})")
        print(f"  └──────────────────────────────────────────────")

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

    bt_diag_t, bt_diag = data['bt_diag']
    if plt is not None:
        # Add a row for the bang-bang phase strip when we have data.
        nrows = 7 if bt_diag.size else 6
        height_ratios = [2.4, 1.2, 1.2, 1.2, 1.2, 1.2]
        if bt_diag.size:
            height_ratios.append(1.0)
        fig = plt.figure(figsize=(13, 14 if not bt_diag.size else 16))
        gs = GridSpec(nrows, 2, figure=fig,
                      height_ratios=height_ratios,
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

        # Row 6 (only if bang-bang diagnostic was bagged): phase
        # strip + commanded tilts + ball-velocity-toward-target.
        # Shows whether the FSM was stuck in one phase, chattering,
        # or transitioning cleanly.
        if bt_diag.size:
            ax = fig.add_subplot(gs[6, :])
            t_rel = (bt_diag_t - t0) * 1e-9
            phase = bt_diag[:, 1]
            # Color-code each phase. -1 stale, 0 settle, 1 accel,
            # 2 coast, 3 brake, 4 stiction_break, 5 pid.
            phase_colors = {
                -1: '#94a3b8',  # slate
                0: '#22c55e',   # green
                1: '#3b82f6',   # blue
                2: '#fbbf24',   # yellow
                3: '#ef4444',   # red
                4: '#a855f7',   # purple
                5: '#f97316',   # orange
            }
            phase_names = {
                -1: 'stale', 0: 'settle', 1: 'accel',
                2: 'coast', 3: 'brake', 4: 'stiction',
                5: 'pid',
            }
            # Scatter the phase strip near the bottom; commanded
            # tilts as line plots above.
            for code, color in phase_colors.items():
                m = phase == code
                if not m.any():
                    continue
                ax.scatter(
                    t_rel[m], np.full(m.sum(), -8.0),
                    s=6, c=color, marker='s',
                    label=f"{phase_names[code]} ({int(m.sum())})")
            ax.plot(t_rel, bt_diag[:, 2], color='#0ea5e9', lw=0.6,
                    label='cmd pitch [deg]')
            ax.plot(t_rel, bt_diag[:, 3], color='#7c3aed', lw=0.6,
                    label='cmd roll [deg]')
            # Overlay measured platform pitch so the command->motion lag
            # is visible against the command that drove it.
            imu_t_p, imu_rp_p = data['imu']
            if imu_rp_p.size and float(np.nanstd(imu_rp_p)) > 1e-4:
                ax.plot((imu_t_p - t0) * 1e-9, imu_rp_p[:, 1],
                        color='#64748b', lw=0.5, alpha=0.7,
                        label='IMU pitch (actual)')
            elif rpy_data.size:
                ax.plot((rpy_t - t0) * 1e-9, rpy_data[:, 1],
                        color='#64748b', lw=0.5, alpha=0.7,
                        label='rpy pitch (actual)')
            ax.set_ylabel('phase / cmd tilt')
            ax.set_xlabel('time [s]')
            ax.set_ylim(-10, 8)
            ax.grid(alpha=0.3)
            ax.legend(fontsize=7, loc='upper right', ncol=4)
            _act = (summary.get('latency_breakdown', {})
                    .get('actuation') or {})
            _act_str = (f" — actuation {_act['actuation_ms']:.0f} ms"
                        if _act.get('actuation_ms') is not None else "")
            ax.set_title(
                'BALL_TRACK FSM phase + commanded tilts' + _act_str,
                fontsize=10)

        # Suptitle — three lines: name+mode, error/settling, params.
        title = (f"Demo digest — {os.path.basename(bag_dir)} "
                 f"({label}{', ' + demo_mode if demo_mode else ''})")
        sub_bits = [f"duration={duration_s:.1f}s"]
        if em.get('n'):
            sub_bits.append(f"|err| rms={em['rms']:.1f} "
                            f"p95={em['p95']:.1f} max={em['max']:.1f} mm")
        if settling is not None:
            sub_bits.append(f"settled={settling:.2f}s")

        # Params line — gains + demo-side params on one row, so two
        # bags side-by-side make their differences obvious.
        param_bits = []
        if gains_at_record:
            g = gains_at_record
            param_bits.append(
                f"kp={g.get('kp')} kd={g.get('kd')} ki={g.get('ki')} "
                f"max_tilt={g.get('max_tilt_deg')}° "
                f"signs(p/r)={int(g.get('pitch_sign', 1))}/"
                f"{int(g.get('roll_sign', 1))}")
        if demo_params:
            p = demo_params
            short = {
                'mode':           'orbit',
                'radius_mm':      'R',
                'n_waypoints':    'N',
                'arrival_tol_mm': 'tol',
                'dwell_s':        'dwell',
                'max_wait_s':     'maxwait',
                'period_s':       'T',
                'dir':            'dir',
                'phase_rad':      'phase',
            }
            d_bits = []
            for k in ('mode', 'radius_mm', 'n_waypoints',
                      'arrival_tol_mm', 'dwell_s', 'max_wait_s',
                      'period_s', 'dir'):
                if k in p:
                    d_bits.append(f"{short[k]}={p[k]}")
            if d_bits:
                param_bits.append(' '.join(d_bits))

        full_sub = '  |  '.join(sub_bits)
        if param_bits:
            full_sub += '\n' + '  |  '.join(param_bits)
        fig.suptitle(title + '\n' + full_sub, fontsize=9)
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
