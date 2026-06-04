#!/usr/bin/env python3
"""digest_latency_bench.py — analyze a Latency Bench tilt-step bag.

The Latency Bench panel commands a clean, aperiodic tilt STEP (no ball,
no vision needed) and records it. This digest measures the *actuation*
half of the loop per-stage — the thing the orbital cross-correlation in
digest_demo_bag can't resolve unambiguously (a limit cycle aliases the
lag; a step does not).

Reads (all already in DEMO_TOPICS + the bench's own diag):
  /latency_bench/diag   std_msgs/Float32MultiArray [t_rel, cmd_pitch_deg,
                          cmd_roll_deg, phase]  — the commanded tilt timeline
  /platform/imu/data    sensor_msgs/Imu        — measured tilt (~240 Hz)
  /leg_encoders         std_msgs/Float64MultiArray[6] — leg positions (turns)
  /status               std_msgs/String (JSON) — caps in effect

Writes alongside the bag (same filenames as the demo digest so the GUI
push reuses unchanged):
  digest.png            — cmd vs IMU tilt timeline with the steps marked
  digest.summary.json   — per-stage step metrics (dead time, rise, settle,
                          overshoot, gain) for pitch + roll + the leg-slew
                          split, plus run_config caps.

The per-stage story:
  cmd → leg-encoder onset   = transport (feeder ZOH + CAN + ODrive dead time)
  leg-encoder onset → IMU   = leg slew → platform
  IMU rise (10→90 %)        = mechanical response
"""
from __future__ import annotations

import json
import os
import sys

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
    from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
    from sensor_msgs.msg import Imu
except ImportError as e:
    print(f"ERROR: ROS message types missing: {e}", file=sys.stderr)
    sys.exit(2)

# Shared, unit-tested latency math (same module the demo digest uses).
try:
    from stewart_bringup._latency import (
        quat_to_roll_pitch_deg, step_train_metrics)
except ImportError:
    sys.path.insert(
        0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from stewart_bringup._latency import (
        quat_to_roll_pitch_deg, step_train_metrics)


_TYPE_CLASS = {
    'std_msgs/msg/Float32MultiArray': Float32MultiArray,
    'std_msgs/msg/Float64MultiArray': Float64MultiArray,
    'std_msgs/msg/String': String,
    'sensor_msgs/msg/Imu': Imu,
}


def _open_bag(bag_dir):
    storage = StorageOptions(uri=bag_dir, storage_id='')
    reader = SequentialReader()
    try:
        reader.open(storage, ConverterOptions('', ''))
        return reader
    except Exception:
        mcaps = sorted(p for p in os.listdir(bag_dir) if p.endswith('.mcap'))
        for mf in mcaps:
            try:
                reader = SequentialReader()
                reader.open(StorageOptions(uri=os.path.join(bag_dir, mf),
                                           storage_id='mcap'),
                            ConverterOptions('', ''))
                return reader
            except Exception:
                continue
        raise RuntimeError(f"could not open bag at {bag_dir}")


def _read_bag(bag_dir):
    reader = _open_bag(bag_dir)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    diag_t, diag = [], []          # [t_rel, cmd_pitch, cmd_roll, phase]
    imu_t, imu_quat = [], []
    enc_t, enc = [], []
    status_d = []
    counts = {}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        cls = _TYPE_CLASS.get(topic_types.get(topic))
        if cls is None:
            continue
        try:
            msg = deserialize_message(raw, cls)
        except Exception:
            continue
        if topic == '/latency_bench/diag':
            d = list(msg.data)
            if len(d) >= 3:
                diag_t.append(t_ns)
                diag.append([float(d[0]), float(d[1]), float(d[2]),
                             float(d[3]) if len(d) > 3 else 0.0])
        elif topic == '/platform/imu/data':
            o = msg.orientation
            imu_t.append(t_ns)
            imu_quat.append([float(o.x), float(o.y), float(o.z), float(o.w)])
        elif topic == '/leg_encoders':
            d = list(msg.data)
            if len(d) >= 6:
                enc_t.append(t_ns)
                enc.append([float(x) for x in d[:6]])
        elif topic == '/status':
            try:
                status_d.append(json.loads(msg.data))
            except Exception:
                pass
    if imu_quat:
        roll, pitch = quat_to_roll_pitch_deg(np.array(imu_quat))
        imu_rp = np.stack([roll, pitch], axis=1)
    else:
        imu_rp = np.zeros((0, 2))
    return {
        'counts': counts,
        'diag_t': np.array(diag_t, dtype=np.int64),
        'diag': np.array(diag) if diag else np.zeros((0, 4)),
        'imu_t': np.array(imu_t, dtype=np.int64),
        'imu_rp': imu_rp,
        'enc_t': np.array(enc_t, dtype=np.int64),
        'enc': np.array(enc) if enc else np.zeros((0, 6)),
        'status': status_d,
    }


def _run_config(status_d):
    out = {}
    for s in reversed(status_d):
        if 'soft_max_vel_tps' not in out and 'soft_max_vel_turns_per_sec' in s:
            out['soft_max_vel_tps'] = float(s['soft_max_vel_turns_per_sec'])
        if 'leg_current_cap_a' not in out and 'leg_current_a' in s:
            out['leg_current_soft_max_a'] = float(s['leg_current_a'])
        if 'soft_max_vel_tps' in out and 'leg_current_soft_max_a' in out:
            break
    return out or None


def digest(bag_dir):
    print(f"[latency-bench] reading {bag_dir}")
    data = _read_bag(bag_dir)
    diag_t, diag = data['diag_t'], data['diag']
    imu_t, imu_rp = data['imu_t'], data['imu_rp']
    enc_t, enc = data['enc_t'], data['enc']

    if diag.shape[0] < 4 or imu_rp.shape[0] < 8:
        raise RuntimeError(
            "bench bag missing /latency_bench/diag or /platform/imu/data — "
            "was this recorded by the Latency Bench panel while armed?")

    t0 = int(min(diag_t[0], imu_t[0] if imu_t.size else diag_t[0]))
    diag_s = (diag_t - t0) * 1e-9
    imu_s = (imu_t - t0) * 1e-9 if imu_t.size else np.zeros((0,))
    enc_s = (enc_t - t0) * 1e-9 if enc_t.size else np.zeros((0,))

    cmd_pitch, cmd_roll = diag[:, 1], diag[:, 2]
    imu_roll = imu_rp[:, 0] if imu_rp.size else np.zeros((0,))
    imu_pitch = imu_rp[:, 1] if imu_rp.size else np.zeros((0,))

    # Driven axis = the commanded axis with the larger range.
    pitch_range = float(np.ptp(cmd_pitch)) if cmd_pitch.size else 0.0
    roll_range = float(np.ptp(cmd_roll)) if cmd_roll.size else 0.0
    driven = 'pitch' if pitch_range >= roll_range else 'roll'
    cmd_driven = cmd_pitch if driven == 'pitch' else cmd_roll

    # Response axis: the IMU axis that actually MOVES. The empirical IK
    # rotates the commanded axis, so a commanded 'pitch' can appear largely
    # in IMU roll — picking the larger-range IMU axis keeps the gain honest
    # instead of reading low because we compared mismatched axes.
    imu_p_rng = float(np.ptp(imu_pitch)) if imu_pitch.size else 0.0
    imu_r_rng = float(np.ptp(imu_roll)) if imu_roll.size else 0.0
    resp_axis = 'pitch' if imu_p_rng >= imu_r_rng else 'roll'
    imu_resp = imu_pitch if resp_axis == 'pitch' else imu_roll

    # Per-rep step metrics (multi-rep-robust): cmd → IMU, and cmd → the
    # most-driven leg encoder for the transport/slew split.
    imu_metrics = step_train_metrics(diag_s, cmd_driven, imu_s, imu_resp)
    leg_metrics = leg_i = None
    if enc.size:
        leg_ranges = [float(np.ptp(enc[:, i])) for i in range(enc.shape[1])]
        leg_i = int(np.argmax(leg_ranges))
        leg_metrics = step_train_metrics(diag_s, cmd_driven, enc_s,
                                         enc[:, leg_i])
        if leg_metrics is not None:
            leg_metrics['leg_index'] = leg_i

    # Per-stage decomposition from the per-rep median dead times.
    stages = None
    if imu_metrics is not None and imu_metrics.get('dead_time_ms') is not None:
        cmd_to_imu = imu_metrics['dead_time_ms']
        cmd_to_leg = leg_metrics.get('dead_time_ms') if leg_metrics else None
        stages = {
            'n_steps': imu_metrics.get('n_steps'),
            'driven_axis': driven,
            'resp_axis': resp_axis,
            'cmd_to_leg_onset_ms': cmd_to_leg,   # feeder ZOH + CAN + ODrive
            'leg_onset_to_imu_ms': (round(cmd_to_imu - cmd_to_leg, 1)
                                    if cmd_to_leg is not None else None),
            'cmd_to_imu_onset_ms': cmd_to_imu,   # total transport (dead time)
            'imu_rise_10_90_ms': imu_metrics.get('rise_10_90_ms'),
            'settle_ms': imu_metrics.get('settle_ms'),
            'overshoot_pct': imu_metrics.get('overshoot_pct'),
            'gain_imu_per_cmd': imu_metrics.get('gain'),
            'note': ('per-rep medians over n_steps. cmd→leg = feeder ZOH + '
                     'CAN + ODrive dead time; leg→IMU = leg slew → platform; '
                     'rise = mechanical. gain<1 ⇒ platform under-tilts the '
                     'command. Dead time is precise only if the diag captured '
                     'the step edges (low diag rate → coarse dead time).'),
        }

    summary = {
        'bag': os.path.abspath(bag_dir),
        'run_type': 'latency_bench',
        'driven_axis': driven,
        'topic_counts': dict(sorted(data['counts'].items())),
        'step_metrics': {'imu': imu_metrics, 'leg': leg_metrics,
                         'driven_axis': driven, 'resp_axis': resp_axis},
        'actuation_stages': stages,
        'run_config': _run_config(data['status']),
    }

    print(f"[latency-bench] driven={driven} resp_axis={resp_axis} "
          f"n_steps={imu_metrics.get('n_steps') if imu_metrics else 0}")
    if stages:
        print(f"  cmd→leg {stages['cmd_to_leg_onset_ms']}ms + leg→IMU "
              f"{stages['leg_onset_to_imu_ms']}ms = cmd→IMU dead "
              f"{stages['cmd_to_imu_onset_ms']}ms; rise "
              f"{stages['imu_rise_10_90_ms']}ms; settle "
              f"{stages['settle_ms']}ms; overshoot "
              f"{stages['overshoot_pct']}%")
    if imu_metrics:
        g = imu_metrics.get('gain')
        print(f"  gain={g} (IMU {resp_axis}/cmd {driven}) — "
              f"{'UNDER-TILTING' if (g is not None and g < 0.8) else 'ok'}")

    # ----- Plot: cmd vs IMU tilt timeline -----
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        plt = None
    if plt is not None:
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        for ax, axis, cmd, imu in (
                (axes[0], 'pitch', cmd_pitch, imu_pitch),
                (axes[1], 'roll', cmd_roll, imu_roll)):
            ax.plot(diag_s, cmd, color='#0ea5e9', lw=1.2,
                    label=f'cmd {axis} [deg]')
            if imu.size:
                ax.plot(imu_s, imu, color='#64748b', lw=0.8, alpha=0.8,
                        label=f'IMU {axis} [deg]')
            ax.set_ylabel(f'{axis} [deg]')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8, loc='upper right')
            if axis == driven:
                ax.set_facecolor('#0b1220')
        axes[1].set_xlabel('time [s]')
        sub = ''
        if stages:
            sub = (f"dead {stages['cmd_to_imu_onset_ms']}ms "
                   f"(cmd→leg {stages['cmd_to_leg_onset_ms']} + leg→IMU "
                   f"{stages['leg_onset_to_imu_ms']})  |  rise "
                   f"{stages['imu_rise_10_90_ms']}ms  |  settle "
                   f"{stages['settle_ms']}ms  |  overshoot "
                   f"{stages['overshoot_pct']}%")
        fig.suptitle(f"Latency Bench — {os.path.basename(bag_dir)} "
                     f"(driven: {driven})\n{sub}", fontsize=9)
        png = os.path.join(bag_dir, 'digest.png')
        fig.savefig(png, dpi=110, bbox_inches='tight')
        plt.close(fig)
        print(f"[latency-bench] wrote {png}")

    js = os.path.join(bag_dir, 'digest.summary.json')
    with open(js, 'w') as f:
        json.dump(summary, f, indent=2, default=str)
        f.write('\n')
    print(f"[latency-bench] wrote {js}")


def main():
    if len(sys.argv) < 2 or not os.path.isdir(sys.argv[1]):
        print("usage: digest_latency_bench.py <bag_dir>", file=sys.stderr)
        return 2
    digest(sys.argv[1])
    return 0


if __name__ == '__main__':
    sys.exit(main())
