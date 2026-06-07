#!/usr/bin/env python3
"""digest_breakaway_bag.py — offline breakaway angle (θ_s) from a ramp bag.

Why offline: the live daemon must decide in real time, but the ball is seen
~150 ms late, so the tilt at the moment motion is DETECTED over-reads the tilt
at the moment the ball actually moved. Here we find the displacement onset,
shift it back by the bag's own photon→state latency (carried on /ball_state),
and read the ACTUAL plate tilt from the IMU at that instant, referenced to the
settled baseline. That removes the latency error, the command-vs-actual error,
and the leveling-zero offset — the math is in stewart_bringup/_breakaway.py
(unit-tested). Pair a +axis and −axis run from the same spot and the half-sum
of the two θ_s cancels the plate's local slope.

Writes digest.png + digest.summary.json (run_type='breakaway') next to the
bag; the GUI 'digest' button runs it on breakaway bags and the fitter reads
θ_s from the summary. RUNS ON THE PI (rosbag2_py + the raw bag).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.dirname(_HERE)
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from stewart_bringup._breakaway import (                 # noqa: E402
    analyze_breakaway, quat_to_roll_pitch_deg, interp_at,
)

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except Exception:                                        # pragma: no cover
    plt = None

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray
    from sensor_msgs.msg import Imu
except Exception as e:                                    # pragma: no cover
    print(f"ERROR: rosbag2_py / rclpy unavailable — run on the Pi. ({e})",
          file=sys.stderr)
    sys.exit(2)


def _open_bag(bag_dir):
    converter = ConverterOptions('', '')
    try:
        reader = SequentialReader()
        reader.open(StorageOptions(uri=bag_dir, storage_id=''), converter)
        return reader
    except Exception as e_auto:
        last = e_auto
        for mf in sorted(p for p in os.listdir(bag_dir) if p.endswith('.mcap')):
            try:
                reader = SequentialReader()
                reader.open(StorageOptions(uri=os.path.join(bag_dir, mf),
                                           storage_id='mcap'), converter)
                return reader
            except Exception as e_file:
                last = e_file
        raise RuntimeError(f"could not open bag at {bag_dir}: {last}")


def read_bag(bag_dir):
    reader = _open_bag(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    t0 = None
    d = dict(ball_ts=[], px=[], py=[], lat=[], imu_ts=[], roll=[], pitch=[],
             diag_ts=[], pcmd=[], rcmd=[], disp=[])
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t0 is None:
            t0 = t_ns
        t = (t_ns - t0) * 1e-9
        tt = types.get(topic)
        try:
            if topic == '/ball_state' and tt == 'geometry_msgs/msg/PoseStamped':
                m = deserialize_message(raw, PoseStamped)
                d['ball_ts'].append(t)
                d['px'].append(float(m.pose.position.x))
                d['py'].append(float(m.pose.position.y))
                d['lat'].append(float(m.pose.orientation.z))
            elif topic == '/platform/imu/data' and tt == 'sensor_msgs/msg/Imu':
                q = deserialize_message(raw, Imu).orientation
                r, p = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)
                d['imu_ts'].append(t)
                d['roll'].append(r)
                d['pitch'].append(p)
            elif (topic == '/latency_bench/diag'
                  and tt == 'std_msgs/msg/Float32MultiArray'):
                a = list(deserialize_message(raw, Float32MultiArray).data)
                if len(a) >= 5:
                    d['diag_ts'].append(t)
                    d['pcmd'].append(float(a[1]))
                    d['rcmd'].append(float(a[2]))
                    d['disp'].append(float(a[5]) if len(a) >= 6 else 0.0)
        except Exception:
            continue
    return d


def _axis_direction(d):
    pk_p = max((abs(x) for x in d['pcmd']), default=0.0)
    pk_r = max((abs(x) for x in d['rcmd']), default=0.0)
    if pk_p == 0.0 and pk_r == 0.0:
        return None, None, []
    axis = 'pitch' if pk_p >= pk_r else 'roll'
    cmds = d['pcmd'] if axis == 'pitch' else d['rcmd']
    signed_pk = max(cmds, key=abs) if cmds else 0.0
    return axis, ('+' if signed_pk >= 0 else '-'), cmds


def _plot(bag, d, res, axis, min_travel):
    if plt is None:
        return
    try:
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
        if d['ball_ts']:
            x0, y0 = res.get('rest_xy_mm', [d['px'][0], d['py'][0]])
            disp = [math.hypot(d['px'][i] - x0, d['py'][i] - y0)
                    for i in range(len(d['ball_ts']))]
            a1.plot(d['ball_ts'], disp, color='tab:blue', label='ball travel')
        a1.axhline(min_travel, color='tab:gray', ls=':', lw=1,
                   label=f'min travel {min_travel:.0f} mm')
        if res.get('ok'):
            a1.axvline(res['onset_t'], color='tab:orange', ls='--',
                       label='onset (as seen)')
            a1.axvline(res['onset_t_corrected'], color='tab:green', ls='--',
                       label='onset (latency-corrected)')
        a1.set_ylabel('ball travel (mm)')
        a1.legend(fontsize=8)
        a1.grid(alpha=0.3)
        if d['imu_ts']:
            a2.plot(d['imu_ts'], d['roll'], color='tab:purple', label='IMU roll')
            a2.plot(d['imu_ts'], d['pitch'], color='tab:red', label='IMU pitch')
        if d['diag_ts'] and axis:
            a2.plot(d['diag_ts'], d['pcmd'] if axis == 'pitch' else d['rcmd'],
                    color='gray', ls=':', label='commanded')
        if res.get('ok'):
            a2.axvline(res['onset_t_corrected'], color='tab:green', ls='--')
            a2.set_title(f"θ_s = {res['theta_s_deg']}°  "
                         f"(IMU tilt change at corrected onset)")
        a2.set_xlabel('t (s)')
        a2.set_ylabel('tilt (deg)')
        a2.legend(fontsize=8)
        a2.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(bag, 'digest.png'), dpi=90)
        plt.close(fig)
    except Exception as e:
        print(f"(plot failed: {e})", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='breakaway bag directory')
    ap.add_argument('--min-travel', type=float, default=25.0,
                    help='mm the ball must roll to confirm (default 25)')
    ap.add_argument('--noise', type=float, default=4.0,
                    help='rest-band noise floor in mm (default 4)')
    a = ap.parse_args()

    d = read_bag(a.bag)
    res = analyze_breakaway(d['ball_ts'], d['px'], d['py'], d['lat'],
                            d['imu_ts'], d['roll'], d['pitch'],
                            min_travel_mm=a.min_travel, noise_mm=a.noise)
    axis, direction, cmds = _axis_direction(d)
    cmd_at_onset = None
    if res.get('ok') and cmds and d['diag_ts']:
        cmd_at_onset = interp_at(d['diag_ts'], [abs(x) for x in cmds],
                                 res['onset_t_corrected'])

    dur = (round(d['ball_ts'][-1] - d['ball_ts'][0], 1)
           if len(d['ball_ts']) >= 2 else None)
    summary = {
        'run_type': 'breakaway',
        'duration_s': dur,
        'axis': axis,
        'direction': direction,
        'theta_s_deg': res.get('theta_s_deg'),
        'commanded_tilt_at_onset_deg': (round(cmd_at_onset, 3)
                                        if cmd_at_onset is not None else None),
        'latency_s': res.get('latency_s'),
        'peak_travel_mm': res.get('peak_travel_mm'),
        'ok': bool(res.get('ok')),
        'reason': res.get('reason'),
        'method': ('IMU tilt change at latency-corrected displacement onset, '
                   'referenced to the settled baseline'),
        'ball_state_n': len(d['ball_ts']),
        'imu_n': len(d['imu_ts']),
    }
    with open(os.path.join(a.bag, 'digest.summary.json'), 'w',
              encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
    _plot(a.bag, d, res, axis, a.min_travel)

    if res.get('ok'):
        print(f"breakaway θ_s = {res['theta_s_deg']}° "
              f"({direction or ''}{axis or '?'}) — IMU at corrected onset; "
              f"commanded read {summary['commanded_tilt_at_onset_deg']}°, "
              f"latency {res['latency_s']}s, travel {res['peak_travel_mm']}mm "
              f"(Δroll {res['d_roll_deg']}°, Δpitch {res['d_pitch_deg']}°)")
    else:
        print(f"breakaway: NO confirmed result — {res.get('reason')} "
              f"(ball_state n={len(d['ball_ts'])}, imu n={len(d['imu_ts'])})")


if __name__ == '__main__':
    main()
