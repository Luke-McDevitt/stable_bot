#!/usr/bin/env python3
"""estimate_kf_noise.py — measure the ball KF's R + outlier rate from a bag.

Reads the RAW measurement (/ball_xy_mono) and the KF posterior (/ball_state),
isolates still windows, and reports:
  • R_mm           — robust raw-measurement scatter on a stationary ball
                     → set BALL_KF_R_MM
  • outlier_rate   — physically-impossible raw jumps (false positives)
                     → motivates the innovation gate
  • posterior |v| std in still windows — what the CURRENT KF produces at rest
                     (the spiky velocity we want gone); shrinks after tuning.

Works on meas-noise bags (one big still window) AND the winning demo/model bags
(the settled dwells). Pass several to pool. RUNS ON THE PI (rosbag2_py).

  python3 scripts/estimate_kf_noise.py tuning_data/*_meas_noise tuning_data/*_demo2
"""
from __future__ import annotations

import argparse
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.dirname(_HERE)
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from stewart_bringup._kf_noise import (                  # noqa: E402
    still_windows, measurement_noise, outlier_rate, _robust_std)

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PointStamped, PoseStamped
except Exception as e:                                    # pragma: no cover
    print(f"ERROR: rosbag2_py / rclpy unavailable — run on the Pi. ({e})",
          file=sys.stderr)
    sys.exit(2)


def _open(bag_dir):
    conv = ConverterOptions('', '')
    try:
        r = SequentialReader()
        r.open(StorageOptions(uri=bag_dir, storage_id=''), conv)
        return r
    except Exception as e_auto:
        last = e_auto
        for mf in sorted(p for p in os.listdir(bag_dir) if p.endswith('.mcap')):
            try:
                r = SequentialReader()
                r.open(StorageOptions(uri=os.path.join(bag_dir, mf),
                                      storage_id='mcap'), conv)
                return r
            except Exception as e:
                last = e
        raise RuntimeError(f"could not open {bag_dir}: {last}")


def _read(bag_dir):
    reader = _open(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    t0 = None
    mt, mx, my = [], [], []                 # raw measurement /ball_xy_mono
    st, sx, sy, sv = [], [], [], []         # posterior /ball_state pos + |v|
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if t0 is None:
            t0 = t_ns
        t = (t_ns - t0) * 1e-9
        tt = types.get(topic)
        try:
            if (topic == '/ball_xy_mono'
                    and tt == 'geometry_msgs/msg/PointStamped'):
                m = deserialize_message(raw, PointStamped)
                mt.append(t)
                mx.append(float(m.point.x))
                my.append(float(m.point.y))
            elif (topic == '/ball_state'
                  and tt == 'geometry_msgs/msg/PoseStamped'):
                m = deserialize_message(raw, PoseStamped)
                st.append(t)
                sx.append(float(m.pose.position.x))
                sy.append(float(m.pose.position.y))
                sv.append(math.hypot(float(m.pose.orientation.x),
                                     float(m.pose.orientation.y)))
        except Exception:
            continue
    return mt, mx, my, st, sx, sy, sv


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='bag dirs (meas-noise and/or demo)')
    ap.add_argument('--range-mm', type=float, default=12.0,
                    help='position range that counts as "still" (default 12)')
    a = ap.parse_args()

    pmt, pmx, pmy = [], [], []      # pooled raw measurement (shifted per bag)
    wins_all = []                   # pooled still windows (same time base)
    win_v = []                      # posterior |v| inside still windows
    for bag in a.bags:
        mt, mx, my, st, sx, sy, sv = _read(bag)
        if not mt or not st:
            print(f"  {os.path.basename(bag)}: no /ball_xy_mono or /ball_state")
            continue
        wins = still_windows(st, sx, sy, range_mm=a.range_mm)
        base = (pmt[-1] + 100.0) if pmt else 0.0    # keep bags non-overlapping
        pmt += [t + base for t in mt]
        pmx += mx
        pmy += my
        wins_all += [(t0 + base, t1 + base) for t0, t1 in wins]
        for t0, t1 in wins:
            win_v += [sv[k] for k in range(len(st)) if t0 <= st[k] <= t1]
        dur = sum(t1 - t0 for t0, t1 in wins)
        print(f"  {os.path.basename(bag)}: {len(wins)} still window(s), "
              f"{dur:.1f}s still, {len(mt)} raw meas")

    R = measurement_noise(pmt, pmx, pmy, wins_all)
    frac, n_out, n = outlier_rate(pmt, pmx, pmy)
    vstd = _robust_std(win_v)

    print("\n=== KF noise estimate ===")
    if R:
        print(f"  R (raw meas std, robust): {R['R_mm']} mm "
              f"(sx {R['std_x_mm']}, sy {R['std_y_mm']}, n={R['n']}, "
              f"windows={R['n_windows']})")
        print(f"    -> set BALL_KF_R_MM={R['R_mm']}")
    else:
        print("  R: not enough still data — record a still-ball bag")
    print(f"  outlier rate (raw jump > 1500 mm/s): {frac * 100:.2f}% "
          f"({n_out}/{n}) — false positives the gate drops")
    print(f"  posterior |v| at rest (current KF): {vstd:.0f} mm/s robust-sigma "
          f"(want ~0; high = velocity chasing vision noise, now gated)")


if __name__ == '__main__':
    main()
