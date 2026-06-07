#!/usr/bin/env python3
"""digest_model_bag.py — per-bag digest for the still (meas-noise) and coasting
(rolling-resistance) data campaigns.

This does NOT add precision over the fitter (it reuses the same unit-tested
math in stewart_bringup._ball_model). Its value is per-bag QA: run it on a
single bag to see that bag's number + a plot, so you can spot a bad take (ball
not detected, never actually still, a dirty coast) BEFORE it pollutes the
combined fit — and the number lands in the GUI data list.

  meas-noise  → R (vision std, mm) from the still-ball scatter
  coast       → c_roll (mm/s²) + c_visc (1/s) from the speed decay

Writes digest.png + digest.summary.json next to the bag. The GUI 'digest'
button routes meas-noise / coast / model_data bags here. RUNS ON THE PI.
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

from stewart_bringup._ball_model import (                # noqa: E402
    fit_measurement_noise, fit_rolling_resistance, resample_uniform,
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


def read_ball_state(bag_dir):
    reader = _open_bag(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    t0 = None
    t_s, px, py, vx, vy = [], [], [], [], []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic != '/ball_state' or types.get(topic) != \
                'geometry_msgs/msg/PoseStamped':
            continue
        try:
            m = deserialize_message(raw, PoseStamped)
        except Exception:
            continue
        if t0 is None:
            t0 = t_ns
        t_s.append((t_ns - t0) * 1e-9)
        px.append(float(m.pose.position.x))
        py.append(float(m.pose.position.y))
        vx.append(float(m.pose.orientation.x))
        vy.append(float(m.pose.orientation.y))
    return t_s, px, py, vx, vy


def _decel_points(t_s, speed, v_floor=20.0, max_decel=3000.0):
    """(v, decel) finite-difference points on the decelerating part — mirrors
    fit_rolling_resistance's regression input (incl. the collision reject), for
    the plot."""
    vs, ds = [], []
    for i in range(len(t_s) - 1):
        dt = t_s[i + 1] - t_s[i]
        if dt < 0.005:                  # skip near-duplicate timestamps
            continue
        v = 0.5 * (speed[i] + speed[i + 1])
        d = -(speed[i + 1] - speed[i]) / dt
        if v >= v_floor and 0 < d <= max_decel:
            vs.append(v)
            ds.append(d)
    return vs, ds


def _med(xs):
    s = sorted(xs)
    n = len(s)
    if not n:
        return 0.0
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def _series_stats(t_s):
    """Sample-timing diagnostics. A tiny dt (near-duplicate timestamps) makes
    the finite-difference deceleration explode (Δv / ~0 = huge), and a large
    gap is a detection dropout — both masquerade as bogus 'decel' spikes."""
    dts = [t_s[i + 1] - t_s[i]
           for i in range(len(t_s) - 1) if t_s[i + 1] > t_s[i]]
    if not dts:
        return dict(hz=0.0, median_dt_ms=0.0, min_dt_ms=0.0, max_dt_ms=0.0,
                    n_tiny_dt=0, n_gaps=0)
    md = _med(dts)
    return dict(
        hz=round(1.0 / md, 1) if md > 0 else 0.0,
        median_dt_ms=round(md * 1e3, 2),
        min_dt_ms=round(min(dts) * 1e3, 3),
        max_dt_ms=round(max(dts) * 1e3, 1),
        n_tiny_dt=sum(1 for d in dts if d < 0.005),   # <5 ms ≈ dup timestamps
        n_gaps=sum(1 for d in dts if d > 3 * md),      # dropouts
    )


def _pos_speed(t_s, px, py):
    """Speed from finite-differencing the KF POSITION — an independent check on
    the KF velocity field (orientation.x/y). If this is smooth while the KF
    speed is spiky, the velocity field (not the ball motion) is the culprit."""
    sp = [0.0]
    for i in range(1, len(t_s)):
        dt = t_s[i] - t_s[i - 1]
        sp.append(math.hypot(px[i] - px[i - 1], py[i] - py[i - 1]) / dt
                  if dt > 0 else 0.0)
    return sp


def _plot_still(bag, px, py, r):
    if plt is None:
        return
    try:
        mx = sum(px) / len(px)
        my = sum(py) / len(py)
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.scatter([x - mx for x in px], [y - my for y in py],
                   s=6, alpha=0.4, color='tab:blue')
        ax.set_aspect('equal')
        ax.set_xlabel('x − mean (mm)')
        ax.set_ylabel('y − mean (mm)')
        ttl = 'still ball scatter'
        if r:
            ttl += (f"  —  R={r['R_mm']} mm "
                    f"(σx {r['std_x_mm']}, σy {r['std_y_mm']}, n={r['n']})")
        ax.set_title(ttl, fontsize=9)
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(bag, 'digest.png'), dpi=90)
        plt.close(fig)
    except Exception as e:
        print(f"(plot failed: {e})", file=sys.stderr)


def _plot_coast(bag, t_s, px, py, kf_speed, pos_speed, tg, vg, rr):
    if plt is None:
        return
    try:
        fig, (a0, a1, a2) = plt.subplots(1, 3, figsize=(15, 4.5))
        a0.plot(t_s, px, color='tab:blue', lw=1, label='x')
        a0.plot(t_s, py, color='tab:orange', lw=1, label='y')
        a0.set_xlabel('t (s)')
        a0.set_ylabel('position (mm)')
        a0.set_title('ball position (is it a clean coast?)')
        a0.legend(fontsize=8)
        a0.grid(alpha=0.3)
        a1.plot(t_s, kf_speed, color='tab:red', lw=0.6, alpha=0.35,
                label='KF |v| (raw, bursty)')
        a1.plot(tg, vg, color='black', lw=1.4, label='resampled 50 Hz (fit input)')
        a1.set_ylim(0, max(1.0, 1.3 * (max(vg) if vg else 1.0)))
        a1.set_xlabel('t (s)')
        a1.set_ylabel('|v| (mm/s)')
        a1.set_title('speed: raw KF vs de-bursted')
        a1.legend(fontsize=8)
        a1.grid(alpha=0.3)
        vs, ds = _decel_points(tg, vg)
        a2.scatter(vs, ds, s=10, alpha=0.5, color='tab:gray',
                   label='decel pts (resampled)')
        if rr and vs:
            xs = [min(vs), max(vs)]
            a2.plot(xs, [rr['c_roll'] + rr['c_visc'] * x for x in xs],
                    color='tab:red',
                    label=f"fit c_roll={rr['c_roll']}, c_visc={rr['c_visc']}")
        a2.set_xlabel('|v| (mm/s)')
        a2.set_ylabel('decel (mm/s²)')
        a2.set_title('rolling resistance fit')
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
    ap.add_argument('bag', help='meas-noise / coast bag directory')
    a = ap.parse_args()

    base = os.path.basename(a.bag.rstrip('/\\'))
    campaign = ('meas_noise' if 'meas_noise' in base
                else 'coast' if 'coast' in base else 'model_data')
    t_s, px, py, vx, vy = read_ball_state(a.bag)
    n = len(t_s)
    dur = round(t_s[-1] - t_s[0], 1) if n >= 2 else None
    summary = {'run_type': 'model_data', 'campaign': campaign,
               'duration_s': dur, 'ball_state_n': n}

    if n < 10:
        summary['reason'] = f'only {n} /ball_state samples — ball detected?'
        with open(os.path.join(a.bag, 'digest.summary.json'), 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"model digest: too few ball samples ({n}) — {campaign}")
        return

    if campaign == 'meas_noise':
        r = fit_measurement_noise(px, py)
        if r:
            summary.update(R_mm=r['R_mm'], std_x_mm=r['std_x_mm'],
                           std_y_mm=r['std_y_mm'])
        else:
            summary['reason'] = 'noise fit failed'
        _plot_still(a.bag, px, py, r)
        msg = (f"meas-noise: R={r['R_mm']} mm "
               f"(σx {r['std_x_mm']}, σy {r['std_y_mm']}, n={r['n']})"
               if r else "meas-noise: fit failed")
    else:
        kf_speed = [math.hypot(vx[i], vy[i]) for i in range(n)]
        pos_speed = _pos_speed(t_s, px, py)
        st = _series_stats(t_s)
        # De-burst onto a uniform 50 Hz grid (median per bin) before fitting —
        # this is what makes the fit immune to the KF's dual-publish bursts.
        tg, vg = resample_uniform(t_s, kf_speed, 0.02)
        rr = fit_rolling_resistance(tg, vg)
        summary.update(
            ball_state_hz=st['hz'], median_dt_ms=st['median_dt_ms'],
            min_dt_ms=st['min_dt_ms'], max_dt_ms=st['max_dt_ms'],
            n_tiny_dt=st['n_tiny_dt'], n_gaps=st['n_gaps'],
            resampled_n=len(tg),
            kf_speed_max=round(max(kf_speed), 1),
            kf_speed_med=round(_med(kf_speed), 1),
            pos_speed_max=round(max(pos_speed), 1),
            pos_speed_med=round(_med(pos_speed), 1))
        if rr:
            summary.update(c_roll=rr['c_roll'], c_visc=rr['c_visc'],
                           fit_n=rr['n'])
        else:
            summary['reason'] = 'no clean decel after de-burst — flick a bit harder'
        _plot_coast(a.bag, t_s, px, py, kf_speed, pos_speed, tg, vg, rr)
        msg = (f"coast (resampled 50Hz): c_roll={rr['c_roll'] if rr else '—'} "
               f"c_visc={rr['c_visc'] if rr else '—'} (n={rr['n'] if rr else 0})"
               f" | raw {st['hz']}Hz min_dt {st['min_dt_ms']}ms "
               f"tiny×{st['n_tiny_dt']} | kf|v|med {summary['kf_speed_med']}")

    with open(os.path.join(a.bag, 'digest.summary.json'), 'w',
              encoding='utf-8') as f:
        json.dump(summary, f, indent=2)
    print(msg)


if __name__ == '__main__':
    main()
