#!/usr/bin/env python3
"""fit_ball_forward_model.py — Phase-1 ball forward-model fitter.

Reads the data-collection bags (recorded via the Physics Modeler GUI panel, or
any rosbag carrying /ball_state) and identifies the gray-box ball-on-plate
parameters, writing config/ball_model.yaml for the control predictor — which
you then review and **Push model.yaml** from the GUI.

RUNS ON THE PI: it needs rosbag2_py and the *raw* bags, which are gitignored
and stay on the Pi (only the resulting small YAML + fit report get pushed).

Campaigns (record them from the Physics Modeler panel):
  meas-noise   still ball             → R   : vision measurement noise (mm)
  coasting     level-plate flick(s)   → c_roll (mm/s²) + c_visc (1/s)
  breakaway    tilt ramp until it goes → θ_s (deg) → breakaway_a (mm/s²)

The identification MATH lives in stewart_bringup._ball_model (fit_measurement_
noise / fit_rolling_resistance / breakaway_a_from_theta) and is unit-tested on
synthetic data (test_ball_model.py). This script is only the bag-reading + I/O
shell around it, so the numbers are trustworthy before they ever touch a bag.

Examples (on the Pi, from the repo root):
  # point it at specific bags
  python3 stewart_bringup/scripts/fit_ball_forward_model.py \
      --meas-noise tuning_data/2026..._meas_noise \
      --coast      tuning_data/2026..._coast \
      --theta-s 5.2
  # or let it find the newest bag of each labelled campaign
  python3 stewart_bringup/scripts/fit_ball_forward_model.py --auto
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

# Make the package importable when run as a loose script (mirrors the digest).
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_PARENT = os.path.dirname(_HERE)
if _PKG_PARENT not in sys.path:
    sys.path.insert(0, _PKG_PARENT)

from stewart_bringup._ball_model import (          # noqa: E402
    BallParams, BallForwardModel, SOLID_ALPHA,
    fit_measurement_noise, fit_rolling_resistance, breakaway_a_from_theta,
    simulate,
)

try:
    import yaml
except Exception as e:                               # pragma: no cover
    print(f"ERROR: pyyaml unavailable: {e}", file=sys.stderr)
    sys.exit(2)

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float32MultiArray
except Exception as e:                               # pragma: no cover
    print(f"ERROR: rosbag2_py / rclpy unavailable — run this ON THE PI "
          f"inside the sourced ROS env. ({e})", file=sys.stderr)
    sys.exit(2)

REPO_ROOT = os.path.dirname(_PKG_PARENT)
DEFAULT_OUT = os.path.join(REPO_ROOT, 'stewart_bringup', 'config',
                           'ball_model.yaml')
TUNING_DIR = os.path.join(REPO_ROOT, 'tuning_data')


# ----- bag reading (mirrors digest_demo_bag._open_bag) -----------------------

def _open_bag(bag_dir: str):
    storage = StorageOptions(uri=bag_dir, storage_id='')
    converter = ConverterOptions('', '')
    reader = SequentialReader()
    try:
        reader.open(storage, converter)
        return reader
    except Exception as e_auto:
        mcaps = sorted(p for p in os.listdir(bag_dir) if p.endswith('.mcap'))
        last = e_auto
        for mf in mcaps:
            try:
                st = StorageOptions(uri=os.path.join(bag_dir, mf),
                                    storage_id='mcap')
                reader = SequentialReader()
                reader.open(st, converter)
                return reader
            except Exception as e_file:
                last = e_file
        raise RuntimeError(f"could not open bag at {bag_dir}: {last}")


def read_ball_state(bag_dir: str):
    """→ (t_s, px, py, vx, vy) lists from /ball_state (KF posterior). Position
    in pose.position.x/y (mm); velocity bagged in orientation.x/y (mm/s)."""
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


def read_breakaway_theta(bag_dir: str):
    """θ_s (deg) from a breakaway bag: the commanded tilt on the first
    /latency_bench/diag row flagged phase≥2 (ball broke loose). Returns
    (theta_deg, axis) or (None, None) if no breakaway was captured."""
    reader = _open_bag(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    max_tilt = 0.0
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic != '/latency_bench/diag' or types.get(topic) != \
                'std_msgs/msg/Float32MultiArray':
            continue
        try:
            d = list(deserialize_message(raw, Float32MultiArray).data)
        except Exception:
            continue
        if len(d) < 5:
            continue
        _, pitch, roll, phase, _speed = d[:5]
        max_tilt = max(max_tilt, abs(pitch), abs(roll))
        if phase >= 2.0:
            if abs(pitch) >= abs(roll):
                return round(abs(pitch), 3), 'pitch'
            return round(abs(roll), 3), 'roll'
    if max_tilt > 0.0:
        print(f"  [breakaway] no phase=2 row — ball never broke loose "
              f"(ramped to {max_tilt:.2f}°). Raise max-tilt or lower v_break.",
              file=sys.stderr)
    return None, None


def _find_latest(label: str):
    """Newest tuning_data bag whose name contains the campaign label."""
    hits = sorted(glob.glob(os.path.join(TUNING_DIR, f'*{label}*')),
                  key=os.path.getmtime)
    return hits[-1] if hits else None


# ----- validation ------------------------------------------------------------

def validate_coast(bag_dir: str, params: BallParams):
    """Replay the coast under the fitted model (level tilt) from each segment's
    first sample and report position RMS vs the recorded ball — the held-out
    check the plan asks for (§10). Coarse but honest: one straight replay."""
    t_s, px, py, vx, vy = read_ball_state(bag_dir)
    if len(t_s) < 20:
        return None
    dt = max(1e-3, (t_s[-1] - t_s[0]) / max(1, len(t_s) - 1))
    n = len(t_s)
    pxs, pys, _, _ = simulate(px[0], py[0], vx[0], vy[0],
                              [0.0] * (n - 1), [0.0] * (n - 1), dt, params)
    err = [math.hypot(pxs[i] - px[i], pys[i] - py[i]) for i in range(n)]
    return {'rms_mm': round((sum(e * e for e in err) / n) ** 0.5, 2),
            'max_mm': round(max(err), 2), 'n': n, 'dt_s': round(dt, 4)}


# ----- fit + write -----------------------------------------------------------

def fit(meas_bag, coast_bag, breakaway_bag, theta_s_deg, alpha, dt, out_path):
    report = {'inputs': {}, 'fit': {}, 'validation': {}}
    params = BallParams(alpha=alpha)

    if meas_bag:
        t_s, px, py, vx, vy = read_ball_state(meas_bag)
        r = fit_measurement_noise(px, py)
        report['inputs']['meas_noise_bag'] = meas_bag
        report['fit']['measurement_noise'] = r
        if r:
            print(f"  meas-noise: R≈{r['R_mm']} mm "
                  f"(σx {r['std_x_mm']}, σy {r['std_y_mm']}, n={r['n']})")

    if coast_bag:
        t_s, px, py, vx, vy = read_ball_state(coast_bag)
        speed = [math.hypot(vx[i], vy[i]) for i in range(len(vx))]
        r = fit_rolling_resistance(t_s, speed)
        report['inputs']['coast_bag'] = coast_bag
        report['fit']['rolling_resistance'] = r
        if r:
            params.c_roll = r['c_roll']
            params.c_visc = r['c_visc']
            print(f"  coasting:   c_roll={r['c_roll']} mm/s², "
                  f"c_visc={r['c_visc']} 1/s (n={r['n']})")
        else:
            print("  coasting:   not enough decelerating samples — skipped")

    # breakaway: explicit θ_s wins; else read it from a breakaway bag
    if theta_s_deg is None and breakaway_bag:
        theta_s_deg, axis = read_breakaway_theta(breakaway_bag)
        report['inputs']['breakaway_bag'] = breakaway_bag
        if theta_s_deg is not None:
            report['fit']['breakaway_axis'] = axis
    if theta_s_deg is not None:
        params.breakaway_a = round(breakaway_a_from_theta(theta_s_deg, alpha), 2)
        report['fit']['theta_s_deg'] = theta_s_deg
        report['fit']['breakaway_a'] = params.breakaway_a
        print(f"  breakaway:  θ_s={theta_s_deg}° → "
              f"breakaway_a={params.breakaway_a} mm/s²")

    if coast_bag:
        v = validate_coast(coast_bag, params)
        report['validation']['coast_replay'] = v
        if v:
            print(f"  validate:   coast-replay RMS {v['rms_mm']} mm "
                  f"(max {v['max_mm']}, n={v['n']})")

    model = BallForwardModel(params, dt=dt)
    doc = model.to_dict()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write("# ball_model.yaml — fitted by fit_ball_forward_model.py.\n"
                "# Review, then Push model.yaml from the Physics Modeler panel.\n")
        yaml.safe_dump(doc, f, sort_keys=False)
    report['output'] = out_path
    report['model'] = doc
    with open(out_path.replace('.yaml', '.fit.json'), 'w',
              encoding='utf-8') as f:
        json.dump(report, f, indent=2)
    return params, report


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--meas-noise', dest='meas', help='still-ball bag dir')
    ap.add_argument('--coast', help='coasting (flick) bag dir')
    ap.add_argument('--breakaway', help='breakaway bag dir (reads θ_s from it)')
    ap.add_argument('--theta-s', type=float, default=None,
                    help='breakaway tilt θ_s in deg (read off the GUI panel; '
                         'overrides --breakaway)')
    ap.add_argument('--auto', action='store_true',
                    help='find the newest tuning_data bag of each campaign '
                         '(meas_noise / coast / breakaway) automatically')
    ap.add_argument('--alpha', type=float, default=SOLID_ALPHA,
                    help='rolling coefficient (default 5/7 solid sphere)')
    ap.add_argument('--dt', type=float, default=0.01, help='integrator dt (s)')
    ap.add_argument('--out', default=DEFAULT_OUT, help='output yaml path')
    a = ap.parse_args()

    meas, coast, brk = a.meas, a.coast, a.breakaway
    if a.auto:
        meas = meas or _find_latest('meas_noise')
        coast = coast or _find_latest('coast')
        brk = brk or _find_latest('breakaway')
        print(f"--auto: meas={meas}\n        coast={coast}\n        brk={brk}")

    if not any([meas, coast, brk, a.theta_s is not None]):
        ap.error("nothing to fit — give --meas-noise/--coast/--breakaway/"
                 "--theta-s, or --auto")

    print("Fitting ball forward model:")
    params, report = fit(meas, coast, brk, a.theta_s, a.alpha, a.dt, a.out)
    print(f"\nWrote {a.out}")
    print(f"      {a.out.replace('.yaml', '.fit.json')}")
    print(f"  params: {params.to_dict()}")
    print("\nReview it, then hit 'Push model.yaml' in the Physics Modeler "
          "panel to ship it.")


if __name__ == '__main__':
    main()
