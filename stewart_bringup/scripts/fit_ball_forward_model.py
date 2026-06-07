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
    resample_uniform, simulate,
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
    """θ_s (deg) for a breakaway bag. Prefers the OFFLINE digest's precise
    value (digest.summary.json from digest_breakaway_bag.py — IMU tilt at the
    latency-corrected motion onset); falls back to the live /latency_bench/diag
    (7th element = start angle, else the commanded tilt at phase≥2). Returns
    (theta_deg, axis) or (None, None)."""
    summ = os.path.join(bag_dir, 'digest.summary.json')
    if os.path.isfile(summ):
        try:
            with open(summ) as f:
                s = json.load(f)
            if (s.get('run_type') == 'breakaway' and s.get('ok')
                    and s.get('theta_s_deg')):
                print(f"  [breakaway] using digested θ_s "
                      f"{s['theta_s_deg']}° (IMU+latency offline)")
                return round(float(s['theta_s_deg']), 3), s.get('axis')
        except Exception:
            pass
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
            axis = 'pitch' if abs(pitch) >= abs(roll) else 'roll'
            # 7th element (when present) is the true θ_s — the angle the ball
            # STARTED at; older 5/6-element bags fall back to the commanded
            # tilt on the confirmation row.
            theta = (d[6] if len(d) >= 7 and d[6] > 0.0
                     else max(abs(pitch), abs(roll)))
            return round(theta, 3), axis
    if max_tilt > 0.0:
        print(f"  [breakaway] no phase=2 row — ball never broke loose "
              f"(ramped to {max_tilt:.2f}°). Raise max-tilt or lower v_break.",
              file=sys.stderr)
    return None, None


def _find_latest(label: str, tuning_dir: str):
    """Newest tuning_data bag whose name contains the campaign label."""
    hits = sorted(glob.glob(os.path.join(tuning_dir, f'*{label}*')),
                  key=os.path.getmtime)
    return hits[-1] if hits else None


def _find_all(label: str, tuning_dir: str):
    """All tuning_data bags whose name contains the campaign label, oldest
    first."""
    return sorted(glob.glob(os.path.join(tuning_dir, f'*{label}*')),
                  key=os.path.getmtime)


def fit_one_coast(bag_dir: str):
    """Read one coast bag → {c_roll, c_visc, n} via the resampled-POSITION
    speed and the no-rectification regression (see _ball_model). None if the
    bag has no usable decay."""
    t_s, px, py, vx, vy = read_ball_state(bag_dir)
    if len(t_s) < 10:
        return None
    tg, pxg = resample_uniform(t_s, px, 0.02)
    _, pyg = resample_uniform(t_s, py, 0.02)
    vg = [0.0]
    for i in range(1, len(tg)):
        ddt = tg[i] - tg[i - 1]
        vg.append(math.hypot(pxg[i] - pxg[i - 1], pyg[i] - pyg[i - 1]) / ddt
                  if ddt > 0 else 0.0)
    return fit_rolling_resistance(tg, vg)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if not n:
        return None
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


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

    # Coast: fit every run and take the MEDIAN c_roll/c_visc. Deceleration is
    # a 2nd derivative of position, so a single run's value wobbles ~±20%;
    # the median over runs is robust to that AND to a bad take (too-short run,
    # a flick that bounced). Need n>=50 points to trust a run.
    coast_bags = ([coast_bag] if isinstance(coast_bag, str)
                  else list(coast_bag or []))
    per_run = []
    for b in coast_bags:
        r = fit_one_coast(b)
        if r and r.get('n', 0) >= 50:
            per_run.append((os.path.basename(b), r))
            print(f"  coasting:   {os.path.basename(b)}  "
                  f"c_roll={r['c_roll']} c_visc={r['c_visc']} (n={r['n']})")
        elif r:
            print(f"  coasting:   {os.path.basename(b)}  "
                  f"SKIP (only n={r.get('n')} pts)")
    if per_run:
        params.c_roll = round(_median([r['c_roll'] for _, r in per_run]), 2)
        params.c_visc = round(_median([r['c_visc'] for _, r in per_run]), 5)
        report['inputs']['coast_bags'] = [name for name, _ in per_run]
        report['fit']['rolling_resistance'] = {
            'c_roll_median': params.c_roll, 'c_visc_median': params.c_visc,
            'n_runs': len(per_run),
            'c_roll_per_run': [r['c_roll'] for _, r in per_run]}
        print(f"  coasting:   → MEDIAN c_roll={params.c_roll} mm/s², "
              f"c_visc={params.c_visc} 1/s over {len(per_run)} runs")
    elif coast_bags:
        print("  coasting:   no usable coast runs (need n>=50) — skipped")

    # breakaway: explicit θ_s wins; else take the MEDIAN θ_s over all breakaway
    # runs (same per-run spread as coast; median is robust to the plate-warp
    # variation between spots and to a no-breakaway run).
    if theta_s_deg is None and breakaway_bag:
        brk_bags = ([breakaway_bag] if isinstance(breakaway_bag, str)
                    else list(breakaway_bag or []))
        thetas = []
        for b in brk_bags:
            th, _ax = read_breakaway_theta(b)
            if th is not None:
                thetas.append(th)
                print(f"  breakaway:  {os.path.basename(b)}  θ_s={th}°")
        if thetas:
            theta_s_deg = round(_median(thetas), 3)
            report['inputs']['breakaway_bags'] = [os.path.basename(b)
                                                  for b in brk_bags]
            report['fit']['theta_s_per_run'] = sorted(thetas)
            if len(thetas) > 1:
                print(f"  breakaway:  → MEDIAN θ_s={theta_s_deg}° "
                      f"over {len(thetas)} runs")
    if theta_s_deg is not None:
        params.breakaway_a = round(breakaway_a_from_theta(theta_s_deg, alpha), 2)
        report['fit']['theta_s_deg'] = theta_s_deg
        report['fit']['breakaway_a'] = params.breakaway_a
        print(f"  breakaway:  θ_s={theta_s_deg}° → "
              f"breakaway_a={params.breakaway_a} mm/s²")

    if coast_bags:
        v = validate_coast(coast_bags[-1], params)
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
    ap.add_argument('--tuning-dir', default=None,
                    help='dir to search for --auto campaign bags '
                         '(default: <repo>/tuning_data)')
    ap.add_argument('--alpha', type=float, default=SOLID_ALPHA,
                    help='rolling coefficient (default 5/7 solid sphere)')
    ap.add_argument('--dt', type=float, default=0.01, help='integrator dt (s)')
    ap.add_argument('--out', default=DEFAULT_OUT, help='output yaml path')
    a = ap.parse_args()

    meas, coast, brk = a.meas, a.coast, a.breakaway
    if a.auto:
        tuning = a.tuning_dir or TUNING_DIR
        meas = meas or _find_latest('meas_noise', tuning)
        coast = coast or _find_all('coast', tuning)        # ALL → median
        brk = brk or _find_all('breakaway', tuning)        # ALL → median
        print(f"--auto (in {tuning}):\n  meas={meas}\n"
              f"  coast={len(coast) if isinstance(coast, list) else coast} run(s)"
              f"\n  brk={len(brk) if isinstance(brk, list) else brk} run(s)")

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
