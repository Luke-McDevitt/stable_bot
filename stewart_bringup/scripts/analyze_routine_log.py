#!/usr/bin/env python3
"""
Offline analyzer for a recorded routine log.

Given a log directory produced by the `rec` checkbox in the GUI (one of
the ~/ros2_ws/src/stewart_bringup/logs/<stamp>_<name>/ folders), this
script:

  1. Loads routine.json, metadata.json, telemetry.csv.
  2. Plots commanded xyz/rpy vs actual IMU rpy (so we can see how well the
     motors + PI level tracked the commanded motion).
  3. Simulates a solid sphere (default = golf ball: 42.67 mm, 45.9 g)
     rolling-without-slipping on a 200 mm radius carbon-fiber disk, using
     the measured IMU tilt as the input, and plots the ball trajectory in
     the platform frame.
  4. Reports whether/when/where the ball would escape the disk edge.

Physics used:
  a_ball = (5/7) * g * sin(θ) * downhill_unit_vec   (solid sphere, no slip)
  Ball treated as a point in the 2D platform plane; the platform's own
  lateral translation (from IK xyz commands) is applied as a moving frame.
  Simple rolling-resistance damping can be enabled with --mu-r.

Usage:
  python3 analyze_routine_log.py ~/ros2_ws/src/stewart_bringup/logs/2026-04-20_15...
  python3 analyze_routine_log.py LATEST     # uses most recent log dir
  python3 analyze_routine_log.py --all      # bulk-process every log dir
  python3 analyze_routine_log.py <dir> --ball-mass-g 50 --ball-dia-mm 45
  python3 analyze_routine_log.py <dir> --mu-r 0.02     # add rolling friction
  python3 analyze_routine_log.py <dir> --no-plot       # text report only

Bulk mode (--all):
  - saves analysis.png inside every log dir (forces --save)
  - writes summary.csv at the top level with a one-row-per-run comparison
  - skips any log that's missing the required columns (prints why)

Requires: numpy, matplotlib (system python; no ROS sourcing needed).
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

try:
    import numpy as np
except ImportError:
    sys.exit("numpy not installed — `pip install numpy` (or rely on the ROS env)")

LOGS_DIR = os.path.expanduser('~/ros2_ws/src/stewart_bringup/logs')


def find_latest_log():
    dirs = sorted(glob.glob(os.path.join(LOGS_DIR, '*')))
    dirs = [d for d in dirs if os.path.isdir(d)]
    if not dirs:
        sys.exit(f"no log folders in {LOGS_DIR}")
    return dirs[-1]


# Columns the analyzer REQUIRES to run. Older logs that predate a feature
# are skipped in --all mode if any of these is missing.
REQUIRED_COLS = ['t_s', 'imu_roll', 'imu_pitch', 'cmd_roll', 'cmd_pitch']

# Columns the analyzer uses but can do without; filled with NaN if missing.
KNOWN_COLS = [
    't_s',
    'cmd_x', 'cmd_y', 'cmd_z', 'cmd_roll', 'cmd_pitch', 'cmd_yaw',
    'imu_roll', 'imu_pitch', 'imu_yaw',
    'imu_ax', 'imu_ay', 'imu_az',
    'gyro_x', 'gyro_y', 'gyro_z',
    'enc_0', 'enc_1', 'enc_2', 'enc_3', 'enc_4', 'enc_5',
    'iq_0', 'iq_1', 'iq_2', 'iq_3', 'iq_4', 'iq_5',
    'tilt_corr_r', 'tilt_corr_p',
    'level_enabled',
    'soft_max_vel_tps', 'leg_current_a',
    'mode_0', 'mode_1', 'mode_2', 'mode_3', 'mode_4', 'mode_5',
    'routine_elapsed_s', 'routine_duration_s',
]


def load_telemetry(csv_path):
    """Return {col: np.array}. Raises ValueError if required cols missing
    or if the file has no rows. Fills known-but-missing cols with NaN."""
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"{csv_path} has no rows")
    present = list(rows[0].keys())
    cols = {}
    for k in present:
        cols[k] = np.array([float(r[k]) if r[k] not in ('', 'nan', 'NaN')
                            else np.nan for r in rows])
    n = len(rows)
    # Check required columns present
    missing_required = [c for c in REQUIRED_COLS if c not in cols]
    if missing_required:
        raise ValueError(
            f"{csv_path} missing required columns: {missing_required}")
    # Pad known-but-absent columns with NaN so downstream code never
    # hits KeyError.
    for k in KNOWN_COLS:
        if k not in cols:
            cols[k] = np.full(n, np.nan)
    return cols


def simulate_ball(t, roll_deg, pitch_deg, level_ref=(0.0, 0.0),
                  ball_dia_mm=42.67, ball_mass_g=45.93,
                  disk_radius_mm=200.0, mu_rolling=0.0,
                  initial_pos_mm=(0.0, 0.0), initial_vel_mms=(0.0, 0.0)):
    """Integrate a solid-sphere ball rolling on the tilted platform.

    Tilt angles are interpreted as world-frame IMU readings; we subtract
    the level_ref so (roll=0, pitch=0) corresponds to actually flat.

    Returns a dict with 'x_mm', 'y_mm', 'vx_mms', 'vy_mms' arrays (all
    aligned to `t`), plus 'escape_idx' or None."""
    g = 9810.0  # mm/s^2
    n = len(t)
    # Subtract level offset so zero-tilt means world-flat
    rr = np.radians(np.asarray(roll_deg)  - level_ref[0])
    pp = np.radians(np.asarray(pitch_deg) - level_ref[1])

    x = np.zeros(n); y = np.zeros(n)
    vx = np.zeros(n); vy = np.zeros(n)
    x[0], y[0] = initial_pos_mm
    vx[0], vy[0] = initial_vel_mms

    R_disk = disk_radius_mm - ball_dia_mm / 2  # edge allowance

    # Solid-sphere rolling-without-slipping coefficient. Ball accel is
    # (5/7) * (component of g along the surface). In small-angle:
    #   a_x = (5/7) * g * sin(pitch)     (+x-slope = downhill in +x)
    #   a_y = (5/7) * g * (-sin(roll))   (+roll tilts +y UP, so downhill -y)
    K = 5.0 / 7.0
    escape_idx = None
    for i in range(1, n):
        dt = t[i] - t[i-1]
        if dt <= 0:
            x[i], y[i], vx[i], vy[i] = x[i-1], y[i-1], vx[i-1], vy[i-1]
            continue
        ax = K * g * math.sin(pp[i-1])
        ay = K * g * (-math.sin(rr[i-1]))
        # Simple rolling friction opposing motion
        if mu_rolling > 0:
            speed = math.hypot(vx[i-1], vy[i-1])
            if speed > 1e-6:
                ax -= mu_rolling * g * vx[i-1] / speed
                ay -= mu_rolling * g * vy[i-1] / speed
        vx[i] = vx[i-1] + ax * dt
        vy[i] = vy[i-1] + ay * dt
        x[i] = x[i-1] + vx[i-1] * dt + 0.5 * ax * dt * dt
        y[i] = y[i-1] + vy[i-1] * dt + 0.5 * ay * dt * dt
        r = math.hypot(x[i], y[i])
        if r > R_disk and escape_idx is None:
            escape_idx = i
    return {
        'x_mm': x, 'y_mm': y, 'vx_mms': vx, 'vy_mms': vy,
        'escape_idx': escape_idx, 'R_disk_mm': R_disk,
        'ball_dia_mm': ball_dia_mm, 'ball_mass_g': ball_mass_g,
        'mu_rolling': mu_rolling,
    }


def report(log_dir, meta, routine, tele, sim, args):
    print(f"\n=== log: {log_dir} ===")
    if routine is not None:
        print(f"routine: {routine.get('name')} "
              f"({routine.get('keyframe_count', len(routine.get('keyframes', [])))} kf)")
        p = routine.get('params')
        if p:
            print(f"params: {json.dumps(p, indent=None)}")
    dur = tele['t_s'][-1] - tele['t_s'][0]
    print(f"telemetry: {len(tele['t_s'])} samples, {dur:.1f} s "
          f"(~{len(tele['t_s'])/max(dur,1e-6):.0f} Hz)")

    # Tracking errors: commanded vs actual
    cr = tele['cmd_roll']; cp = tele['cmd_pitch']
    ir = tele['imu_roll']; ip = tele['imu_pitch']
    # Remove level offset so both are in world-flat frame
    lr = meta.get('level_cal') or {}
    ref_r = lr.get('ref_roll_deg',  0.0) if lr else 0.0
    ref_p = lr.get('ref_pitch_deg', 0.0) if lr else 0.0
    actual_r = ir - ref_r
    actual_p = ip - ref_p
    err_r = cr - actual_r
    err_p = cp - actual_p
    # ignore rows where imu is nan
    mask = np.isfinite(err_r) & np.isfinite(err_p)
    if mask.any():
        print(f"tracking err roll:  mean={np.nanmean(err_r[mask]):+.3f}°  "
              f"rms={np.sqrt(np.nanmean(err_r[mask]**2)):.3f}°  "
              f"max|{np.nanmax(np.abs(err_r[mask])):.3f}°")
        print(f"tracking err pitch: mean={np.nanmean(err_p[mask]):+.3f}°  "
              f"rms={np.sqrt(np.nanmean(err_p[mask]**2)):.3f}°  "
              f"max|{np.nanmax(np.abs(err_p[mask])):.3f}°")

    # Leg current peaks
    for i in range(6):
        iq = tele[f'iq_{i}']
        iq = iq[np.isfinite(iq)]
        if iq.size:
            print(f"  leg {i}: peak |Iq| = {np.nanmax(np.abs(iq)):.2f} A, "
                  f"rms = {np.sqrt(np.nanmean(iq**2)):.2f} A")

    # Ball simulation report
    esc = sim['escape_idx']
    t = tele['t_s']
    R_disk = sim['R_disk_mm']
    if esc is None:
        r_end = math.hypot(sim['x_mm'][-1], sim['y_mm'][-1])
        print(f"ball sim ({sim['ball_dia_mm']:.1f} mm, {sim['ball_mass_g']:.1f} g, "
              f"μ_r={sim['mu_rolling']}): STAYED on disk. Final r = "
              f"{r_end:.1f} mm (disk edge at {R_disk:.1f}).")
    else:
        t_esc = t[esc]
        ang = math.degrees(math.atan2(sim['y_mm'][esc], sim['x_mm'][esc]))
        print(f"ball sim ({sim['ball_dia_mm']:.1f} mm, {sim['ball_mass_g']:.1f} g, "
              f"μ_r={sim['mu_rolling']}): ESCAPED at t={t_esc:.2f}s, "
              f"angle={ang:+.0f}°. Max orbit radius reached = "
              f"{np.max(np.hypot(sim['x_mm'][:esc+1], sim['y_mm'][:esc+1])):.1f} mm.")
    print()


def plots(tele, sim, meta, save_dir=None):
    try:
        import matplotlib
        matplotlib.use('Agg' if save_dir else 'TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — skipping plots")
        return

    t = tele['t_s']
    lr = meta.get('level_cal') or {}
    ref_r = lr.get('ref_roll_deg',  0.0) if lr else 0.0
    ref_p = lr.get('ref_pitch_deg', 0.0) if lr else 0.0

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=False)

    ax = axes[0]
    ax.plot(t, tele['cmd_roll'],  label='cmd_roll',  lw=1)
    ax.plot(t, tele['imu_roll'] - ref_r, label='imu_roll (level-corrected)', lw=1, alpha=0.7)
    ax.plot(t, tele['cmd_pitch'], label='cmd_pitch', lw=1)
    ax.plot(t, tele['imu_pitch'] - ref_p, label='imu_pitch (level-corrected)', lw=1, alpha=0.7)
    ax.set_ylabel('deg'); ax.set_title('Commanded vs measured tilt'); ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(t, tele['cmd_z'], label='cmd_z (mm)', lw=1)
    for i in range(6):
        ax.plot(t, tele[f'enc_{i}'], label=f'enc_{i} (turns)', lw=0.6, alpha=0.6)
    ax.set_ylabel('mm / turns'); ax.set_title('Commanded Z and leg encoders')
    ax.legend(loc='upper right', fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    ax = axes[2]
    for i in range(6):
        ax.plot(t, tele[f'iq_{i}'], label=f'Iq_{i}', lw=0.7, alpha=0.8)
    ax.set_ylabel('A'); ax.set_title('Leg currents (Iq_measured)')
    ax.legend(loc='upper right', fontsize=7, ncol=2); ax.grid(True, alpha=0.3)

    # Ball sim 2D trajectory (final panel)
    ax = axes[3]
    ax.set_aspect('equal')
    theta = np.linspace(0, 2*np.pi, 200)
    ax.plot(sim['R_disk_mm']*np.cos(theta), sim['R_disk_mm']*np.sin(theta),
            'k--', alpha=0.4, label=f'disk edge ({sim["R_disk_mm"]:.0f} mm)')
    esc = sim['escape_idx']
    end = esc + 1 if esc is not None else len(sim['x_mm'])
    ax.plot(sim['x_mm'][:end], sim['y_mm'][:end], lw=1.0,
            label=f"ball (∅{sim['ball_dia_mm']:.1f} mm, μ_r={sim['mu_rolling']})")
    ax.plot([sim['x_mm'][0]], [sim['y_mm'][0]], 'go', label='start')
    if esc is not None:
        ax.plot([sim['x_mm'][esc]], [sim['y_mm'][esc]], 'rx',
                markersize=10, label=f'escape @ t={t[esc]:.2f}s')
    else:
        ax.plot([sim['x_mm'][-1]], [sim['y_mm'][-1]], 'b^', label='end')
    ax.set_xlabel('x (mm, platform frame)')
    ax.set_ylabel('y (mm)')
    ax.set_title('Simulated ball trajectory')
    ax.legend(loc='upper right', fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    if save_dir:
        out = os.path.join(save_dir, 'analysis.png')
        plt.savefig(out, dpi=110)
        print(f"plot saved to {out}")
    else:
        plt.show()


def _nan_safe_max_abs(a):
    a = a[np.isfinite(a)]
    return float(np.max(np.abs(a))) if a.size else float('nan')


def _nan_safe_rms(a):
    a = a[np.isfinite(a)]
    return float(np.sqrt(np.mean(a * a))) if a.size else float('nan')


def process_one(log_dir, args, silent=False):
    """Run analysis on one log dir. Returns a metrics dict (for summary
    aggregation) or None if the log was unusable. Always saves analysis.png
    into the log dir when --save is set OR when --all bulk mode is active
    (silent=True implies bulk)."""
    csv_path = os.path.join(log_dir, 'telemetry.csv')
    meta_path = os.path.join(log_dir, 'metadata.json')
    routine_path = os.path.join(log_dir, 'routine.json')
    if not os.path.exists(csv_path):
        if not silent:
            print(f"SKIP {log_dir}: no telemetry.csv")
        return None
    try:
        tele = load_telemetry(csv_path)
    except Exception as e:
        print(f"SKIP {log_dir}: {e}")
        return None

    meta = {}
    if os.path.exists(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except Exception:
            meta = {}
    routine = None
    if os.path.exists(routine_path):
        try:
            with open(routine_path) as f:
                routine = json.load(f)
        except Exception:
            routine = None

    lr = meta.get('level_cal') or {}
    ref = ((lr.get('ref_roll_deg', 0.0), lr.get('ref_pitch_deg', 0.0))
           if lr else (0.0, 0.0))
    ip = [float(s) for s in args.initial_pos_mm.split(',')]
    sim = simulate_ball(
        tele['t_s'], tele['imu_roll'], tele['imu_pitch'], level_ref=ref,
        ball_dia_mm=args.ball_dia_mm, ball_mass_g=args.ball_mass_g,
        disk_radius_mm=args.disk_radius_mm, mu_rolling=args.mu_r,
        initial_pos_mm=(ip[0], ip[1]) if len(ip) == 2 else (0, 0))

    if not silent:
        report(log_dir, meta, routine, tele, sim, args)
    if not args.no_plot:
        save_dir = log_dir if (args.save or silent) else None
        plots(tele, sim, meta, save_dir=save_dir)

    # Compute summary metrics for bulk aggregation
    ref_r = ref[0]; ref_p = ref[1]
    actual_r = tele['imu_roll']  - ref_r
    actual_p = tele['imu_pitch'] - ref_p
    err_r = tele['cmd_roll']  - actual_r
    err_p = tele['cmd_pitch'] - actual_p
    peak_iq = float('nan')
    for i in range(6):
        iq = tele.get(f'iq_{i}')
        if iq is None: continue
        val = _nan_safe_max_abs(iq)
        if np.isfinite(val) and (np.isnan(peak_iq) or val > peak_iq):
            peak_iq = val
    duration = float(tele['t_s'][-1] - tele['t_s'][0]) if len(tele['t_s']) else 0.0
    # Motor limits (take median over run to tolerate mid-run changes)
    def _med(col):
        a = tele.get(col)
        if a is None: return float('nan')
        a = a[np.isfinite(a)]
        return float(np.median(a)) if a.size else float('nan')
    params = routine.get('params', {}) if routine else {}
    # Feature version (for grouping pre/post improvements in the summary)
    features = meta.get('features', {}) if isinstance(meta, dict) else {}
    ver_bits = []
    if features.get('vel_feedforward'):    ver_bits.append('vff')
    if features.get('dynamic_level_ref'):  ver_bits.append('dynref')
    feature_tag = '+'.join(ver_bits) if ver_bits else 'legacy'

    metrics = {
        'log_dir': os.path.basename(log_dir),
        'routine': (routine.get('name') if routine
                    else meta.get('routine_name', '')),
        'version': feature_tag,
        'duration_s': round(duration, 2),
        'soft_max_vel_tps_med': round(_med('soft_max_vel_tps'), 3),
        'leg_current_a_med': round(_med('leg_current_a'), 2),
        'level_enabled_start': int(bool(meta.get('level_enabled_at_start'))),
        # Routine-generator params (for rolling-ball comparisons)
        'period_s': params.get('period_s', ''),
        'radius_m': params.get('radius_m', ''),
        'max_tilt_deg': params.get('max_tilt_deg', ''),
        'center_z_mm': params.get('center_z_mm', ''),
        'ramp_up_s': params.get('ramp_up_s', ''),
        'hold_cycles': params.get('hold_cycles', ''),
        # Tracking error (commanded vs measured, world-frame)
        'err_roll_rms_deg':  round(_nan_safe_rms(err_r), 3),
        'err_pitch_rms_deg': round(_nan_safe_rms(err_p), 3),
        'err_roll_max_abs':  round(_nan_safe_max_abs(err_r), 3),
        'err_pitch_max_abs': round(_nan_safe_max_abs(err_p), 3),
        # Ball sim outcome
        'ball_escaped': int(sim['escape_idx'] is not None),
        'ball_escape_s': (round(float(tele['t_s'][sim['escape_idx']]), 2)
                          if sim['escape_idx'] is not None else ''),
        'ball_max_r_mm': round(
            float(np.max(np.hypot(sim['x_mm'], sim['y_mm']))), 1),
        'peak_iq_any_leg_A': round(peak_iq, 2) if np.isfinite(peak_iq) else '',
    }
    return metrics


def write_summary(metrics_list, out_path):
    if not metrics_list:
        print("no usable logs — nothing to summarize")
        return
    # Union of all keys to tolerate per-run differences
    keys = []
    seen = set()
    for m in metrics_list:
        for k in m.keys():
            if k not in seen:
                keys.append(k); seen.add(k)
    with open(out_path, 'w') as f:
        f.write(','.join(keys) + '\n')
        for m in metrics_list:
            f.write(','.join(str(m.get(k, '')) for k in keys) + '\n')
    print(f"\nsummary written to {out_path} ({len(metrics_list)} rows)")


def print_summary_table(metrics_list):
    if not metrics_list:
        return
    cols = ['log_dir', 'routine', 'version', 'duration_s',
            'leg_current_a_med', 'soft_max_vel_tps_med', 'max_tilt_deg',
            'ball_escaped', 'ball_escape_s', 'ball_max_r_mm',
            'err_roll_rms_deg', 'err_pitch_rms_deg', 'peak_iq_any_leg_A']
    # column widths
    w = {c: max(len(c), max(len(str(m.get(c, ''))) for m in metrics_list))
         for c in cols}
    print('\n' + '  '.join(c.ljust(w[c]) for c in cols))
    print('  '.join('-' * w[c] for c in cols))
    for m in metrics_list:
        print('  '.join(str(m.get(c, '')).ljust(w[c]) for c in cols))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('log_dir', nargs='?', default='LATEST',
                   help='log directory, or "LATEST" for most recent')
    p.add_argument('--all', action='store_true',
                   help='bulk mode: iterate every log dir under ~/ros2_ws/src/stewart_bringup/logs/')
    p.add_argument('--ball-dia-mm', type=float, default=42.67,
                   help='ball diameter mm (default 42.67 = golf ball)')
    p.add_argument('--ball-mass-g', type=float, default=45.93,
                   help='ball mass g (default 45.93 = golf ball)')
    p.add_argument('--disk-radius-mm', type=float, default=200.0,
                   help='platform top disk radius mm (default 200)')
    p.add_argument('--mu-r', type=float, default=0.0,
                   help='rolling-friction coefficient (0 = idealized)')
    p.add_argument('--initial-pos-mm', type=str, default='0,0',
                   help='starting ball position (x,y) mm')
    p.add_argument('--no-plot', action='store_true',
                   help='skip matplotlib plots (text report only)')
    p.add_argument('--save', action='store_true',
                   help='save plot to analysis.png inside the log dir (implied by --all)')
    args = p.parse_args()

    if args.all:
        dirs = sorted(d for d in glob.glob(os.path.join(LOGS_DIR, '*'))
                      if os.path.isdir(d))
        if not dirs:
            sys.exit(f"no log folders in {LOGS_DIR}")
        print(f"bulk processing {len(dirs)} log directories")
        metrics_list = []
        for d in dirs:
            print(f"\n--- {os.path.basename(d)} ---")
            try:
                m = process_one(d, args, silent=True)
                if m is not None:
                    metrics_list.append(m)
                    esc = f"escape@{m['ball_escape_s']}s" if m['ball_escaped'] else 'on-disk'
                    print(f"  ok: RMS roll={m['err_roll_rms_deg']}° "
                          f"pitch={m['err_pitch_rms_deg']}°  {esc}")
            except Exception as e:
                print(f"  FAIL: {e}")
        write_summary(metrics_list,
                      os.path.join(LOGS_DIR, 'summary.csv'))
        print_summary_table(metrics_list)
        return

    # Single-log mode
    log_dir = find_latest_log() if args.log_dir == 'LATEST' else args.log_dir
    if not os.path.isdir(log_dir):
        sys.exit(f"not a directory: {log_dir}")
    process_one(log_dir, args, silent=False)


if __name__ == '__main__':
    main()
