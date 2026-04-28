#!/usr/bin/env python3
"""
analyze_level_bag.py — Phase 3 of docs/level_pi_tuning_plan.md.

Reads a /level_diag bag (single test) or a sweep_*/ dir, computes
the summary stats the tuning protocol needs, and writes:

  <out>/<name>_summary.json     — full stats (KB-scale, diff-able)
  <out>/<name>_timeseries.csv   — downsampled 50 Hz → 10 Hz timeseries

For sweep dirs, also writes <out>/<sweep>_sweep_summary.json with
the per-Z rollup (rms, fft peak, rise/overshoot/settling vs Z).

These outputs are meant to live under tuning_data/ in the repo so
they can be committed and reviewed remotely. Raw bags stay on the Pi.

Run with ROS sourced (system + workspace overlay), e.g.:
  source /opt/ros/kilted/setup.bash
  source ~/ros2_ws/install/local_setup.bash
  python3 analyze_level_bag.py ~/stable_bot_bags/sweep_<UTC>_<prefix>/
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

try:
    import rclpy.serialization
    import rosbag2_py
    from jugglebot_interfaces.msg import LevelDiag
except ImportError as e:
    sys.exit(
        f"Import failed ({e}). Source ROS first:\n"
        f"  source /opt/ros/kilted/setup.bash\n"
        f"  source ~/ros2_ws/install/local_setup.bash"
    )


def _detect_storage_id(bag_dir):
    """Decide which rosbag2 storage plugin to use for this bag.

    metadata.yaml is the authoritative source — every rosbag2 bag has
    one and it carries the storage_identifier ("mcap" or "sqlite3").
    Fall back to file extension if the YAML is unreadable, and
    finally to "mcap" because that's the Kilted default."""
    bag_dir = Path(bag_dir)
    meta = bag_dir / 'metadata.yaml'
    if meta.exists() and yaml is not None:
        try:
            with open(meta) as f:
                data = yaml.safe_load(f) or {}
            sid = (data.get('rosbag2_bagfile_information') or {}).get(
                'storage_identifier')
            if sid:
                return str(sid)
        except Exception:
            pass
    if any(bag_dir.glob('*.mcap')):
        return 'mcap'
    if any(bag_dir.glob('*.db3')):
        return 'sqlite3'
    return 'mcap'


# Bit indices match _level_run's clip_flags assembly in stewart_control_node.
CLIP_BITS = {
    'rate_limit_r':  1 << 0,
    'rate_limit_p':  1 << 1,
    'max_corr_r':    1 << 2,
    'max_corr_p':    1 << 3,
    'deadband_r':    1 << 4,
    'deadband_p':    1 << 5,
    'integ_clamp_r': 1 << 6,
    'integ_clamp_p': 1 << 7,
}

# ODrive 0.6.x active_errors bits we care about for level diagnostics.
# Source: ODrive_Pro.dbc + firmware error_codes.hpp. Not exhaustive —
# add bits as we encounter them in the wild.
ODRIVE_ERROR_BITS = {
    0x00000001: 'INITIALIZING',
    0x00000002: 'SYSTEM_LEVEL',
    0x00000004: 'TIMING_ERROR',
    0x00000008: 'MISSING_ESTIMATE',
    0x00000010: 'BAD_CONFIG',
    0x00000020: 'DRV_FAULT',
    0x00000040: 'MISSING_INPUT',
    0x00000100: 'DC_BUS_OVER_VOLTAGE',
    0x00000200: 'DC_BUS_UNDER_VOLTAGE',
    0x00000400: 'DC_BUS_OVER_CURRENT',
    0x00000800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x00001000: 'CURRENT_LIMIT_VIOLATION',
    0x00002000: 'MOTOR_OVER_TEMP',
    0x00004000: 'INVERTER_OVER_TEMP',
    0x00008000: 'VELOCITY_LIMIT_VIOLATION',
    0x00010000: 'POSITION_LIMIT_VIOLATION',
    0x01000000: 'WATCHDOG_TIMER_EXPIRED',
    0x02000000: 'ESTOP_REQUESTED',
    0x04000000: 'SPINOUT_DETECTED',
    0x08000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
}


def _decode_error_bits(code):
    if code == 0xFFFFFFFF or code == 0:
        return []
    out = []
    for bit, name in ODRIVE_ERROR_BITS.items():
        if code & bit:
            out.append(name)
    if not out:
        out.append(f'unknown(0x{code:08x})')
    return out


def read_level_diag(bag_dir):
    """Iterate /level_diag in the bag and return [(t_s, msg), ...].

    Auto-detects the storage backend (mcap on Kilted defaults; sqlite3
    on older or explicitly-configured workspaces)."""
    sid = _detect_storage_id(bag_dir)
    storage = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=sid)
    converter = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)
    out = []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == '/level_diag':
            msg = rclpy.serialization.deserialize_message(data, LevelDiag)
            out.append((t_ns / 1e9, msg))
    return out


def _arr(samples, attr):
    return np.array([getattr(m, attr) for _t, m in samples], dtype=float)


def detect_step_edges(samples, threshold=0.5, low=0.05):
    """Detect rising step edges in target_roll. Returns [(t_edge, amp), ...].
    Triggers on transition from |target| < low → |target| >= threshold."""
    edges = []
    high = False
    for t, m in samples:
        cur = float(m.target_roll)
        if not high and abs(cur) >= threshold:
            edges.append((t, cur))
            high = True
        elif high and abs(cur) < low:
            high = False
    return edges


def compute_baseline(samples, end_s):
    """Stats over [0, end_s] of the bag — the no-input portion that
    auto-sweep records before the step battery."""
    if not samples:
        return None
    t0 = samples[0][0]
    base = [(t, m) for t, m in samples if (t - t0) < end_s]
    if len(base) < 10:
        return None
    err_r = _arr(base, 'err_r_filt')
    err_p = _arr(base, 'err_p_filt')
    clip = _arr(base, 'clip_flags').astype(np.int64)
    n = len(base)

    # PSD via Hann-windowed rfft on err_r, last min(25 s, available).
    dt = float(np.median(np.diff([t for t, _ in base])))
    sr = 1.0 / dt if dt > 0 else 50.0
    win_n = min(len(err_r), int(round(25 * sr)))
    win = err_r[-win_n:] - err_r[-win_n:].mean()
    if win_n >= 8:
        hann = np.hanning(win_n)
        F = np.fft.rfft(win * hann)
        freqs = np.fft.rfftfreq(win_n, 1 / sr)
        mag = np.abs(F) * (2.0 / np.sum(hann))
        # Skip DC; find peak in 0.05 Hz < f < sr/2 band
        valid = freqs > 0.05
        if valid.any():
            sub = mag[valid]
            sf = freqs[valid]
            i = int(np.argmax(sub))
            peak_f = float(sf[i])
            peak_a = float(sub[i])
        else:
            peak_f, peak_a = 0.0, 0.0
    else:
        peak_f, peak_a = 0.0, 0.0

    sat = {k: float(((clip & v) != 0).sum() / n * 100) for k, v in CLIP_BITS.items()}
    return {
        'window_s': [0.0, float(base[-1][0] - t0)],
        'samples': int(n),
        'sample_rate_hz': float(sr),
        'rms_roll_deg': float(np.sqrt(np.mean(err_r ** 2))),
        'rms_pitch_deg': float(np.sqrt(np.mean(err_p ** 2))),
        'p2p_roll_deg': float(np.ptp(err_r)),
        'p2p_pitch_deg': float(np.ptp(err_p)),
        'mean_roll_deg': float(err_r.mean()),
        'mean_pitch_deg': float(err_p.mean()),
        'std_roll_deg': float(err_r.std()),
        'std_pitch_deg': float(err_p.std()),
        'fft_peak_freq_hz': peak_f,
        'fft_peak_amp_deg': peak_a,
        'saturation_pct_any': float((clip != 0).sum() / n * 100),
        'saturation_pct_by_flag': sat,
    }


def step_metrics(samples, t_edge, amp, hold_s, ss_band_deg=0.1):
    """Single step trial → rise/overshoot/settling/ss-offset.

    Pre-step mean (1 s before the edge) is subtracted from roll so the
    baseline limit-cycle DC offset doesn't pollute the transient — same
    treatment described in level_pi_tuning_plan.md Phase 3.
    """
    pre = [(t, m) for t, m in samples
           if t_edge - 1.0 <= t < t_edge]
    post = [(t, m) for t, m in samples
            if t_edge <= t <= t_edge + hold_s]
    if len(pre) < 5 or len(post) < 10:
        return None
    pre_mean = float(np.mean([m.roll for _t, m in pre]))
    sign = 1.0 if amp > 0 else -1.0
    target = abs(float(amp))
    ts = np.array([t - t_edge for t, _ in post])
    ys = np.array([sign * (m.roll - pre_mean) for _, m in post])

    cross10 = np.where(ys >= 0.1 * target)[0]
    cross90 = np.where(ys >= 0.9 * target)[0]
    if len(cross10) and len(cross90) and cross90[0] >= cross10[0]:
        rise = float(ts[cross90[0]] - ts[cross10[0]])
    else:
        rise = None

    peak = float(np.max(ys))
    overshoot = float((peak - target) / target * 100) if target > 0 else None

    out_of_band = np.where(np.abs(ys - target) > ss_band_deg)[0]
    if len(out_of_band) == 0:
        settle = float(ts[0])
    elif int(out_of_band[-1]) == len(ts) - 1:
        settle = None
    else:
        settle = float(ts[int(out_of_band[-1]) + 1])

    tail_mask = ts >= (ts[-1] - 0.5)
    ss_off = float(np.mean(ys[tail_mask]) - target) if tail_mask.any() else None

    return {
        't_edge': float(t_edge - samples[0][0]),
        'amp_deg': float(amp),
        'rise_time_s': rise,
        'overshoot_pct': overshoot,
        'settling_time_s': settle,
        'ss_offset_deg': ss_off,
        'peak_deg': peak,
    }


def _compute_health(samples):
    """Per-leg state/error/feeder-mode rollup over the bag. The whole
    point of these fields: surface silent disarms, watchdog hits, and
    feeder-mode mismatches *as the headline* — not buried in a 50 Hz
    timeseries CSV. If anything in this block is non-empty/non-zero,
    leveling can't be expected to work no matter what gains we pick."""
    if not samples:
        return None
    has_state = hasattr(samples[0][1], 'axis_state')
    has_aerr = hasattr(samples[0][1], 'active_errors')
    has_fmode = hasattr(samples[0][1], 'feeder_mode')
    has_yaw = hasattr(samples[0][1], 'yaw')
    if not (has_state or has_aerr or has_fmode or has_yaw):
        return {'note': 'bag pre-dates health-diag fields; re-record to populate'}

    out = {}

    if has_yaw:
        yaws = np.array([float(m.yaw) for _t, m in samples])
        out['yaw_first_deg'] = float(yaws[0])
        out['yaw_last_deg'] = float(yaws[-1])
        out['yaw_range_deg'] = float(np.ptp(yaws))
        out['yaw_drift_rate_deg_per_s'] = (
            float((yaws[-1] - yaws[0]) /
                  max(1e-3, samples[-1][0] - samples[0][0])))

    if has_state:
        # For each leg, count the fraction of ticks where state != 8
        # (CLOSED_LOOP). Anything > 0 means the leg dropped out at
        # least once — and the first_drop_t says when.
        per_leg = []
        for n in range(6):
            states = np.array([int(m.axis_state[n]) for _t, m in samples])
            n_closed = int((states == 8).sum())
            n_idle = int((states == 1).sum())
            n_stale = int((states == 0).sum())
            n_other = len(states) - n_closed - n_idle - n_stale
            # First sample where state was NOT closed-loop (after we'd
            # seen at least one closed-loop sample, so we don't flag
            # the pre-arm part of the bag).
            first_drop = None
            saw_closed = False
            for i, s in enumerate(states):
                if s == 8:
                    saw_closed = True
                elif saw_closed and s != 8:
                    first_drop = float(samples[i][0] - samples[0][0])
                    break
            per_leg.append({
                'leg': n,
                'closed_loop_pct': float(n_closed / len(states) * 100),
                'idle_count': n_idle,
                'stale_count': n_stale,
                'other_count': n_other,
                'first_drop_t_s': first_drop,
            })
        out['per_leg_state'] = per_leg
        any_drop = any(p['first_drop_t_s'] is not None for p in per_leg)
        out['any_leg_dropped_out'] = bool(any_drop)

    if has_aerr:
        # Union of all error bits seen on each leg (excluding 0xFFFFFFFF
        # = "no fresh frame this tick"). Decoded into human names.
        per_leg = []
        for n in range(6):
            seen = 0
            first_seen_t = None
            for t, m in samples:
                code = int(m.active_errors[n]) & 0xFFFFFFFF
                if code == 0xFFFFFFFF or code == 0:
                    continue
                if first_seen_t is None:
                    first_seen_t = float(t - samples[0][0])
                seen |= code
            per_leg.append({
                'leg': n,
                'seen_mask_hex': f'0x{seen:08x}',
                'decoded': _decode_error_bits(seen),
                'first_seen_t_s': first_seen_t,
            })
        out['per_leg_active_errors'] = per_leg
        any_err = any(p['decoded'] for p in per_leg)
        out['any_leg_active_error'] = bool(any_err)

    if has_fmode:
        # Mode names per leg over the bag — a leg that's anything other
        # than 'pos' during a level run is silently ignoring corrections.
        per_leg = []
        _name = {0: 'idle', 1: 'pos', 2: 'vel', 255: 'unknown'}
        for n in range(6):
            modes = [int(m.feeder_mode[n]) for _t, m in samples]
            unique = sorted(set(modes))
            per_leg.append({
                'leg': n,
                'modes_seen': [_name.get(u, str(u)) for u in unique],
                'pos_pct': float(modes.count(1) / len(modes) * 100),
            })
        out['per_leg_feeder_mode'] = per_leg
        out['any_leg_not_in_pos'] = any(
            p['pos_pct'] < 99.0 for p in per_leg)

    return out


def aggregate_steps(trials):
    if not trials:
        return None
    out = {'n_trials': len(trials)}
    for k in ('rise_time_s', 'overshoot_pct', 'settling_time_s', 'ss_offset_deg'):
        vals = [t[k] for t in trials if t.get(k) is not None]
        if vals:
            out[k + '_mean'] = float(np.mean(vals))
            out[k + '_std'] = float(np.std(vals))
            out[k + '_n'] = len(vals)
        else:
            out[k + '_mean'] = None
            out[k + '_std'] = None
            out[k + '_n'] = 0
    return out


def downsample(samples, target_hz=10.0):
    if len(samples) < 2:
        return list(samples)
    t0 = samples[0][0]
    t_end = samples[-1][0]
    duration = max(1e-6, t_end - t0)
    sr = (len(samples) - 1) / duration
    stride = max(1, int(round(sr / target_hz)))
    return samples[::stride]


def write_timeseries_csv(samples, out_path):
    if not samples:
        return
    t0 = samples[0][0]
    # Whether the bag was recorded with an older LevelDiag that didn't
    # include leg_vel / leg_iq_sp. Old bags can still be analyzed; the
    # new columns just stay NaN.
    has_vel = hasattr(samples[0][1], 'leg_vel')
    has_iqsp = hasattr(samples[0][1], 'leg_iq_sp')
    has_yaw = hasattr(samples[0][1], 'yaw')
    has_state = hasattr(samples[0][1], 'axis_state')
    has_aerr = hasattr(samples[0][1], 'active_errors')
    has_fmode = hasattr(samples[0][1], 'feeder_mode')
    headers = [
        't_s', 'roll', 'pitch',
    ]
    if has_yaw:
        headers.append('yaw')
    headers += [
        'err_r', 'err_p', 'err_r_filt', 'err_p_filt',
        'integ_r', 'integ_p',
        'pi_out_r', 'pi_out_p',
        'corr_r', 'corr_p',
        'target_roll', 'target_pitch',
        'clip_flags', 'dt_actual',
    ]
    # Per-leg columns — wide but trivially compressible. Lets us plot
    # commanded-vs-actual position, leg velocity, iq_sp vs iq_measured.
    for n in range(6):
        headers.append(f'mt_{n}')
        headers.append(f'enc_{n}')
        if has_vel:
            headers.append(f'vel_{n}')
        if has_iqsp:
            headers.append(f'iq_sp_{n}')
        headers.append(f'iq_{n}')
        if has_state:
            headers.append(f'state_{n}')
        if has_aerr:
            headers.append(f'aerr_{n}')
        if has_fmode:
            headers.append(f'fmode_{n}')
    with open(out_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(headers)
        for t, m in samples:
            row = [
                f"{t - t0:.4f}",
                f"{m.roll:.5f}", f"{m.pitch:.5f}",
            ]
            if has_yaw:
                row.append(f"{m.yaw:.5f}")
            row += [
                f"{m.err_r:.5f}", f"{m.err_p:.5f}",
                f"{m.err_r_filt:.5f}", f"{m.err_p_filt:.5f}",
                f"{m.integ_r:.5f}", f"{m.integ_p:.5f}",
                f"{m.pi_out_r:.5f}", f"{m.pi_out_p:.5f}",
                f"{m.corr_r:.5f}", f"{m.corr_p:.5f}",
                f"{m.target_roll:.5f}", f"{m.target_pitch:.5f}",
                int(m.clip_flags),
                f"{m.dt_actual:.5f}",
            ]
            for n in range(6):
                row.append(f"{m.motor_targets[n]:.5f}")
                row.append(f"{m.leg_enc[n]:.5f}")
                if has_vel:
                    row.append(f"{m.leg_vel[n]:.5f}")
                if has_iqsp:
                    row.append(f"{m.leg_iq_sp[n]:.5f}")
                row.append(f"{m.leg_iq[n]:.5f}")
                if has_state:
                    row.append(int(m.axis_state[n]))
                if has_aerr:
                    # Hex with 0x prefix so a glance at the CSV is readable
                    row.append(f"0x{int(m.active_errors[n]) & 0xFFFFFFFF:08x}")
                if has_fmode:
                    row.append(int(m.feeder_mode[n]))
            w.writerow(row)


def _read_sidecar(bag_dir):
    side = str(bag_dir) + '_notes.json'
    if os.path.isfile(side):
        try:
            with open(side) as f:
                return json.load(f)
        except Exception:
            return {}
    return {}


def analyze_bag(bag_dir, out_dir, name=None):
    """Single-bag analysis. Writes <out_dir>/<name>_summary.json + _timeseries.csv."""
    bag_dir = Path(bag_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    name = name or bag_dir.name

    samples = read_level_diag(bag_dir)
    if not samples:
        return {'error': 'no /level_diag messages in bag', 'bag': str(bag_dir)}

    t0 = samples[0][0]
    duration = samples[-1][0] - t0

    # Detect step edges. Baseline = up to first edge (or full bag if no steps).
    edges = detect_step_edges(samples)
    baseline_end = (edges[0][0] - t0) - 0.5 if edges else duration
    baseline_end = max(2.0, baseline_end)
    baseline = compute_baseline(samples, end_s=baseline_end)

    trial_results = []
    for t_edge, amp in edges:
        # hold window: until next edge or 5 s
        next_edge_t = None
        for te, _a in edges:
            if te > t_edge:
                next_edge_t = te
                break
        hold = min(5.0, (next_edge_t - t_edge) - 0.1) if next_edge_t else 5.0
        m = step_metrics(samples, t_edge, amp, hold_s=hold)
        if m:
            trial_results.append(m)

    pos_trials = [r for r in trial_results if (r.get('amp_deg') or 0) > 0]
    summary = {
        'bag_path': str(bag_dir),
        'name': name,
        'duration_s': float(duration),
        'n_diag_msgs': len(samples),
        'tick_period_actual_mean_ms': float(
            np.mean([m.dt_actual for _t, m in samples]) * 1000),
        'tick_period_actual_max_ms': float(
            np.max([m.dt_actual for _t, m in samples]) * 1000),
        'health': _compute_health(samples),
        'baseline': baseline,
        'steps_per_trial': trial_results,
        'steps_aggregated': aggregate_steps(pos_trials),
        'sidecar': _read_sidecar(bag_dir),
    }

    summary_path = out_dir / f"{name}_summary.json"
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2, default=lambda o:
            float(o) if hasattr(o, 'item') else None)

    csv_path = out_dir / f"{name}_timeseries.csv"
    write_timeseries_csv(downsample(samples, target_hz=10.0), csv_path)

    return {
        'summary_path': str(summary_path),
        'csv_path': str(csv_path),
        'n_samples': len(samples),
        'n_step_trials': len(trial_results),
    }


def analyze_sweep(sweep_dir, out_base):
    """Walk a sweep_*/ dir's manifest.json, analyze each child bag,
    emit a combined sweep_summary.json with the per-Z metrics rollup."""
    sweep_dir = Path(sweep_dir)
    manifest_path = sweep_dir / 'manifest.json'
    if not manifest_path.exists():
        return {'error': 'no manifest.json', 'sweep_dir': str(sweep_dir)}
    with open(manifest_path) as f:
        manifest = json.load(f)

    out_dir = Path(out_base) / sweep_dir.name
    out_dir.mkdir(parents=True, exist_ok=True)

    per_z = []
    children = []
    for entry in manifest.get('bags', []):
        bag_path = Path(entry.get('bag_dir', ''))
        if not bag_path.exists():
            children.append({'z_mm': entry.get('z_mm'), 'error': 'bag missing'})
            continue
        bag_name = bag_path.name
        result = analyze_bag(bag_path, out_dir, name=bag_name)
        children.append({'z_mm': entry.get('z_mm'), 'bag': bag_name, **result})
        # Pull the per-z rollup row from the just-written summary.
        try:
            with open(result['summary_path']) as f:
                s = json.load(f)
            base = s.get('baseline') or {}
            agg = s.get('steps_aggregated') or {}
            health = s.get('health') or {}
            per_z.append({
                'z_mm': entry.get('z_mm'),
                'bag': bag_name,
                'baseline_rms_roll_deg': base.get('rms_roll_deg'),
                'baseline_rms_pitch_deg': base.get('rms_pitch_deg'),
                'baseline_p2p_roll_deg': base.get('p2p_roll_deg'),
                'fft_peak_freq_hz': base.get('fft_peak_freq_hz'),
                'fft_peak_amp_deg': base.get('fft_peak_amp_deg'),
                'saturation_pct_any': base.get('saturation_pct_any'),
                'rise_time_s': agg.get('rise_time_s_mean'),
                'overshoot_pct': agg.get('overshoot_pct_mean'),
                'settling_time_s': agg.get('settling_time_s_mean'),
                'ss_offset_deg': agg.get('ss_offset_deg_mean'),
                # Health rollup (these jump out at the top of the sweep
                # if any Z had a silent drop / error / mode mismatch).
                'any_leg_dropped_out': health.get('any_leg_dropped_out'),
                'any_leg_active_error': health.get('any_leg_active_error'),
                'any_leg_not_in_pos': health.get('any_leg_not_in_pos'),
                'yaw_range_deg': health.get('yaw_range_deg'),
            })
        except Exception:
            pass

    sweep_summary = {
        'sweep_dir': str(sweep_dir),
        'name': sweep_dir.name,
        'manifest': manifest,
        'children': children,
        'per_z': per_z,
    }
    summary_path = out_dir / f"{sweep_dir.name}_sweep_summary.json"
    with open(summary_path, 'w') as f:
        json.dump(sweep_summary, f, indent=2)

    return {
        'sweep_summary_path': str(summary_path),
        'per_z_rows': len(per_z),
        'children_processed': len(children),
    }


def _default_out_dir():
    """Default to <repo>/tuning_data/ if we can find the repo root, else cwd."""
    here = Path(__file__).resolve()
    # scripts → stewart_bringup → repo
    candidate = here.parent.parent.parent / 'tuning_data'
    if (here.parent.parent.parent / 'jugglebot_interfaces').exists():
        return candidate
    return Path.cwd() / 'tuning_data'


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('input', help='Path to a bag dir OR a sweep_*/ dir')
    p.add_argument('--out', default=None,
                   help='Output base dir (default: <repo>/tuning_data/)')
    args = p.parse_args()

    inp = Path(args.input).expanduser().resolve()
    out_base = Path(args.out).expanduser().resolve() if args.out else _default_out_dir()
    out_base.mkdir(parents=True, exist_ok=True)

    if (inp / 'manifest.json').exists():
        result = analyze_sweep(inp, out_base)
    else:
        # Single bag — write into a per-bag subfolder.
        result = analyze_bag(inp, out_base / inp.name, name=inp.name)

    print(json.dumps(result, indent=2))
    return 0 if 'error' not in result else 1


if __name__ == '__main__':
    sys.exit(main())
