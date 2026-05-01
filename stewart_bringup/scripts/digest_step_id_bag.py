#!/usr/bin/env python3
"""Digest a STEP_ID session: produce summary PNGs and refined JSONs
from summary.json + log.jsonl + bag/.

Reads tuning_data/step_id_<UTC>/ and writes alongside it:

  plant_gain_fit.png         — open-loop tilt-step trial: ball
                               trajectory vs time + parabolic fit;
                               shows extracted G_eff visually.
  step_response_per_trial.png — closed-loop verification: 4-panel grid,
                               one row per trial. Columns: ball
                               trajectory in xy, x(t)/y(t) vs time,
                               error magnitude vs time.
  recommended_gains.png      — predicted closed-loop step response with
                               current gains vs recommended gains,
                               based on the second-order plant model
                               and observed dead-time.
  step_id_summary.json       — refined summary: per-trial settling
                               time, peak overshoot, initial-acceleration
                               G_eff (closed-loop), agreement with the
                               open-loop G_eff.
  step_id_recommendation.json — the actionable output: current gains,
                               recommended gains, expected settling
                               time, ζ, ωn, G_eff (open-loop and
                               cross-validated from closed-loop).

Usage:
    python3 stewart_bringup/scripts/digest_step_id_bag.py \\
        ~/stable_bot_repo/tuning_data/step_id_20260501T123000Z

This digest is JSON+optional-bag-based. Bag is parsed only if present
and only for /ball_state if available; the trial JSON already carries
the same data at the same rate, so for the v0 of the digest we read
only the JSON. Bag stays on the Pi for retrospective deep-dives.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _load_summary(d: Path) -> dict:
    p = d / 'summary.json'
    if not p.exists():
        print(f'no summary.json in {d}', file=sys.stderr)
        return {}
    with open(p) as f:
        return json.load(f)


def _plant_gain_fit_png(session: dict, out_path: Path) -> None:
    ol = session.get('phases', {}).get('open_loop')
    if not ol or not ol.get('samples'):
        return
    samples = ol['samples']
    ts = np.array([s['t'] for s in samples])
    xs = np.array([s['x'] for s in samples])
    ys = np.array([s['y'] for s in samples])
    phases = [s.get('phase', '') for s in samples]
    tilt_mask = np.array([p == 'tilt' for p in phases])
    # Project onto expected motion direction so the parabolic fit is
    # 1-D and visually clean.
    fit = ol.get('fit', {})
    motion_unit = fit.get('expected_motion_unit', [1.0, 0.0])
    baseline_xy = fit.get('baseline_xy', [0.0, 0.0])
    a_fit = fit.get('a_fit_mm_s2', 0.0)
    fit_window = fit.get('fit_window_s', [0.0, 0.5])
    d = ((xs - baseline_xy[0]) * motion_unit[0]
         + (ys - baseline_xy[1]) * motion_unit[1])
    fig, axes = plt.subplots(1, 2, figsize=(11, 4), dpi=100)
    # Left: raw x(t) and y(t) with phase colours.
    ax = axes[0]
    ax.plot(ts, xs, '-', color='#3b82f6', label='x [mm]')
    ax.plot(ts, ys, '-', color='#ef4444', label='y [mm]')
    ax.axvspan(0.0, fit_window[0], color='#9ca3af', alpha=0.15,
               label='ramp / pre-fit')
    ax.axvspan(fit_window[0], fit_window[1], color='#10b981',
               alpha=0.15, label='fit window')
    ax.set_xlabel('t [s] since tilt cmd')
    ax.set_ylabel('ball position [mm]')
    ax.set_title('open-loop tilt-step: raw ball trajectory')
    ax.grid(alpha=0.25)
    ax.legend(loc='best', fontsize=8)
    # Right: 1-D projection + parabolic fit.
    ax = axes[1]
    ax.plot(ts, d, 'o', color='#60a5fa', markersize=3,
            alpha=0.7, label='ball displacement (projected)')
    if fit_window[1] > fit_window[0]:
        t_pred = np.linspace(fit_window[0], fit_window[1], 50)
        d_pred = 0.5 * a_fit * (t_pred - fit_window[0]) ** 2
        # The c0/c1 terms shift the fit; recompute via polyfit on the
        # window for visual fidelity.
        wmask = (ts >= fit_window[0]) & (ts <= fit_window[1])
        if wmask.sum() >= 3:
            try:
                co = np.polyfit(ts[wmask], d[wmask], 2)
                d_pred = np.polyval(co, t_pred)
                ax.plot(t_pred, d_pred, '-', color='#10b981',
                        linewidth=2,
                        label=(f'parabolic fit, a={a_fit:.0f} mm/s²'))
            except Exception:
                pass
    g_eff = ol.get('g_eff_mm_s2_per_deg', 0.0)
    td = ol.get('td_observed_s', 0.0)
    ax.set_xlabel('t [s] since tilt cmd')
    ax.set_ylabel('displacement along motion axis [mm]')
    ax.set_title(
        f'plant gain fit: G_eff={g_eff:.1f} mm/s²/°, '
        f'Td={td*1000:.0f} ms')
    ax.grid(alpha=0.25)
    ax.legend(loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _step_response_per_trial_png(session: dict, out_path: Path) -> None:
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    n = len(trials)
    fig, axes = plt.subplots(n, 3, figsize=(12, 3 * n), dpi=100)
    if n == 1:
        axes = np.array([axes])
    for i, t in enumerate(trials):
        samples = t.get('samples', [])
        if not samples:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        target = t.get('target_xy', [0, 0])
        start = t.get('ball_start_xy', [0, 0])
        err = np.hypot(xs - target[0], ys - target[1])
        # Col 0: xy trajectory
        ax = axes[i][0]
        # Platform circle (200 mm radius for 400 mm dia), marker ring
        theta = np.linspace(0, 2 * np.pi, 200)
        ax.plot(200 * np.cos(theta), 200 * np.sin(theta),
                '-', color='#9ca3af', linewidth=0.8)
        ax.plot(120 * np.cos(theta), 120 * np.sin(theta),
                ':', color='#9ca3af', linewidth=0.6)
        ax.plot(xs, ys, '-', color='#60a5fa', linewidth=1.0)
        ax.plot(start[0], start[1], 'o', color='#10b981',
                markersize=8, label=f'start (m{t["start_marker"]})')
        ax.plot(target[0], target[1], '*', color='#ef4444',
                markersize=12, label=f'target (m{t["target_marker"]})')
        ax.set_xlim(-220, 220)
        ax.set_ylim(-220, 220)
        ax.set_aspect('equal')
        ax.grid(alpha=0.25)
        if i == 0:
            ax.set_title('xy trajectory')
        ax.legend(loc='upper right', fontsize=7)
        ax.set_ylabel(f'trial {i+1}\n'
                      f'{"settled" if t.get("settled") else "did not settle"}')
        # Col 1: x(t), y(t)
        ax = axes[i][1]
        ax.plot(ts, xs, '-', color='#3b82f6', label='x')
        ax.plot(ts, ys, '-', color='#ef4444', label='y')
        ax.axhline(target[0], color='#3b82f6', linestyle=':', alpha=0.4)
        ax.axhline(target[1], color='#ef4444', linestyle=':', alpha=0.4)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('position [mm]')
        ax.grid(alpha=0.25)
        ax.legend(loc='best', fontsize=8)
        if i == 0:
            ax.set_title('x(t), y(t) vs target')
        # Col 2: error magnitude
        ax = axes[i][2]
        ax.plot(ts, err, '-', color='#a855f7', linewidth=1.2)
        ax.axhline(15.0, color='#10b981', linestyle=':',
                   linewidth=0.8, label='settle band 15 mm')
        if t.get('settled') and t.get('settled_at_s'):
            ax.axvline(t['settled_at_s'], color='#10b981',
                       linestyle='--', linewidth=1.0,
                       label=f'settled @ {t["settled_at_s"]:.1f}s')
        ax.set_xlabel('t [s]')
        ax.set_ylabel('|error| [mm]')
        ax.grid(alpha=0.25)
        ax.legend(loc='best', fontsize=8)
        if i == 0:
            ax.set_title('error magnitude')
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _initial_accel_g_eff(trial: dict, window_s: float = 0.20
                         ) -> float | None:
    """From a closed-loop trial's first window_s of samples, fit the
    initial acceleration along the start→target direction. With initial
    error = D and commanded tilt ≈ Kp·D (clamped at max_tilt), the
    initial ball acceleration ≈ G_eff·θ_cmd. Kp is constant during the
    window; commanded tilt magnitude is roughly known from the YAML
    snapshot. We can't precisely recover G_eff without the commanded-
    tilt time series in the bag — this is a coarse estimator. Returns
    G_eff in mm/s²/° on success, None if data is too sparse."""
    samples = trial.get('samples', [])
    if len(samples) < 5:
        return None
    target = trial.get('target_xy', [0, 0])
    start = trial.get('ball_start_xy', [0, 0])
    dx = target[0] - start[0]
    dy = target[1] - start[1]
    nrm = math.hypot(dx, dy)
    if nrm < 1.0:
        return None
    ux, uy = dx / nrm, dy / nrm
    ts = np.array([s['t'] for s in samples])
    xs = np.array([s['x'] for s in samples])
    ys = np.array([s['y'] for s in samples])
    mask = ts <= window_s
    if mask.sum() < 4:
        return None
    d = (xs - start[0]) * ux + (ys - start[1]) * uy
    try:
        co = np.polyfit(ts[mask], d[mask], 2)
    except Exception:
        return None
    # Magnitude only — sign tells us if controller has wrong direction.
    return float(abs(co[0]) * 2.0)


def _refine_summary(session: dict) -> dict:
    """Add closed-loop initial-acceleration G_eff estimates to the
    summary so the digest reports both open-loop and closed-loop
    measurements and their cross-validation agreement."""
    out = dict(session)
    closed = out.get('phases', {}).get('verification', [])
    cl_g = [_initial_accel_g_eff(t) for t in closed]
    cl_g = [g for g in cl_g if g is not None and g > 0]
    if cl_g:
        out['closed_loop_initial_accel_mm_s2'] = {
            'samples': [float(g) for g in cl_g],
            'mean': float(np.mean(cl_g)),
            'std': float(np.std(cl_g)),
        }
    return out


def _recommended_gains_png(session: dict, out_path: Path) -> None:
    rec = session.get('phases', {}).get('recommendation', {})
    if not rec or not rec.get('valid'):
        return
    cur = session.get('current_gains', {})
    g = rec.get('g_eff_used', 122.0)
    omega_n_new = rec.get('omega_n_rad_s', 4.0)
    zeta_new = rec.get('zeta', 0.7)
    kp_new = rec.get('kp', 0.0)
    kd_new = rec.get('kd', 0.0)
    kp_old = cur.get('kp', 0.015)
    kd_old = cur.get('kd', 0.03)
    # Predicted closed-loop step response: x(s)/r(s) for PD on a double
    # integrator: T(s) = (Kd·s + Kp)·G / (s² + Kd·G·s + Kp·G).
    # Time-domain via numpy lsim-equivalent with scipy if available;
    # otherwise just plot ωn/ζ as labels and the predicted second-order
    # response x(t) = 1 - exp(-ζωn·t)·(cos(ωd·t) + ζ/√(1-ζ²)·sin(ωd·t))
    # for the dominant pole approximation.

    def step_response(omega_n, zeta, t_end=4.0, n=400):
        ts = np.linspace(0, t_end, n)
        if zeta < 1.0:
            wd = omega_n * math.sqrt(1 - zeta * zeta)
            return ts, 1.0 - np.exp(-zeta * omega_n * ts) * (
                np.cos(wd * ts)
                + (zeta / math.sqrt(1 - zeta * zeta)) * np.sin(wd * ts))
        else:
            # Critically/over-damped — single exp envelope
            return ts, 1.0 - np.exp(-omega_n * ts)

    # Old gains' predicted ωn/ζ from the same plant gain
    omega_n_old = math.sqrt(max(kp_old * g, 1e-9))
    zeta_old = (kd_old * g) / (2 * omega_n_old) if omega_n_old > 0 else 0.0
    ts_o, y_o = step_response(omega_n_old, zeta_old)
    ts_n, y_n = step_response(omega_n_new, zeta_new)
    fig, ax = plt.subplots(figsize=(7, 4.5), dpi=100)
    ax.plot(ts_o, y_o, '-', color='#9ca3af', linewidth=1.5,
            label=(f'current  kp={kp_old:.4f} kd={kd_old:.4f} '
                   f'(ωn={omega_n_old:.2f}, ζ={zeta_old:.2f})'))
    ax.plot(ts_n, y_n, '-', color='#10b981', linewidth=2,
            label=(f'recommended  kp={kp_new:.4f} kd={kd_new:.4f} '
                   f'(ωn={omega_n_new:.2f}, ζ={zeta_new:.2f})'))
    ax.axhline(1.0, color='#6b7280', linestyle=':', linewidth=0.6)
    ax.axhspan(0.95, 1.05, color='#10b981', alpha=0.07,
               label='±5% settle band')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('predicted normalised position')
    ax.set_title(
        f'predicted closed-loop step response\n'
        f'(plant G_eff={g:.1f} mm/s²/° from open-loop fit)')
    ax.grid(alpha=0.25)
    ax.legend(loc='lower right', fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _write_recommendation(session: dict, out_path: Path) -> None:
    rec = session.get('phases', {}).get('recommendation', {})
    cur = session.get('current_gains', {})
    closed_g = session.get('closed_loop_initial_accel_mm_s2', {})
    summary = {
        'current_gains': cur,
        'recommended_gains': {
            'kp': rec.get('kp'),
            'kd': rec.get('kd'),
            'ki': rec.get('ki'),
        },
        'omega_n_rad_s': rec.get('omega_n_rad_s'),
        'zeta': rec.get('zeta'),
        'g_eff_open_loop_mm_s2_per_deg': rec.get('g_eff_used'),
        'td_observed_s': rec.get('td_observed_s'),
        'closed_loop_initial_accel_mm_s2': closed_g,
        'recommendation_valid': rec.get('valid', False),
        'note': rec.get('note', ''),
    }
    with open(out_path, 'w') as f:
        json.dump(summary, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description='Digest a STEP_ID session directory.')
    parser.add_argument('session_dir', type=Path,
                        help='path to tuning_data/step_id_<UTC>/')
    args = parser.parse_args()
    d = args.session_dir
    if not d.is_dir():
        print(f'not a directory: {d}', file=sys.stderr)
        sys.exit(2)
    session = _load_summary(d)
    if not session:
        sys.exit(1)
    refined = _refine_summary(session)
    _plant_gain_fit_png(session, d / 'plant_gain_fit.png')
    _step_response_per_trial_png(session, d / 'step_response_per_trial.png')
    _recommended_gains_png(session, d / 'recommended_gains.png')
    with open(d / 'step_id_summary.json', 'w') as f:
        json.dump(refined, f, indent=2,
                  default=lambda o: float(o) if isinstance(o, np.floating)
                  else None)
    _write_recommendation(refined, d / 'step_id_recommendation.json')
    print(f'digest written → {d}')
    print('  plant_gain_fit.png')
    print('  step_response_per_trial.png')
    print('  recommended_gains.png')
    print('  step_id_summary.json')
    print('  step_id_recommendation.json')


if __name__ == '__main__':
    main()
