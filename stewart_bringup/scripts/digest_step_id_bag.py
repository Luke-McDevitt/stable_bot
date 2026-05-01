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
  ball_paths_combined.png    — all verification trials' xy paths
                               overlaid on a single platform disc.
                               Time-coloured per trial (light → dark)
                               with start dots and target stars.
                               Quick visual of trial-to-trial
                               consistency, axis asymmetry, and
                               oscillation patterns.
  phase_plane.png            — phase-plane plot per trial: |error| vs
                               velocity-toward-target, scatter coloured
                               by time within the trial. Convergent
                               spiral → origin = stable; closed loop =
                               limit cycle; outward growth = unstable.
  samples.csv                — flat per-sample dump across all
                               verification trials: trial_idx, t,
                               x, y, vx, vy, err, target_x, target_y,
                               start/target marker IDs. Lets the
                               downstream review do any analysis the
                               canned plots don't cover.
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
import csv
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
    # IMU-rpy time series for verification: did the platform actually
    # tilt? Pulled from the per-sample imu_*_deg fields written by
    # auto_tune_node. Falls back to all-zero if the trial bag predates
    # the IMU recording change.
    imu_roll = np.array([s.get('imu_roll_deg', 0.0) for s in samples])
    imu_pitch = np.array([s.get('imu_pitch_deg', 0.0) for s in samples])
    commanded_roll = ol.get('roll_deg', 0.0)
    commanded_pitch = ol.get('pitch_deg', 0.0)
    achieved = ol.get('imu_achieved_tilt_deg', 0.0)
    ratio = ol.get('imu_achievement_ratio', 0.0)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4), dpi=100)
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
    # Middle: commanded vs achieved tilt.
    ax = axes[1]
    ax.plot(ts, imu_roll, '-', color='#3b82f6',
            label='IMU roll [°]')
    ax.plot(ts, imu_pitch, '-', color='#ef4444',
            label='IMU pitch [°]')
    ax.axhline(commanded_roll, color='#3b82f6',
               linestyle=':', linewidth=1.0,
               label=f'commanded roll={commanded_roll:+.2f}°')
    ax.axhline(commanded_pitch, color='#ef4444',
               linestyle=':', linewidth=1.0,
               label=f'commanded pitch={commanded_pitch:+.2f}°')
    ax.axvspan(fit_window[0], fit_window[1], color='#10b981',
               alpha=0.15)
    ax.set_xlabel('t [s] since tilt cmd')
    ax.set_ylabel('platform tilt [°]')
    ax.set_title(
        f'IMU achievement: {achieved:.2f}° '
        f'({ratio*100:.0f}% of commanded)')
    ax.grid(alpha=0.25)
    ax.legend(loc='best', fontsize=7)
    # Right: 1-D projection + parabolic fit.
    ax = axes[2]
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


# ---------- Marker layout (used by the path-overlay plot) ---------

# Hardcoded to match stewart_vision/config/marker_layout.yaml. The
# digest doesn't have an obvious read path to that YAML (it lives in
# a different package), and the marker geometry is a build-time fixed
# property — printing the markers happens once and they're glued onto
# the plate. If the layout ever changes, this list and the YAML need
# to update together.
_MARKER_LAYOUT = [
    (0,    0.0,  120.0),
    (1,   84.853,  84.853),
    (2,  120.0,    0.0),
    (3,   84.853, -84.853),
    (4,    0.0, -120.0),
    (5,  -84.853, -84.853),
    (6, -120.0,    0.0),
    (7,  -84.853,  84.853),
]


def _ball_paths_combined_png(session: dict, out_path: Path) -> None:
    """Overlay all verification trials' xy paths on a single platform
    disc. Each trial's path is a single colour (one slot from the
    viridis cmap per trial); within a trial, marker dots are
    colour-graded by time so early/late points are visible. Start
    (filled circle) and target (star) markers per trial."""
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
    # Platform circle (200 mm radius) + marker ring (r=120 mm)
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(200 * np.cos(theta), 200 * np.sin(theta),
            '-', color='#9ca3af', linewidth=0.8,
            label='platform rim (r=200 mm)')
    ax.plot(120 * np.cos(theta), 120 * np.sin(theta),
            ':', color='#9ca3af', linewidth=0.6,
            label='ArUco ring (r=120 mm)')
    # Marker positions
    for mid, mx, my in _MARKER_LAYOUT:
        ax.plot(mx, my, 's', color='#475569', markersize=7,
                markeredgecolor='#cbd5e1')
        ax.annotate(f'm{mid}', (mx, my), xytext=(7, 7),
                    textcoords='offset points', fontsize=9,
                    color='#94a3b8')
    # Centre + dead zone
    ax.plot(0, 0, 'x', color='#dc2626', markersize=10, markeredgewidth=2)
    ax.add_patch(plt.Circle((0, 0), 30, fill=False,
                            edgecolor='#dc2626', linestyle=':',
                            linewidth=0.8, alpha=0.6))
    # Trial paths, one colour per trial.
    cmap = plt.cm.viridis
    n_trials = len(trials)
    for i, trial in enumerate(trials):
        samples = trial.get('samples', [])
        if not samples:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        color = cmap(i / max(n_trials - 1, 1))
        # Path line
        ax.plot(xs, ys, '-', color=color, linewidth=1.4, alpha=0.85,
                label=(f'Trial {trial.get("trial_idx", i+1)}: '
                       f'm{trial.get("start_marker", "?")} → '
                       f'm{trial.get("target_marker", "?")} '
                       f'({"settled" if trial.get("settled") else "did not settle"})'))
        # Time-graded sample dots — every Nth point so the plot
        # doesn't get overwhelmed at 30 Hz × 25 s
        stride = max(1, len(ts) // 60)
        idx = np.arange(0, len(ts), stride)
        ax.scatter(xs[idx], ys[idx],
                   c=ts[idx],
                   cmap='Greys' if i % 2 == 0 else 'Blues',
                   s=8, alpha=0.5, edgecolors='none')
        # Start (filled circle) + target (star)
        start = trial.get('ball_start_xy', [xs[0], ys[0]])
        target = trial.get('target_xy', [0, 0])
        ax.plot(start[0], start[1], 'o', color=color, markersize=10,
                markeredgecolor='white', markeredgewidth=1.5)
        ax.plot(target[0], target[1], '*', color=color,
                markersize=18, markeredgecolor='white',
                markeredgewidth=1.0)
    ax.set_xlim(-220, 220)
    ax.set_ylim(-220, 220)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25)
    ax.set_xlabel('x [mm]')
    ax.set_ylabel('y [mm]')
    ax.set_title('Verification trials: ball paths overlaid')
    ax.legend(loc='upper right', fontsize=7)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _phase_plane_png(session: dict, out_path: Path) -> None:
    """Phase-plane per trial: |error| vs velocity-toward-target,
    coloured by time within the trial. Stable controller draws an
    inward spiral toward (0, 0); closed loop = limit cycle; outward
    growth = unstable. Settle band drawn at err=15 mm."""
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    n = len(trials)
    fig, axes = plt.subplots(1, n, figsize=(4.5 * n, 4.5), dpi=100)
    if n == 1:
        axes = np.array([axes])
    last_sc = None
    for i, trial in enumerate(trials):
        ax = axes[i]
        samples = trial.get('samples', [])
        if len(samples) < 5:
            ax.set_title(f'Trial {trial.get("trial_idx", i+1)}: '
                         f'too few samples')
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        target = trial.get('target_xy', [0, 0])
        dx = target[0] - xs
        dy = target[1] - ys
        err = np.hypot(dx, dy)
        # Velocity via central differences
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        # Velocity component toward target (positive when ball
        # closing in, negative when receding).
        ux = dx / np.maximum(err, 1e-3)
        uy = dy / np.maximum(err, 1e-3)
        v_toward = vx * ux + vy * uy
        sc = ax.scatter(err, v_toward, c=ts, cmap='viridis',
                        s=10, alpha=0.75, edgecolors='none')
        last_sc = sc
        ax.axhline(0, color='#6b7280', linestyle=':', linewidth=0.8)
        ax.axvline(15, color='#10b981', linestyle=':',
                   linewidth=0.8, label='settle band 15 mm')
        # Mark start and end with annotations
        ax.plot(err[0], v_toward[0], 'o', color='#3b82f6',
                markersize=10, markeredgecolor='white',
                label='start')
        ax.plot(err[-1], v_toward[-1], 's', color='#ef4444',
                markersize=10, markeredgecolor='white',
                label='end')
        ax.set_xlabel('|error| [mm]')
        ax.set_ylabel('v toward target [mm/s]')
        ax.set_title(
            f'Trial {trial.get("trial_idx", i+1)}: '
            f'm{trial.get("start_marker", "?")} → '
            f'm{trial.get("target_marker", "?")}\n'
            f'{"SETTLED" if trial.get("settled") else "DID NOT SETTLE"}')
        ax.grid(alpha=0.25)
        ax.legend(loc='best', fontsize=8)
    if last_sc is not None:
        fig.colorbar(last_sc, ax=axes.tolist(),
                     label='t [s] within trial', shrink=0.8)
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)


def _export_samples_csv(session: dict, out_path: Path) -> None:
    """Flat per-sample dump across all verification trials. One row
    per ball-state sample. Columns let downstream tooling do any
    analysis the canned plots don't cover (pandas / plotly / Excel /
    whatever) without re-parsing the JSON."""
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    fieldnames = [
        'trial_idx', 'start_marker', 'target_marker',
        'target_x_mm', 'target_y_mm',
        'ball_start_x_mm', 'ball_start_y_mm',
        'settled', 'settled_at_s',
        't_in_trial_s', 'x_mm', 'y_mm',
        'vx_mm_s', 'vy_mm_s', 'speed_mm_s',
        'err_mm', 'v_toward_target_mm_s',
    ]
    rows: list[dict] = []
    for trial in trials:
        samples = trial.get('samples', [])
        if len(samples) < 2:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        target = trial.get('target_xy', [0, 0])
        ball_start = trial.get('ball_start_xy', [xs[0], ys[0]])
        # Computed velocity (central differences). At the boundaries
        # numpy.gradient uses one-sided differences automatically.
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        speed = np.hypot(vx, vy)
        dx = target[0] - xs
        dy = target[1] - ys
        err = np.hypot(dx, dy)
        ux = dx / np.maximum(err, 1e-3)
        uy = dy / np.maximum(err, 1e-3)
        v_toward = vx * ux + vy * uy
        for i in range(len(samples)):
            rows.append({
                'trial_idx': int(trial.get('trial_idx', 0)),
                'start_marker': int(trial.get('start_marker', -1)),
                'target_marker': int(trial.get('target_marker', -1)),
                'target_x_mm': float(target[0]),
                'target_y_mm': float(target[1]),
                'ball_start_x_mm': float(ball_start[0]),
                'ball_start_y_mm': float(ball_start[1]),
                'settled': bool(trial.get('settled', False)),
                'settled_at_s': (float(trial['settled_at_s'])
                                 if trial.get('settled_at_s') is not None
                                 else ''),
                't_in_trial_s': float(ts[i]),
                'x_mm': float(xs[i]),
                'y_mm': float(ys[i]),
                'vx_mm_s': float(vx[i]),
                'vy_mm_s': float(vy[i]),
                'speed_mm_s': float(speed[i]),
                'err_mm': float(err[i]),
                'v_toward_target_mm_s': float(v_toward[i]),
            })
    if not rows:
        return
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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
    """Add closed-loop initial-acceleration G_eff estimates +
    cross-check vs open-loop + verification-feedback refinement
    to the summary."""
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
    # Cross-check: does the open-loop G_eff predict the closed-loop
    # initial accelerations? Per-trial: predicted_accel = G_eff · θ_cmd
    # where θ_cmd = min(Kp · D_initial, max_tilt). If the open-loop
    # fit was right, observed accel ≈ predicted across all 4
    # verification trials. Mean ratio of observed/predicted near 1.0
    # = good agreement; far from 1.0 = open-loop fit is unreliable
    # (probably stiction-contaminated; recommendation suspect).
    out['cross_check'] = _cross_check_open_loop_vs_closed_loop(
        out, closed)
    # Verification feedback: detect failure patterns and propose
    # refined gains. Operator can apply these (or not) via the
    # editable inputs in the GUI panel.
    out['verification_analysis'] = _analyze_verification_for_refinement(
        closed, out.get('phases', {}).get('recommendation', {}),
        out.get('current_gains', {}))
    return out


def _cross_check_open_loop_vs_closed_loop(session: dict,
                                          trials: list[dict]
                                          ) -> dict:
    """Per-trial: compute the initial commanded tilt (Kp · D, clipped
    at max_tilt) using the controller gains that were active during
    the verification trials (i.e., session.current_gains, not the
    recommendation). Predict the initial ball acceleration via
    open-loop G_eff. Compare to the observed initial acceleration.

    A high level of agreement (mean ratio ≈ 1.0) means the open-loop
    fit captured the plant correctly; a big disagreement means the
    open-loop fit is suspect — usually stiction-contaminated, since
    closed-loop saturates max_tilt initially and overcomes stiction
    fast, while open-loop's small tilts are stiction-held.

    Returns a dict suitable for embedding in step_id_summary.json.
    Empty when there's no open-loop G_eff or no verification trials.
    """
    if not trials:
        return {}
    rec = session.get('phases', {}).get('recommendation', {}) or {}
    cur = session.get('current_gains', {}) or {}
    g_eff_ol = rec.get('g_eff_used') or rec.get(
        'g_eff_open_loop_mm_s2_per_deg')
    if not g_eff_ol or g_eff_ol < 1.0:
        return {'note': 'no open-loop G_eff to cross-check against'}
    kp_active = float(cur.get('kp', 0.015))
    max_tilt_active = float(cur.get('max_tilt_deg', 2.5))
    rows = []
    ratios = []
    for t in trials:
        accel = _initial_accel_g_eff(t, window_s=0.20)
        if accel is None or accel <= 0:
            continue
        target = t.get('target_xy', [0, 0])
        start = t.get('ball_start_xy', [0, 0])
        d_initial = math.hypot(target[0] - start[0],
                               target[1] - start[1])
        cmd_tilt = min(kp_active * d_initial, max_tilt_active)
        predicted_accel = g_eff_ol * cmd_tilt
        if predicted_accel < 1.0:
            continue
        ratio = accel / predicted_accel
        rows.append({
            'trial_idx': int(t.get('trial_idx', 0)),
            'd_initial_mm': float(d_initial),
            'cmd_tilt_deg': float(cmd_tilt),
            'observed_accel_mm_s2': float(accel),
            'predicted_accel_mm_s2': float(predicted_accel),
            'ratio': float(ratio),
        })
        ratios.append(ratio)
    if not ratios:
        return {'note': 'no usable closed-loop initial-accel data'}
    mean_ratio = float(np.mean(ratios))
    # Agreement quality: ratio in [0.7, 1.4] → good; outside →
    # flag the recommendation. Asymmetric range because saturation
    # truncation tends to underestimate observed (controller can't
    # produce more tilt than commanded), so values <1 are slightly
    # more expected than values >1.
    if 0.7 <= mean_ratio <= 1.4:
        agreement = 'good'
    elif 0.4 <= mean_ratio <= 2.0:
        agreement = 'fair'
    else:
        agreement = 'poor'
    return {
        'g_eff_open_loop': float(g_eff_ol),
        'per_trial': rows,
        'mean_ratio_observed_vs_predicted': mean_ratio,
        'agreement_quality': agreement,
        'note': (
            f'mean observed/predicted accel ratio = {mean_ratio:.2f} '
            f'across {len(rows)} trials; '
            + ('OK — open-loop G_eff is consistent with closed-loop '
               'initial dynamics' if agreement == 'good'
               else 'WARNING — open-loop fit may be stiction-'
                    'contaminated or have wrong tilt direction; '
                    'recommendation should be treated with caution')),
    }


def _analyze_verification_for_refinement(
        trials: list[dict],
        recommendation: dict,
        current_gains: dict) -> dict:
    """Look at verification-trial outcomes and propose refined gains
    when patterns suggest the recommendation needs adjustment.

    Failure modes detected:
      - All 4 trials failed AND median peak velocity > 1500 mm/s →
        vision-noise-dominated Kd → halve Kd
      - All 4 failed AND median peak velocity < 200 mm/s → ball
        wasn't moving → Kp too low (or stiction unaddressed) →
        bump Kp 50%, suggest enabling stiction relief
      - All 4 failed AND mean late-trial radius > 150 mm → orbital
        limit cycle → recommend operator activate the velocity
        sanity-gate AND stiction relief; halve Kd as defensive
      - All 4 settled AND median settling time > 2× predicted →
        too conservative → bump Kp 30%
      - All 4 settled AND any trial overshot > 50% of step distance
        → too aggressive → bump Kd 30%

    Output is analysis-only — operator chooses to apply or not via
    the GUI editable-gain inputs. No automatic re-run.
    """
    if not trials or not recommendation:
        return {'note': 'no trials or no recommendation to refine'}
    n = len(trials)
    n_settled = sum(1 for t in trials if t.get('settled'))
    settling_times = [t['settled_at_s']
                      for t in trials
                      if t.get('settled') and t.get('settled_at_s')]
    # Per-trial robustified peak velocity (95th percentile, not max,
    # to filter glitches) and overshoot.
    peak_vs = []
    overshoots = []
    late_radii = []
    for t in trials:
        samples = t.get('samples', [])
        if len(samples) < 10:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        speeds = np.hypot(vx, vy)
        peak_vs.append(float(np.percentile(speeds, 95)))
        # Overshoot: max of (distance past target along start→target
        # direction). If the ball goes past the target then comes
        # back, we want the max excursion.
        target = t.get('target_xy', [0, 0])
        start = t.get('ball_start_xy', [0, 0])
        d_axis_x = target[0] - start[0]
        d_axis_y = target[1] - start[1]
        d_axis_len = math.hypot(d_axis_x, d_axis_y)
        if d_axis_len > 1.0:
            ux = d_axis_x / d_axis_len
            uy = d_axis_y / d_axis_len
            # Project ball position onto start→target axis;
            # subtract d_axis_len so 0 = at target. Positive
            # values = past target (overshoot).
            proj = ((xs - start[0]) * ux + (ys - start[1]) * uy
                    - d_axis_len)
            overshoot = float(np.max(proj))
            overshoots.append(max(0.0, overshoot))
        # Late-trial radius (last 25% of samples) — if ball is
        # orbiting, this stabilises near the rim.
        late_start = max(1, int(0.75 * len(samples)))
        late_xs = xs[late_start:]
        late_ys = ys[late_start:]
        late_radii.append(float(
            np.mean(np.hypot(late_xs, late_ys))))
    median_peak_v = (float(np.median(peak_vs))
                     if peak_vs else None)
    median_overshoot = (float(np.median(overshoots))
                        if overshoots else None)
    median_late_radius = (float(np.median(late_radii))
                          if late_radii else None)
    median_settle_time = (float(np.median(settling_times))
                          if settling_times else None)
    refined = {
        'kp': recommendation.get('kp'),
        'kd': recommendation.get('kd'),
        'ki': recommendation.get('ki'),
        'max_tilt_deg': recommendation.get('max_tilt_deg'),
    }
    reasons = []
    advisory = []
    if n_settled == 0 and n > 0:
        # All failed. Diagnose by velocity profile.
        if median_peak_v is not None and median_peak_v > 1500.0:
            refined['kd'] = (recommendation.get('kd', 0.03) * 0.5)
            reasons.append(
                f'all 4 trials failed; median peak velocity '
                f'{median_peak_v:.0f} mm/s suggests vision-noise-'
                f'driven Kd instability — Kd halved')
        elif median_peak_v is not None and median_peak_v < 200.0:
            refined['kp'] = (recommendation.get('kp', 0.015) * 1.5)
            reasons.append(
                f'all 4 trials failed; median peak velocity '
                f'{median_peak_v:.0f} mm/s is below typical drive '
                f'levels — Kp +50% to overcome friction floor')
            advisory.append(
                'consider activating stiction relief via the '
                'stiction_break_s / stiction_v_threshold_mm_s gains')
        if (median_late_radius is not None
                and median_late_radius > 150.0):
            advisory.append(
                f'mean late-trial radius {median_late_radius:.0f} mm '
                f'(rim is 200) suggests orbital limit cycle. The '
                f'velocity-sanity-gate (stewart_control_node, '
                f'BT_MAX_BALL_VEL_MM_S=800) and stiction-relief '
                f'PID modification are the structural fixes for '
                f'this; lower Kd as a stopgap')
            if 'Kd halved' not in ' '.join(reasons):
                refined['kd'] = (recommendation.get('kd', 0.03) * 0.7)
                reasons.append(
                    'orbital limit cycle suspected — Kd reduced 30% '
                    'as defensive measure')
    elif n_settled == n and median_settle_time is not None:
        # All settled — fine-tune.
        # Predicted settling time from ωn, ζ:
        # for ζ=0.7, t_2pct ≈ 4 / (ζ · ωn) ≈ 5.7 / ωn
        omega_n = recommendation.get('omega_n_rad_s', 4.0)
        if omega_n > 0:
            predicted_settle = 5.7 / omega_n
            if median_settle_time > 2.0 * predicted_settle:
                refined['kp'] = (
                    recommendation.get('kp', 0.015) * 1.3)
                reasons.append(
                    f'all settled but median settling time '
                    f'{median_settle_time:.1f} s exceeds predicted '
                    f'{predicted_settle:.1f} s by 2×; recommendation '
                    f'too conservative — Kp +30%')
        if (median_overshoot is not None
                and median_overshoot > 50.0):
            refined['kd'] = (recommendation.get('kd', 0.03) * 1.3)
            reasons.append(
                f'all settled but median overshoot '
                f'{median_overshoot:.0f} mm — too aggressive; '
                f'Kd +30% for more damping')
    else:
        reasons.append(
            f'{n_settled}/{n} trials settled — partial success, no '
            f'automatic refinement (insufficient signal). Inspect '
            f'phase_plane.png and ball_paths_combined.png to '
            f'decide manually.')
    refined_changed = any(
        refined.get(k) != recommendation.get(k)
        for k in ('kp', 'kd', 'ki', 'max_tilt_deg'))
    return {
        'n_trials': n,
        'n_settled': n_settled,
        'median_peak_velocity_mm_s': median_peak_v,
        'median_settling_time_s': median_settle_time,
        'median_overshoot_mm': median_overshoot,
        'median_late_radius_mm': median_late_radius,
        'refined_gains': refined if refined_changed else None,
        'refinement_reasons': reasons,
        'advisory': advisory,
    }


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
    cross_check = session.get('cross_check', {})
    verif_analysis = session.get('verification_analysis', {})
    aggregate = session.get('phases', {}).get('open_loop_aggregate',
                                              {})
    summary = {
        'current_gains': cur,
        'recommended_gains': {
            'kp': rec.get('kp'),
            'kd': rec.get('kd'),
            'ki': rec.get('ki'),
            'max_tilt_deg': rec.get('max_tilt_deg'),
        },
        # If verification suggested a refinement, surface it here
        # alongside the original analytic recommendation. Operator
        # picks which to apply via the GUI editable inputs.
        'refined_gains': (verif_analysis.get('refined_gains')
                          if verif_analysis else None),
        'refinement_reasons': (verif_analysis.get('refinement_reasons')
                               if verif_analysis else None),
        'advisory': (verif_analysis.get('advisory')
                     if verif_analysis else None),
        'omega_n_rad_s': rec.get('omega_n_rad_s'),
        'zeta': rec.get('zeta'),
        'g_eff_open_loop_mm_s2_per_deg': rec.get('g_eff_used'),
        'g_eff_open_loop_std_mm_s2_per_deg':
            aggregate.get('g_eff_std_mm_s2_per_deg'),
        'g_eff_open_loop_cv': aggregate.get('g_eff_cv'),
        'n_open_loop_replicates': aggregate.get('n_replicates'),
        'td_observed_s': rec.get('td_observed_s'),
        'closed_loop_initial_accel_mm_s2': closed_g,
        'cross_check_open_loop_vs_closed_loop': cross_check,
        'verification_summary': {
            'n_trials': verif_analysis.get('n_trials'),
            'n_settled': verif_analysis.get('n_settled'),
            'median_settling_time_s':
                verif_analysis.get('median_settling_time_s'),
            'median_peak_velocity_mm_s':
                verif_analysis.get('median_peak_velocity_mm_s'),
            'median_overshoot_mm':
                verif_analysis.get('median_overshoot_mm'),
            'median_late_radius_mm':
                verif_analysis.get('median_late_radius_mm'),
        } if verif_analysis else {},
        'recommendation_valid': rec.get('valid', False),
        'note': rec.get('note', ''),
    }
    with open(out_path, 'w') as f:
        json.dump(summary, f, indent=2,
                  default=lambda o: float(o) if isinstance(o,
                                                            np.floating)
                  else None)


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
    _ball_paths_combined_png(session, d / 'ball_paths_combined.png')
    _phase_plane_png(session, d / 'phase_plane.png')
    _export_samples_csv(session, d / 'samples.csv')
    _recommended_gains_png(session, d / 'recommended_gains.png')
    with open(d / 'step_id_summary.json', 'w') as f:
        json.dump(refined, f, indent=2,
                  default=lambda o: float(o) if isinstance(o, np.floating)
                  else None)
    _write_recommendation(refined, d / 'step_id_recommendation.json')
    print(f'digest written → {d}')
    print('  plant_gain_fit.png')
    print('  step_response_per_trial.png')
    print('  ball_paths_combined.png')
    print('  phase_plane.png')
    print('  samples.csv')
    print('  recommended_gains.png')
    print('  step_id_summary.json')
    print('  step_id_recommendation.json')


if __name__ == '__main__':
    main()
