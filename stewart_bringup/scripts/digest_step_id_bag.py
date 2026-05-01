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
  tilt_timeseries.png        — per verification trial: commanded tilt
                               (pitch + roll) vs IMU-achieved tilt
                               over time, plus a phase-mix bar
                               (PID/STICTION_BREAK/SETTLE share of
                               trial time). Shows max_tilt saturation
                               events, stiction-relief activations,
                               inner-loop tracking quality.
  speed_histogram.png        — per verification trial: histogram of
                               ball-speed values on log scale, with
                               the 800 mm/s velocity-gate threshold
                               marked. Visualises how often vision
                               noise pushes velocity past the
                               controller's clamp.
  samples.csv                — flat per-sample dump covering BOTH
                               open-loop replicates AND verification
                               trials. `phase` column distinguishes
                               (open_loop_repN | verification_trial_N).
                               Includes IMU + commanded-tilt + phase-
                               code per sample for verification rows
                               so any analysis the canned plots don't
                               cover can be done downstream.
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


def _vision_backend_tag(session: dict) -> str:
    """One-line summary of the vision backend used in this session,
    suitable for embedding in plot titles. Pulls from the session-
    level snapshot first, falls back to the first replicate's
    snapshot, and finally to '(unknown)' when the session predates
    the snapshot capture."""
    vb = session.get('vision_backend_at_start') or {}
    if not vb.get('v0_backend'):
        # Fall back to first replicate's snapshot for older
        # sessions that didn't have a session-level field.
        reps = (session.get('phases', {})
                .get('open_loop_replicates') or [])
        if not reps:
            ol = session.get('phases', {}).get('open_loop')
            reps = [ol] if ol else []
        if reps:
            vb = reps[0].get('vision_backend_at_start') or {}
    bk = vb.get('v0_backend')
    if not bk:
        return '(vision backend: unknown)'
    extra_parts = []
    if bk == 'v1_yolo':
        arch = vb.get('v1_arch')
        if arch:
            extra_parts.append(arch)
    rgb_fps = vb.get('rgb_fps')
    if rgb_fps:
        extra_parts.append(f'{rgb_fps}fps')
    extra = ' / '.join(extra_parts)
    return f'vision: {bk}{" / " + extra if extra else ""}'


def _plant_gain_fit_png(session: dict, out_path: Path) -> None:
    """Open-loop plant ID visual. Multi-replicate when the session
    ran ≥2 open-loop trials (default n=3): all replicates overlaid
    in each panel with one colour per replicate, plus aggregate
    label showing mean ± std and CV. Falls back to single-replicate
    layout when phases.open_loop_replicates isn't present (older
    sessions or n=1 runs)."""
    phases = session.get('phases', {})
    replicates = phases.get('open_loop_replicates') or []
    aggregate = phases.get('open_loop_aggregate') or {}
    # Back-compat: single-replicate sessions only have phases.open_loop
    # populated. Treat as a 1-element replicate list.
    if not replicates:
        ol = phases.get('open_loop')
        if not ol or not ol.get('samples'):
            return
        replicates = [ol]
    n_reps = len(replicates)
    cmap = plt.cm.viridis
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), dpi=100)
    # We'll plot per-replicate traces in each panel; one colour per
    # replicate so they're visually distinguishable. Solid line for
    # replicate 0 (which is also phases.open_loop for back-compat),
    # dashed for the rest.
    g_effs = []
    fit_windows_for_shading = []
    for i, ol in enumerate(replicates):
        samples = ol.get('samples') or []
        if not samples:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        phases_per_sample = [s.get('phase', '') for s in samples]
        # Project ball displacement onto motion direction (1-D fit).
        fit = ol.get('fit', {})
        motion_unit = fit.get('expected_motion_unit', [1.0, 0.0])
        baseline_xy = fit.get('baseline_xy', [0.0, 0.0])
        a_fit = fit.get('a_fit_mm_s2', 0.0)
        fit_window = fit.get('fit_window_s', [0.0, 0.5])
        fit_windows_for_shading.append(fit_window)
        d = ((xs - baseline_xy[0]) * motion_unit[0]
             + (ys - baseline_xy[1]) * motion_unit[1])
        imu_roll = np.array([s.get('imu_roll_deg', 0.0)
                             for s in samples])
        imu_pitch = np.array([s.get('imu_pitch_deg', 0.0)
                              for s in samples])
        commanded_roll = ol.get('roll_deg', 0.0)
        commanded_pitch = ol.get('pitch_deg', 0.0)
        g_eff = ol.get('g_eff_mm_s2_per_deg', 0.0)
        g_effs.append(g_eff)
        ratio = ol.get('imu_achievement_ratio', 0.0)
        color = cmap(i / max(n_reps - 1, 1))
        rep_label = (f'rep {i+1} (G={g_eff:.0f}, '
                     f'IMU={ratio*100:.0f}%)')
        # Left: raw ball position projected onto motion axis (1-D
        # signal). Replicates may differ in starting position so we
        # plot displacement-from-baseline rather than absolute coords.
        axes[0].plot(ts, d, '-', color=color, linewidth=1.4,
                     alpha=0.85, label=rep_label)
        # Middle: IMU achieved tilt (dominant axis only — whichever
        # was commanded). Use signed value so under/over shoot is
        # visible.
        if abs(commanded_pitch) >= abs(commanded_roll):
            axes[1].plot(ts, imu_pitch, '-', color=color,
                         linewidth=1.4, alpha=0.85,
                         label=f'rep {i+1} pitch')
            axes[1].axhline(commanded_pitch, color=color,
                            linestyle=':', linewidth=0.8, alpha=0.5)
        else:
            axes[1].plot(ts, imu_roll, '-', color=color,
                         linewidth=1.4, alpha=0.85,
                         label=f'rep {i+1} roll')
            axes[1].axhline(commanded_roll, color=color,
                            linestyle=':', linewidth=0.8, alpha=0.5)
        # Right: parabolic fit on the post-motion-onset window.
        if fit_window[1] > fit_window[0]:
            wmask = (ts >= fit_window[0]) & (ts <= fit_window[1])
            if wmask.sum() >= 3:
                try:
                    co = np.polyfit(ts[wmask], d[wmask], 2)
                    t_pred = np.linspace(fit_window[0],
                                         fit_window[1], 50)
                    d_pred = np.polyval(co, t_pred)
                    axes[2].plot(t_pred, d_pred, '-', color=color,
                                 linewidth=2,
                                 label=(f'rep {i+1} fit, '
                                        f'a={a_fit:.0f} mm/s²'))
                except Exception:
                    pass
        # Per-replicate raw-data scatter on the right panel — small
        # dots so we can see what's being fit.
        axes[2].plot(ts, d, '.', color=color, markersize=2,
                     alpha=0.4)
    # Shade the union of all fit windows on each panel — operator
    # gets a sense of where the fits are operating.
    if fit_windows_for_shading:
        wmin = min(w[0] for w in fit_windows_for_shading)
        wmax = max(w[1] for w in fit_windows_for_shading)
        for ax in axes:
            ax.axvspan(wmin, wmax, color='#10b981', alpha=0.07,
                       zorder=0)
    # Panel titles & axis labels.
    if aggregate:
        n = aggregate.get('n_replicates', n_reps)
        gm = aggregate.get('g_eff_mean_mm_s2_per_deg', 0.0)
        gs = aggregate.get('g_eff_std_mm_s2_per_deg', 0.0)
        cv = aggregate.get('g_eff_cv', 0.0)
        title_main = (f'open-loop plant ID, n={n}: '
                      f'G_eff = {gm:.1f} ± {gs:.1f} mm/s²/° '
                      f'(CV = {cv*100:.0f}%)')
    else:
        gm = g_effs[0] if g_effs else 0.0
        title_main = (f'open-loop plant ID '
                      f'(single trial): G_eff = {gm:.1f} mm/s²/°')
    axes[0].set_xlabel('t [s] since tilt cmd')
    axes[0].set_ylabel('displacement on motion axis [mm]')
    axes[0].set_title('ball trajectory (1-D)')
    axes[0].grid(alpha=0.25)
    axes[0].legend(loc='best', fontsize=7)
    axes[1].set_xlabel('t [s] since tilt cmd')
    axes[1].set_ylabel('platform tilt [°]')
    axes[1].set_title('IMU achievement (dominant axis)')
    axes[1].grid(alpha=0.25)
    axes[1].legend(loc='best', fontsize=7)
    axes[2].set_xlabel('t [s] since tilt cmd')
    axes[2].set_ylabel('displacement [mm]')
    axes[2].set_title('parabolic fits')
    axes[2].grid(alpha=0.25)
    axes[2].legend(loc='best', fontsize=7)
    # Append vision-backend tag so the operator can compare apples-
    # to-apples across sessions (cv2 vs v1_yolo will produce very
    # different open-loop fits because of motion-blur robustness).
    fig.suptitle(f'{title_main}   ·   {_vision_backend_tag(session)}',
                 fontsize=11)
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
    ax.set_title(
        'Verification trials: ball paths overlaid'
        f'   ·   {_vision_backend_tag(session)}')
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
    """Flat per-sample dump covering BOTH open-loop replicates and
    verification trials. One row per ball-state sample. The `phase`
    column distinguishes:
      - 'open_loop_rep0', 'open_loop_rep1', ... (one per replicate)
      - 'verification_trial_1', ... 'verification_trial_4'
    Columns let downstream tooling (pandas / plotly / Excel /
    whatever) do any analysis the canned plots don't cover without
    re-parsing the JSON.
    """
    fieldnames = [
        'phase',                     # open_loop_repN | verification_trial_N
        'trial_idx',                 # 0 for open-loop, 1-4 for verification
        # Common geometry
        'start_marker', 'target_marker',
        'target_x_mm', 'target_y_mm',
        'ball_start_x_mm', 'ball_start_y_mm',
        # Verification-only metadata (blank for open-loop)
        'settled', 'settled_at_s',
        # Time series
        't_in_trial_s', 'x_mm', 'y_mm',
        'vx_mm_s', 'vy_mm_s', 'speed_mm_s',
        'err_mm', 'v_toward_target_mm_s',
        # Tilt instrumentation (NEW). For open-loop: cmd_pitch/roll
        # are constant (the trial's commanded values); IMU is the
        # platform's measured response. For verification: cmd_pitch/
        # roll are the BALL_TRACK loop's per-tick output. Both
        # populated when /ball_track/diagnostic was running (i.e.
        # bag was recorded after 2026-05-01 commit 2eb6235).
        'imu_pitch_deg', 'imu_roll_deg',
        'cmd_pitch_deg', 'cmd_roll_deg',
        'phase_code',                # 0 settle, 1 accel, 2 coast,
                                     # 3 brake, 4 stiction_break,
                                     # 5 pid, -1 stale, -2 no diag
    ]
    rows: list[dict] = []
    # ---- Open-loop replicates (NEW) -----------------------------
    # Each replicate is its own "trial" for the CSV; phase column
    # carries the replicate number so they're distinguishable.
    replicates = (session.get('phases', {})
                  .get('open_loop_replicates') or [])
    if not replicates:
        # Back-compat: single open-loop trial sessions.
        ol = session.get('phases', {}).get('open_loop')
        if ol:
            replicates = [ol]
    for rep_idx, ol in enumerate(replicates):
        samples = ol.get('samples') or []
        if len(samples) < 2:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        ball_start = ol.get('ball_start_xy', [xs[0], ys[0]])
        cmd_pitch_const = ol.get('pitch_deg', 0.0)
        cmd_roll_const = ol.get('roll_deg', 0.0)
        sm = ol.get('start_marker', -1)
        # Computed velocity for open-loop too.
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        speed = np.hypot(vx, vy)
        for i, sample in enumerate(samples):
            rows.append({
                'phase': f'open_loop_rep{rep_idx}',
                'trial_idx': 0,
                'start_marker': int(sm),
                'target_marker': -1,
                'target_x_mm': '', 'target_y_mm': '',
                'ball_start_x_mm': float(ball_start[0]),
                'ball_start_y_mm': float(ball_start[1]),
                'settled': '', 'settled_at_s': '',
                't_in_trial_s': float(ts[i]),
                'x_mm': float(xs[i]), 'y_mm': float(ys[i]),
                'vx_mm_s': float(vx[i]), 'vy_mm_s': float(vy[i]),
                'speed_mm_s': float(speed[i]),
                'err_mm': '',
                'v_toward_target_mm_s': '',
                'imu_pitch_deg':
                    float(sample.get('imu_pitch_deg', 0.0)),
                'imu_roll_deg':
                    float(sample.get('imu_roll_deg', 0.0)),
                'cmd_pitch_deg': float(cmd_pitch_const),
                'cmd_roll_deg': float(cmd_roll_const),
                'phase_code': '',  # no phase concept in open-loop
            })
    # ---- Verification trials ------------------------------------
    trials = session.get('phases', {}).get('verification', []) or []
    for trial in trials:
        samples = trial.get('samples', [])
        if len(samples) < 2:
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        target = trial.get('target_xy', [0, 0])
        ball_start = trial.get('ball_start_xy', [xs[0], ys[0]])
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        speed = np.hypot(vx, vy)
        dx = target[0] - xs
        dy = target[1] - ys
        err = np.hypot(dx, dy)
        ux = dx / np.maximum(err, 1e-3)
        uy = dy / np.maximum(err, 1e-3)
        v_toward = vx * ux + vy * uy
        for i, sample in enumerate(samples):
            rows.append({
                'phase': f'verification_trial_{trial.get("trial_idx", 0)}',
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
                'x_mm': float(xs[i]), 'y_mm': float(ys[i]),
                'vx_mm_s': float(vx[i]), 'vy_mm_s': float(vy[i]),
                'speed_mm_s': float(speed[i]),
                'err_mm': float(err[i]),
                'v_toward_target_mm_s': float(v_toward[i]),
                'imu_pitch_deg':
                    float(sample.get('imu_pitch_deg', 0.0)),
                'imu_roll_deg':
                    float(sample.get('imu_roll_deg', 0.0)),
                'cmd_pitch_deg':
                    float(sample.get('cmd_pitch_deg', 0.0)),
                'cmd_roll_deg':
                    float(sample.get('cmd_roll_deg', 0.0)),
                'phase_code':
                    float(sample.get('phase_code', -2.0)),
            })
    if not rows:
        return
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


# ---------- Phase-code colour map -----------------------------
# Mirrors stewart_control_node._ball_track_run's phase_code field.
# Used by the tilt-timeseries plot's coloured strip.
_PHASE_CODES = {
    -2: ('no diag', '#cbd5e1'),    # no diag captured for this sample
    -1: ('stale state', '#6b7280'),  # state-stale branch in ctrl node
    0:  ('settle/PID', '#10b981'),
    1:  ('accel (BB)', '#3b82f6'),
    2:  ('coast (BB)', '#f59e0b'),
    3:  ('brake (BB)', '#ef4444'),
    4:  ('STICTION', '#a855f7'),
    5:  ('PID', '#06b6d4'),
}


def _tilt_timeseries_png(session: dict, out_path: Path) -> None:
    """Per verification trial: commanded tilt vs IMU-achieved tilt,
    with a phase strip below showing PID / STICTION_BREAK / etc.
    Surfaces controller-side behaviour the existing
    step_response_per_trial.png doesn't show:
      - max_tilt saturation events (commanded clamps at the cap)
      - stiction-relief activation (phase = STICTION, magnitude
        boosted to max_tilt regardless of error)
      - inner-loop tracking quality (gap between commanded and IMU)
      - vision-noise-driven tilt commands (high-frequency commanded
        oscillation that IMU can't possibly track)

    Falls back gracefully when verification samples don't carry the
    cmd_pitch_deg / phase_code fields (older sessions): the panel
    is just blank instead of crashing.
    """
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    n = len(trials)
    # constrained_layout handles per-row title/label spacing better
    # than tight_layout for this n-row grid; without it, trial-N
    # titles overlap trial-(N-1)'s x-axis labels at n=4.
    fig, axes = plt.subplots(n, 2, figsize=(13, 3.4 * n),
                             dpi=100,
                             gridspec_kw={'width_ratios': [4, 1],
                                          'wspace': 0.18,
                                          'hspace': 0.55},
                             constrained_layout=True)
    if n == 1:
        axes = np.array([axes])
    for i, t in enumerate(trials):
        ax_main = axes[i][0]
        ax_phase = axes[i][1]
        samples = t.get('samples', [])
        if len(samples) < 5:
            ax_main.text(0.5, 0.5, 'no samples',
                         ha='center', va='center',
                         transform=ax_main.transAxes)
            continue
        ts = np.array([s['t'] for s in samples])
        cmd_p = np.array([s.get('cmd_pitch_deg', 0.0)
                          for s in samples])
        cmd_r = np.array([s.get('cmd_roll_deg', 0.0)
                          for s in samples])
        imu_p = np.array([s.get('imu_pitch_deg', 0.0)
                          for s in samples])
        imu_r = np.array([s.get('imu_roll_deg', 0.0)
                          for s in samples])
        phase = np.array([s.get('phase_code', -2.0)
                          for s in samples])
        # Detect "no diag captured" — old session bag, or BALL_TRACK
        # diag was never published. Annotate so reader knows.
        no_diag = float(np.mean(phase == -2.0)) > 0.5
        ax_main.plot(ts, cmd_p, '-', color='#ef4444', linewidth=1.5,
                     label='cmd pitch')
        ax_main.plot(ts, imu_p, '-', color='#fda4af', linewidth=1.0,
                     label='IMU pitch', alpha=0.85)
        ax_main.plot(ts, cmd_r, '-', color='#3b82f6', linewidth=1.5,
                     label='cmd roll')
        ax_main.plot(ts, imu_r, '-', color='#93c5fd', linewidth=1.0,
                     label='IMU roll', alpha=0.85)
        ax_main.axhline(0, color='#6b7280', linestyle=':',
                        linewidth=0.6)
        # Reference the active max_tilt as a horizontal band so
        # saturation is immediately visible. Pull from current_gains
        # because that's what was active during the verification trials.
        current_gains = session.get('current_gains', {}) or {}
        max_tilt = float(current_gains.get('max_tilt_deg', 2.5))
        ax_main.axhline(max_tilt, color='#dc2626', linestyle='--',
                        linewidth=0.7, alpha=0.5,
                        label=f'±max_tilt ({max_tilt:.1f}°)')
        ax_main.axhline(-max_tilt, color='#dc2626', linestyle='--',
                        linewidth=0.7, alpha=0.5)
        if t.get('settled') and t.get('settled_at_s'):
            ax_main.axvline(t['settled_at_s'], color='#10b981',
                            linestyle='--', linewidth=1.0,
                            label='settled')
        ax_main.set_xlabel('t [s]')
        ax_main.set_ylabel('tilt [°]')
        ax_main.set_title(
            f'Trial {t.get("trial_idx", i+1)}: '
            f'm{t.get("start_marker", "?")} → '
            f'm{t.get("target_marker", "?")} '
            f'{"SETTLED" if t.get("settled") else "DID NOT SETTLE"}'
            + (' [no diag]' if no_diag else ''))
        ax_main.grid(alpha=0.25)
        ax_main.legend(loc='best', fontsize=7, ncol=3)
        # Right column: phase strip — colour-coded vertical bars per
        # tick. Visualises when the controller was in PID vs
        # STICTION_BREAK vs SETTLE vs (during bang-bang) ACCEL/
        # BRAKE/COAST. For PID-only sessions you mostly see green
        # (PID) and purple (STICTION_BREAK) when the relief fires.
        unique_phases = sorted(set(int(p) for p in phase))
        # Build a stacked bar of phase fractions (proportional time
        # in each phase across the trial).
        total = len(phase)
        fractions = []
        labels = []
        colors = []
        for code in unique_phases:
            count = int(np.sum(phase == code))
            name, color = _PHASE_CODES.get(code, (f'?{code}', '#9ca3af'))
            fractions.append(count / total)
            labels.append(f'{name}\n{count/total*100:.0f}%')
            colors.append(color)
        bottom = 0.0
        for frac, color, label in zip(fractions, colors, labels):
            ax_phase.barh([0], [frac], left=[bottom],
                          height=0.5, color=color,
                          edgecolor='white', linewidth=0.5)
            if frac > 0.10:  # only label segments visible enough
                ax_phase.text(bottom + frac / 2, 0, label,
                              ha='center', va='center',
                              fontsize=7, color='white')
            bottom += frac
        ax_phase.set_xlim(0, 1)
        ax_phase.set_ylim(-0.4, 0.4)
        ax_phase.set_yticks([])
        ax_phase.set_xticks([])
        ax_phase.set_title('phase mix', fontsize=9)
    # constrained_layout (set in subplots() above) handles spacing;
    # don't call tight_layout — it warns and overrides.
    fig.suptitle(
        f'commanded vs IMU tilt + phase mix per trial   ·   '
        f'{_vision_backend_tag(session)}', fontsize=10)
    fig.savefig(out_path)
    plt.close(fig)


def _speed_histogram_png(session: dict, out_path: Path) -> None:
    """Per verification trial: histogram of ball-speed values
    (computed from KF positions via numpy.gradient). Marks the
    BT_MAX_BALL_VEL_MM_S = 800 mm/s safety-gate threshold so the
    operator can see how often vision noise pushes velocity past
    the controller's clamp.

    A trial dominated by physical motion has a unimodal speed
    distribution peaking around 100-300 mm/s. A trial corrupted by
    vision noise has a long tail or a bimodal distribution
    extending past 800 mm/s — those samples are the ones the
    controller's velocity gate is rejecting.
    """
    trials = session.get('phases', {}).get('verification', []) or []
    if not trials:
        return
    n = len(trials)
    fig, axes = plt.subplots(1, n, figsize=(4 * n, 4), dpi=100,
                             sharey=True)
    if n == 1:
        axes = np.array([axes])
    GATE = 800.0
    for i, t in enumerate(trials):
        ax = axes[i]
        samples = t.get('samples', [])
        if len(samples) < 5:
            ax.set_title(f'trial {t.get("trial_idx", i+1)}: '
                         f'too few samples')
            continue
        ts = np.array([s['t'] for s in samples])
        xs = np.array([s['x'] for s in samples])
        ys = np.array([s['y'] for s in samples])
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        speed = np.hypot(vx, vy)
        # Log scale because the tail goes to 4-5 m/s on noisy
        # sessions; linear-scale plot would just show a spike near
        # zero and you couldn't see the tail.
        bins = np.logspace(0, np.log10(max(speed.max() + 1.0, 100)),
                           40)
        ax.hist(speed, bins=bins, color='#60a5fa',
                edgecolor='#1e3a8a', alpha=0.75)
        ax.axvline(GATE, color='#dc2626', linestyle='--',
                   linewidth=1.2,
                   label=f'gate {GATE:.0f} mm/s')
        # Annotate fraction of samples above the gate.
        n_over = int(np.sum(speed > GATE))
        frac = n_over / len(speed)
        ax.text(0.97, 0.95, f'{frac*100:.1f}% > gate\n'
                            f'p95 = {np.percentile(speed, 95):.0f}\n'
                            f'max = {speed.max():.0f}',
                ha='right', va='top', transform=ax.transAxes,
                fontsize=8, color='#1f2937',
                bbox={'facecolor': 'white', 'edgecolor': '#cbd5e1',
                      'pad': 3})
        ax.set_xscale('log')
        ax.set_xlabel('speed [mm/s]')
        if i == 0:
            ax.set_ylabel('count')
        ax.set_title(
            f'Trial {t.get("trial_idx", i+1)}: '
            f'm{t.get("start_marker", "?")} → '
            f'm{t.get("target_marker", "?")}')
        ax.grid(alpha=0.25)
        ax.legend(loc='upper left', fontsize=8)
    fig.suptitle(
        f'speed distribution per trial   ·   '
        f'{_vision_backend_tag(session)}',
        fontsize=10)
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
    # Vision backend tag: which detector was running during this
    # session. Lets cross-session comparison stay apples-to-apples.
    vision_backend = (session.get('vision_backend_at_start')
                      or {}).copy()
    if not vision_backend.get('v0_backend'):
        # Fall back to first replicate's snapshot.
        reps = (session.get('phases', {})
                .get('open_loop_replicates') or [])
        if not reps and session.get('phases', {}).get('open_loop'):
            reps = [session['phases']['open_loop']]
        if reps:
            vision_backend = (reps[0].get('vision_backend_at_start')
                              or {}).copy()
    summary = {
        'current_gains': cur,
        'recommended_gains': {
            'kp': rec.get('kp'),
            'kd': rec.get('kd'),
            'ki': rec.get('ki'),
            'max_tilt_deg': rec.get('max_tilt_deg'),
        },
        'vision_backend': vision_backend,
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
    _tilt_timeseries_png(session, d / 'tilt_timeseries.png')
    _speed_histogram_png(session, d / 'speed_histogram.png')
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
    print('  tilt_timeseries.png')
    print('  speed_histogram.png')
    print('  samples.csv')
    print('  recommended_gains.png')
    print('  step_id_summary.json')
    print('  step_id_recommendation.json')


if __name__ == '__main__':
    main()
