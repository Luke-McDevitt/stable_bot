#!/usr/bin/env python3
"""Digest an auto-tune session: produce summary PNGs from log.jsonl.

Reads tuning_data/auto_tune_<UTC>/log.jsonl (one row per trial) and
emits two PNGs + a digest.summary.json next to it:

  fitness_curve.png    — per-trial fitness, with running max overlaid.
                         The "training curve" of the optimizer.
  gain_trajectory.png  — each gain plotted vs trial number. Shows
                         which knobs the algorithm leaned on.
  target_coverage.png  — scatter of trial targets on the platform,
                         colored by trial fitness.
  digest.summary.json  — best-trial summary + run statistics.

Usage:
    python3 stewart_bringup/scripts/digest_auto_tune_bag.py \\
        ~/stable_bot_repo/tuning_data/auto_tune_20260501T123000Z

The digest is JSONL-based (not bag-based) — same data, much faster
to render. The bag is for retrospective deep-dive of /ball_state vs
/control_cmd; the digest is the at-a-glance view.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _load_jsonl(path: Path) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _fitness_curve_png(rows: list[dict], out_path: Path) -> None:
    trials = [r['trial'] for r in rows]
    fits = [r.get('fitness', 0.0) for r in rows]
    # Running best.
    best = []
    cur = -1.0
    for f in fits:
        cur = max(cur, f)
        best.append(cur)
    fig, ax = plt.subplots(figsize=(7, 4), dpi=100)
    ax.plot(trials, fits, 'o-', color='#60a5fa',
            markersize=4, alpha=0.75, label='per-trial fitness')
    ax.plot(trials, best, '-', color='#10b981',
            linewidth=2, label='running best')
    ax.set_xlabel('trial')
    ax.set_ylabel('fitness')
    ax.set_title('auto-tune fitness curve')
    ax.set_ylim(0.0, 1.0)
    ax.grid(alpha=0.25)
    ax.legend(loc='lower right', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _gain_trajectory_png(rows: list[dict], out_path: Path) -> None:
    trials = [r['trial'] for r in rows]
    # Collect all gain keys that appear at least once.
    keys: set[str] = set()
    for r in rows:
        keys.update(r.get('gains', {}).keys())
    keys = sorted(k for k in keys
                  if k in ('kp', 'kd', 'ki', 'max_tilt_deg',
                           'pitch_sign', 'roll_sign'))
    if not keys:
        return
    fig, axes = plt.subplots(len(keys), 1, figsize=(7, 1.6 * len(keys)),
                             sharex=True, dpi=100)
    if len(keys) == 1:
        axes = [axes]
    for i, k in enumerate(keys):
        vals = [r.get('gains', {}).get(k, np.nan) for r in rows]
        axes[i].plot(trials, vals, 'o-', color='#60a5fa',
                     markersize=3, linewidth=1)
        axes[i].set_ylabel(k, fontsize=9)
        axes[i].grid(alpha=0.25)
    axes[-1].set_xlabel('trial')
    fig.suptitle('gain trajectory', fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _target_coverage_png(rows: list[dict], out_path: Path) -> None:
    xs = []; ys = []; fits = []
    for r in rows:
        t = r.get('target') or {}
        if 'x_mm' not in t or 'y_mm' not in t:
            continue
        xs.append(t['x_mm'])
        ys.append(t['y_mm'])
        fits.append(r.get('fitness', 0.0))
    if not xs:
        return
    fig, ax = plt.subplots(figsize=(6, 6), dpi=100)
    sc = ax.scatter(xs, ys, c=fits, cmap='viridis', s=40,
                    edgecolors='#0f172a', vmin=0.0, vmax=1.0)
    # Platform disk + safe + center exclusion overlays.
    R = 200.0
    ax.add_patch(plt.Circle((0, 0), R, fill=False,
                            edgecolor='#475569', linewidth=1))
    ax.add_patch(plt.Circle((0, 0), 0.7 * R, fill=False,
                            edgecolor='#64748b', linewidth=0.7,
                            linestyle='--'))
    ax.add_patch(plt.Circle((0, 0), 40, fill=False,
                            edgecolor='#dc2626', linewidth=1,
                            linestyle=':'))
    ax.set_xlim(-R - 20, R + 20)
    ax.set_ylim(-R - 20, R + 20)
    ax.set_aspect('equal')
    ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm)')
    ax.set_title('target coverage (color = fitness)')
    ax.grid(alpha=0.2)
    cbar = fig.colorbar(sc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label('fitness')
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def _summary(rows: list[dict]) -> dict:
    if not rows:
        return {'n_trials': 0}
    best = max(rows, key=lambda r: r.get('fitness', 0))
    fits = [r.get('fitness', 0.0) for r in rows]
    return {
        'n_trials': len(rows),
        'best_fitness': best.get('fitness', 0.0),
        'best_trial': best.get('trial', 0),
        'best_gains': best.get('gains', {}),
        'best_components': best.get('components', {}),
        'mean_fitness': float(np.mean(fits)),
        'median_fitness': float(np.median(fits)),
        'final_fitness': fits[-1],
        'n_settled': sum(1 for r in rows if r.get('settled')),
        'n_aborted': sum(1 for r in rows if r.get('aborted')),
        'n_stuck_on_center': sum(
            1 for r in rows
            if r.get('components', {}).get('stuck_on_center_fraction', 0) > 0.4),
    }


def digest(session_dir: Path) -> int:
    log_path = session_dir / 'log.jsonl'
    if not log_path.is_file():
        print(f'ERROR: no log.jsonl in {session_dir}', file=sys.stderr)
        return 2
    rows = _load_jsonl(log_path)
    if not rows:
        print(f'ERROR: log.jsonl is empty', file=sys.stderr)
        return 2
    _fitness_curve_png(rows, session_dir / 'fitness_curve.png')
    _gain_trajectory_png(rows, session_dir / 'gain_trajectory.png')
    _target_coverage_png(rows, session_dir / 'target_coverage.png')
    summary = _summary(rows)
    summary_path = session_dir / 'digest.summary.json'
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    print(f'wrote: {summary_path}')
    print(f'  n_trials: {summary["n_trials"]}')
    print(f'  best_fitness: {summary.get("best_fitness", 0):.3f} '
          f'(trial {summary.get("best_trial", 0)})')
    print(f'  best_gains: {summary.get("best_gains")}')
    print(f'  settled: {summary.get("n_settled")} / {summary["n_trials"]}')
    print(f'  aborted: {summary.get("n_aborted")}')
    print(f'  center-stuck: {summary.get("n_stuck_on_center")}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='Path to auto_tune_<UTC>/ directory')
    args = ap.parse_args()
    sys.exit(digest(Path(args.session).expanduser().resolve()))


if __name__ == '__main__':
    main()
