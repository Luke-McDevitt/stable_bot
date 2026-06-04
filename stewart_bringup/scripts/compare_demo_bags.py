#!/usr/bin/env python3
"""Compare gains and outcome metrics across multiple Demo 2 bags.

Walks each bag directory's `digest.summary.json` (produced by
`digest_demo_bag.py`), pulls the gains snapshot + outcome metrics out of
each, and prints a single comparison table sorted by tracking quality.

Useful when you've run a dozen variations of gains and you need to see
"which gain change moved which metric, and which combo actually worked."

Usage:
  python3 compare_demo_bags.py
      → compares every *_demo2/ under ./tuning_data/

  python3 compare_demo_bags.py --glob 'tuning_data/20260502T02*_demo2'
      → restrict to a time window

  python3 compare_demo_bags.py --sort-by saturation_fraction
      → sort by a different metric

  python3 compare_demo_bags.py --csv
      → emit CSV instead of pretty table (paste into spreadsheets)

  python3 compare_demo_bags.py --diff
      → highlight gains that *changed* between consecutive runs
        (greys out columns that stay constant the whole sweep)

Designed to live alongside digest_demo_bag.py and consume its outputs;
no rosbag2 reading happens here — purely JSON aggregation.
"""
from __future__ import annotations

import argparse
import csv
import glob as glob_mod
import json
import os
import sys
from pathlib import Path


# Gains we care about for Demo 2 tuning. Order = display order.
# Anything in `gains_at_record` not in this list is silently ignored
# from the table (still readable in the raw JSON if you need it).
GAIN_KEYS_PID = [
    'algorithm',
    'kp',
    'kd',
    'ki',
    'max_tilt_deg',
    'kd_v_tau_s',
    'control_latency_s',
    # stiction RAMP knobs — the ones that have actually been moved
    'stiction_ramp_start_deg',
    'stiction_ramp_max_deg',
    'stiction_ramp_rate_deg_per_s',
    'stiction_ramp_timeout_s',
    'stiction_v_threshold_mm_s',
    'stiction_break_s',
    'stiction_pos_delta_mm',
    'stiction_moving_hyst_ticks',
    'tilt_slew_up_deg_per_s',
]

# Outcome metrics we care about (extracted from next_step.metrics +
# error_mm + a few others). Lower is better unless flagged.
METRIC_KEYS = [
    ('duration_s', 's', False),                # higher is more data
    ('median_tail_err_mm', 'mm', True),        # ↓ is better
    ('p95_err_mm', 'mm', True),
    ('peak_speed_mm_s', 'mm/s', True),
    ('saturation_fraction', '%', True),
    ('cmd_jitter_deg_per_tick', '°/tick', True),
    ('settled', '', False),                    # bool, True is better
]

# Run-config caps (the live-settable limits that LABEL an A/B — the gap
# that made the 1.0-vs-1.5 vel run unreadable) + measured latency +
# whether the legs hit the current cap (torque-limited evidence).
CONFIG_KEYS = ['soft_max_vel_tps', 'leg_current_cap_a']
LATENCY_KEYS = ['actuation_ms', 'act_ambiguous', 'see_to_move_ms',
                'cur_peak_a', 'cur_sat']


def load_summary(bag_dir: Path) -> dict | None:
    p = bag_dir / 'digest.summary.json'
    if not p.exists():
        return None
    try:
        with p.open() as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"[warn] failed to read {p}: {e}", file=sys.stderr)
        return None


def extract_row(bag_dir: Path, summary: dict) -> dict:
    """Pull a flat dict of {col: value} from a summary JSON."""
    bag_name = bag_dir.name
    # Time component for chronological sort. Bag dirs are
    # YYYYMMDDTHHMMSSZ_label or 150035Z_label; either sorts
    # lexicographically.
    row = {'bag': bag_name}
    gains = summary.get('gains_at_record') or {}
    for k in GAIN_KEYS_PID:
        row[k] = gains.get(k)
    next_step = summary.get('next_step') or {}
    metrics = next_step.get('metrics') or {}
    for mk, _unit, _lower_better in METRIC_KEYS:
        if mk == 'duration_s':
            row[mk] = summary.get('duration_s')
        else:
            row[mk] = metrics.get(mk)
    # Error stats from error_mm if available.
    err = summary.get('error_mm') or {}
    row['err_mean_mm'] = err.get('mean')
    row['err_rms_mm'] = err.get('rms')
    # Run-config caps (label the A/B) + measured latency + current-cap
    # saturation. All produced by the upgraded digest; older bags just
    # show '-'.
    rc = summary.get('run_config') or {}
    row['soft_max_vel_tps'] = rc.get('soft_max_vel_tps')
    row['leg_current_cap_a'] = rc.get('leg_current_cap_a')
    lb = summary.get('latency_breakdown') or {}
    act = lb.get('actuation') if isinstance(lb.get('actuation'), dict) else {}
    row['actuation_ms'] = act.get('actuation_ms')
    row['act_ambiguous'] = act.get('ambiguous')
    row['see_to_move_ms'] = lb.get('see_to_move_est_ms')
    lc = summary.get('leg_current') or {}
    row['cur_peak_a'] = lc.get('measured_peak_a')
    row['cur_sat'] = lc.get('saturation_fraction')
    return row


def fmt_cell(v, sig: int = 4) -> str:
    if v is None:
        return '-'
    if isinstance(v, bool):
        return 'Y' if v else 'N'
    if isinstance(v, (int, float)):
        if isinstance(v, float):
            # tighter format for small numbers, wider for big
            if abs(v) >= 1000:
                return f'{v:,.0f}'
            elif abs(v) >= 100:
                return f'{v:.1f}'
            elif abs(v) >= 10:
                return f'{v:.2f}'
            elif abs(v) >= 1:
                return f'{v:.3f}'
            else:
                return f'{v:.4f}'
        return f'{v:,}'
    return str(v)


def render_table(rows: list[dict], cols: list[str],
                 highlight_best_idx: int | None = None) -> str:
    """Right-align numeric, left-align bag/algo. Header underline."""
    if not rows:
        return '(no rows)'
    # Pretty headers — strip _deg etc for display compactness.
    pretty = {
        'bag':                            'bag',
        'algorithm':                      'algo',
        'kp':                             'kp',
        'kd':                             'kd',
        'ki':                             'ki',
        'max_tilt_deg':                   'maxT',
        'kd_v_tau_s':                     'kd_τ',
        'control_latency_s':              'lat',
        'stiction_ramp_start_deg':        'rStart',
        'stiction_ramp_max_deg':          'rMax',
        'stiction_ramp_rate_deg_per_s':   'rRate',
        'stiction_ramp_timeout_s':        'rTmO',
        'stiction_v_threshold_mm_s':      'vThr',
        'stiction_break_s':               'brk_s',
        'stiction_pos_delta_mm':          'posΔ',
        'stiction_moving_hyst_ticks':     'hyst',
        'tilt_slew_up_deg_per_s':         'slew',
        'duration_s':                     'dur',
        'median_tail_err_mm':             'medErr',
        'p95_err_mm':                     'p95Err',
        'peak_speed_mm_s':                'pkSpd',
        'saturation_fraction':            'sat',
        'cmd_jitter_deg_per_tick':        'jit',
        'settled':                        'set',
        'err_mean_mm':                    'errMu',
        'err_rms_mm':                     'errRMS',
        'soft_max_vel_tps':               'velCap',
        'leg_current_cap_a':              'curCap',
        'actuation_ms':                   'actMs',
        'act_ambiguous':                  'amb?',
        'see_to_move_ms':                 's2mMs',
        'cur_peak_a':                     'curPk',
        'cur_sat':                        'curSat',
    }
    headers = [pretty.get(c, c) for c in cols]
    cells = [[fmt_cell(r.get(c)) for c in cols] for r in rows]
    widths = [len(h) for h in headers]
    for row in cells:
        for i, c in enumerate(row):
            widths[i] = max(widths[i], len(c))
    lines: list[str] = []
    # Left-align bag column; right-align everything else.
    aligns = ['l' if c == 'bag' else 'r' for c in cols]
    def fmt_row(parts: list[str]) -> str:
        out = []
        for i, p in enumerate(parts):
            if aligns[i] == 'l':
                out.append(p.ljust(widths[i]))
            else:
                out.append(p.rjust(widths[i]))
        return '  '.join(out)
    lines.append(fmt_row(headers))
    lines.append('  '.join('-' * w for w in widths))
    for i, row in enumerate(cells):
        marker = ''
        if highlight_best_idx is not None and i == highlight_best_idx:
            marker = '  ★ best'
        lines.append(fmt_row(row) + marker)
    return '\n'.join(lines)


def collapse_constant_columns(rows: list[dict],
                              cols: list[str]) -> list[str]:
    """Return only columns that VARY across the row set. A column where
    all rows have the same value gets dropped from the diff view."""
    keep = []
    for c in cols:
        values = {repr(r.get(c)) for r in rows}
        if len(values) > 1:
            keep.append(c)
    return keep


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--glob', default='tuning_data/*_demo2',
                   help="bag-dir glob (default: tuning_data/*_demo2)")
    p.add_argument('--sort-by',
                   default='median_tail_err_mm',
                   help='metric to sort by (default: median_tail_err_mm)')
    p.add_argument('--ascending', action='store_true', default=True,
                   help='sort ascending (default for error metrics)')
    p.add_argument('--csv', action='store_true',
                   help='emit CSV instead of formatted table')
    p.add_argument('--diff', action='store_true',
                   help='only show columns that vary across the set')
    p.add_argument('--chronological', action='store_true',
                   help='sort by bag timestamp instead of metric')
    p.add_argument('--repo', default='.',
                   help='repo root (glob is resolved relative to this)')
    args = p.parse_args()

    repo = Path(args.repo).resolve()
    pattern = str(repo / args.glob)
    bag_dirs = sorted(Path(d) for d in glob_mod.glob(pattern)
                      if Path(d).is_dir())
    if not bag_dirs:
        print(f"no bags matched: {pattern}", file=sys.stderr)
        sys.exit(2)

    rows: list[dict] = []
    skipped = 0
    for bd in bag_dirs:
        s = load_summary(bd)
        if s is None:
            skipped += 1
            continue
        rows.append(extract_row(bd, s))
    if not rows:
        print(f"matched {len(bag_dirs)} bags but none had digest.summary.json",
              file=sys.stderr)
        sys.exit(2)

    # Sort.
    if args.chronological:
        rows.sort(key=lambda r: r['bag'])
    else:
        # Default: ascending by sort key, treating None as worst.
        sort_key = args.sort_by
        rows.sort(key=lambda r: (r.get(sort_key) is None,
                                 r.get(sort_key)
                                 if r.get(sort_key) is not None else 0))

    cols = (['bag'] + GAIN_KEYS_PID + CONFIG_KEYS
            + [m[0] for m in METRIC_KEYS]
            + ['err_mean_mm', 'err_rms_mm'] + LATENCY_KEYS)

    if args.diff:
        # Always keep bag column; collapse the rest based on variation.
        kept = collapse_constant_columns(rows, cols[1:])
        cols = ['bag'] + kept

    if args.csv:
        w = csv.DictWriter(sys.stdout, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow(r)
        return

    # Highlight the best row (lowest median_tail_err_mm if available).
    best_idx = None
    if not args.chronological:
        best_idx = 0  # already sorted ascending → row 0 is best
    print(f"# Compared {len(rows)} bags  "
          f"({skipped} skipped — no digest.summary.json)")
    print(f"# Sorted by: {'bag (chronological)' if args.chronological else args.sort_by} "
          f"({'asc' if args.ascending else 'desc'})")
    if args.diff:
        print(f"# DIFF view: only columns that varied across the set are shown")
    print()
    print(render_table(rows, cols, highlight_best_idx=best_idx))
    print()
    # Quick callouts.
    if not args.chronological and rows:
        best = rows[0]
        worst = rows[-1]
        sk = args.sort_by
        if best.get(sk) is not None and worst.get(sk) is not None:
            print(f"best  → {best['bag']}  ({sk}={fmt_cell(best.get(sk))})")
            print(f"worst → {worst['bag']}  ({sk}={fmt_cell(worst.get(sk))})")


if __name__ == '__main__':
    main()
