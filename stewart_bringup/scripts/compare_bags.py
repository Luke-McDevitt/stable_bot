#!/usr/bin/env python3
"""
compare_bags.py — side-by-side comparison of digested level-loop bags.

Iterating on control gains is fast in the GUI now that the gains panel
is editable, but eyeballing JSON to compare runs is a chore. This
takes N bag dirs (or sweep dirs — children are auto-expanded) and
prints a sectioned table:

  - outer-loop gains (KP, KI, deadband, decay knobs, ...)
  - per-drive ODrive config (wL_FF, current_soft_max, ...)
  - result metrics (RMS, time-in-band, settling, ss-stats, dom-osc)

Plus an overlay PNG of err_r/err_p time series so you can see the
limit-cycle character of each run, not just the aggregate stats.

Usage:
  python3 compare_bags.py <dir> [<dir>...]
  python3 compare_bags.py tuning_data/sweep_*/   # all children of all sweeps
  python3 compare_bags.py --out-md cmp.md \\
      tuning_data/20260429T01*  tuning_data/sweep_20260429T*

The dirs you pass are the per-bag DIGEST dirs (the ones containing
<name>_summary.json). For sweeps, pass the sweep dir; per-Z children
are picked up automatically.
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    _HAVE_MPL = True
except ImportError:
    _HAVE_MPL = False


def _find_bags_in_dir(p):
    """Return list of (label, summary_path, csv_path) for every bag
    digest under p. Single bag → one record; sweep dir → one record
    per per-Z child."""
    p = Path(p)
    out = []
    # Single bag: <name>_summary.json next to the dir name == <name>
    direct = p / f"{p.name}_summary.json"
    if direct.exists():
        out.append((
            p.name,
            direct,
            p / f"{p.name}_timeseries.csv"))
        return out
    # Sweep: any *_z*_summary.json files inside.
    for child in sorted(p.iterdir() if p.is_dir() else []):
        if not child.is_file():
            continue
        n = child.name
        if n.endswith('_summary.json') and not n.endswith('sweep_summary.json'):
            stem = n[:-len('_summary.json')]
            out.append((
                f"{p.name}/{stem}",
                child,
                p / f"{stem}_timeseries.csv"))
    return out


def _first_value(per_endpoint, key):
    """Return first non-None entry from per_endpoint[key].values; useful
    when a config is symmetric across all 6 drives (the common case)."""
    e = (per_endpoint or {}).get(key) or {}
    vals = e.get('values') or []
    for v in vals:
        if v is not None:
            return v
    return None


def _all_unique(per_endpoint, key):
    """Return list of unique values seen for an endpoint across drives.
    Used to detect asymmetric inner-loop config that the simpler
    _first_value would hide."""
    e = (per_endpoint or {}).get(key) or {}
    vals = e.get('values') or []
    out = []
    for v in vals:
        if v not in out:
            out.append(v)
    return out


def extract_metrics(summary):
    """Flatten a per-bag summary.json into the dict of fields we want
    to compare. Some fields can be None if the bag predates the field
    being added — caller renders that gracefully."""
    if not isinstance(summary, dict):
        return {}
    sc = summary.get('sidecar') or {}
    gains = sc.get('gains') or {}
    base = summary.get('baseline') or {}
    sett = summary.get('settling') or {}
    ilc = summary.get('inner_loop_config') or {}
    pe = ilc.get('per_endpoint') or {}
    return {
        # context
        'duration_s': summary.get('duration_s'),
        'git_sha': (sc.get('git_sha') or '')[:7],
        'loop_hz': sc.get('level_loop_hz'),
        # outer-loop gains
        'kp': gains.get('kp'),
        'ki': gains.get('ki'),
        'deadband_deg': gains.get('deadband_deg'),
        'filter_alpha': gains.get('filter_alpha'),
        'rate_limit_deg_per_iter': gains.get('rate_limit_deg_per_iter'),
        'max_corr_deg': gains.get('max_corr_deg'),
        'integ_decay_outer_deg': gains.get('integ_decay_outer_deg'),
        'integ_decay_per_tick_at_50hz': gains.get('integ_decay_per_tick_at_50hz'),
        # inner-loop config (first drive — *_uniques in caller flags asymmetry)
        'wL_FF_enable': _first_value(pe, 'axis0.config.motor.wL_FF_enable'),
        'current_soft_max': _first_value(pe, 'axis0.config.motor.current_soft_max'),
        'vel_integrator_gain': _first_value(pe, 'axis0.controller.config.vel_integrator_gain'),
        'control_mode': _first_value(pe, 'axis0.controller.config.control_mode'),
        'input_mode': _first_value(pe, 'axis0.controller.config.input_mode'),
        'encoder_msg_rate_ms': _first_value(pe, 'axis0.config.can.encoder_msg_rate_ms'),
        '_ilc_uniques': {
            'wL_FF_enable':       _all_unique(pe, 'axis0.config.motor.wL_FF_enable'),
            'current_soft_max':   _all_unique(pe, 'axis0.config.motor.current_soft_max'),
            'vel_integrator_gain': _all_unique(pe, 'axis0.controller.config.vel_integrator_gain'),
        },
        # baseline result metrics (whole-bag aggregate)
        'mean_roll_deg': base.get('mean_roll_deg'),
        'mean_pitch_deg': base.get('mean_pitch_deg'),
        'rms_roll_deg': base.get('rms_roll_deg'),
        'rms_pitch_deg': base.get('rms_pitch_deg'),
        'p2p_roll_deg': base.get('p2p_roll_deg'),
        'p2p_pitch_deg': base.get('p2p_pitch_deg'),
        'saturation_pct_any': base.get('saturation_pct_any'),
        'fft_peak_freq_hz': base.get('fft_peak_freq_hz'),
        'fft_peak_amp_deg': base.get('fft_peak_amp_deg'),
        # settling block (post-fix metrics)
        'time_in_band_005_pct': sett.get('time_in_band_005_pct'),
        'time_in_band_010_pct': sett.get('time_in_band_010_pct'),
        'time_in_band_020_pct': sett.get('time_in_band_020_pct'),
        'settling_time_roll_s': sett.get('settling_time_roll_s'),
        'settling_time_pitch_s': sett.get('settling_time_pitch_s'),
        'ss_mean_roll_deg': sett.get('ss_mean_roll_deg'),
        'ss_mean_pitch_deg': sett.get('ss_mean_pitch_deg'),
        'ss_std_roll_deg': sett.get('ss_std_roll_deg'),
        'ss_std_pitch_deg': sett.get('ss_std_pitch_deg'),
        'integrator_drift_p2p_roll_deg': sett.get('integrator_drift_p2p_roll_deg'),
        'integrator_drift_p2p_pitch_deg': sett.get('integrator_drift_p2p_pitch_deg'),
        'dominant_osc_hz': sett.get('dominant_osc_hz'),
        'dominant_osc_period_s': sett.get('dominant_osc_period_s'),
        'dominant_osc_amp_deg': sett.get('dominant_osc_amp_deg'),
    }


# Sectioned layout for the comparison table. Each entry: (display key,
# format spec). None spec → repr().
SECTIONS = [
    ('context', [
        ('duration_s',  '7.1f'),
        ('git_sha',     '>7s'),
        ('loop_hz',     '5.0f'),
    ]),
    ('outer-loop gains', [
        ('kp',                              '6.3f'),
        ('ki',                              '6.3f'),
        ('deadband_deg',                    '6.3f'),
        ('filter_alpha',                    '6.3f'),
        ('rate_limit_deg_per_iter',         '6.3f'),
        ('max_corr_deg',                    '6.2f'),
        ('integ_decay_outer_deg',           '6.3f'),
        ('integ_decay_per_tick_at_50hz',    '6.4f'),
    ]),
    ('inner-loop config (first drive)', [
        ('wL_FF_enable',         '6s'),
        ('current_soft_max',     '6.2f'),
        ('vel_integrator_gain',  '6.3f'),
        ('control_mode',         '6d'),
        ('input_mode',           '6d'),
        ('encoder_msg_rate_ms',  '6d'),
    ]),
    ('result aggregates', [
        ('mean_roll_deg',        '+7.4f'),
        ('mean_pitch_deg',       '+7.4f'),
        ('rms_roll_deg',         '7.4f'),
        ('rms_pitch_deg',        '7.4f'),
        ('p2p_roll_deg',         '7.4f'),
        ('p2p_pitch_deg',        '7.4f'),
        ('saturation_pct_any',   '7.1f'),
    ]),
    ('settling / time-in-band (the headline metrics)', [
        ('time_in_band_005_pct',       '7.2f'),
        ('time_in_band_010_pct',       '7.2f'),
        ('time_in_band_020_pct',       '7.2f'),
        ('settling_time_roll_s',       '7.2f'),
        ('settling_time_pitch_s',      '7.2f'),
        ('ss_mean_roll_deg',           '+7.4f'),
        ('ss_mean_pitch_deg',          '+7.4f'),
        ('ss_std_roll_deg',            '7.4f'),
        ('ss_std_pitch_deg',           '7.4f'),
        ('integrator_drift_p2p_roll_deg',  '7.3f'),
        ('integrator_drift_p2p_pitch_deg', '7.3f'),
        ('dominant_osc_hz',            '7.3f'),
        ('dominant_osc_period_s',      '7.2f'),
        ('dominant_osc_amp_deg',       '7.3f'),
    ]),
]


def _fmt_cell(val, spec):
    if val is None:
        return '   —  '[: max(6, _spec_width(spec))]
    if spec.endswith('s'):
        return ('{:' + spec + '}').format(str(val))
    if spec.endswith('d'):
        try:
            return ('{:' + spec + '}').format(int(val))
        except (TypeError, ValueError):
            return f"{val!r:>6.6}"
    try:
        return ('{:' + spec + '}').format(float(val))
    except (TypeError, ValueError):
        return f"{val!r:>6.6}"


def _spec_width(spec):
    """Approximate width from a format spec like '7.4f' → 7."""
    digits = ''
    for c in spec:
        if c.isdigit():
            digits += c
        elif digits:
            break
    return int(digits or '6')


def render_table(records, out_stream=sys.stdout):
    """Print sectioned comparison table to out_stream."""
    labels = [r['_label'] for r in records]
    max_label_w = max(15, max(len(l) for l in labels))
    legend_w = 32   # max metric name width

    print(f"\n=== Comparing {len(records)} bag(s) ===", file=out_stream)
    for i, lbl in enumerate(labels):
        print(f"  {chr(ord('A') + i)}: {lbl}", file=out_stream)
    print('', file=out_stream)

    # column header (A B C ...)
    short_labels = [chr(ord('A') + i) for i in range(len(records))]
    col_w = 9
    for section_name, fields in SECTIONS:
        print(f"--- {section_name} ---", file=out_stream)
        # header
        line = ' ' * legend_w + ''.join(
            f"{sl:>{col_w}}" for sl in short_labels)
        print(line, file=out_stream)
        for key, spec in fields:
            cells = [_fmt_cell(r.get(key), spec) for r in records]
            cell_strs = [f"{c:>{col_w}}" for c in cells]
            print(f"{key:<{legend_w}}{''.join(cell_strs)}", file=out_stream)
        print('', file=out_stream)

    # Asymmetry callouts: any inner-loop knob that differs across the
    # 6 drives within a single bag — almost always a sign something was
    # written non-uniformly and that's the smoking gun for any RMS
    # difference between this bag and the others.
    asym_lines = []
    for r, lbl in zip(records, labels):
        u = r.get('_ilc_uniques') or {}
        diffs = [k for k, vals in u.items() if len(vals) > 1]
        if diffs:
            asym_lines.append(f"  [{lbl}] inner-loop config DIFFERS across drives "
                              f"on: {diffs}")
    if asym_lines:
        print("--- inner-loop config asymmetry warnings ---", file=out_stream)
        for l in asym_lines:
            print(l, file=out_stream)
        print('', file=out_stream)


def render_markdown(records, out_path):
    """Same content as render_table, but markdown for committing."""
    labels = [r['_label'] for r in records]
    short_labels = [chr(ord('A') + i) for i in range(len(records))]
    with open(out_path, 'w') as f:
        f.write(f"# Bag comparison ({len(records)} bag(s))\n\n")
        for sl, lbl in zip(short_labels, labels):
            f.write(f"- **{sl}** = `{lbl}`\n")
        f.write('\n')
        for section_name, fields in SECTIONS:
            f.write(f"## {section_name}\n\n")
            f.write('| metric | ' + ' | '.join(short_labels) + ' |\n')
            f.write('|---|' + '|'.join(['---'] * len(short_labels)) + '|\n')
            for key, spec in fields:
                cells = [_fmt_cell(r.get(key), spec).strip() for r in records]
                f.write(f"| `{key}` | " + ' | '.join(cells) + ' |\n')
            f.write('\n')
        # Asymmetry warnings
        any_asym = False
        for r, lbl in zip(records, labels):
            u = r.get('_ilc_uniques') or {}
            diffs = [k for k, vals in u.items() if len(vals) > 1]
            if diffs:
                if not any_asym:
                    f.write('## Inner-loop config asymmetry warnings\n\n')
                    any_asym = True
                f.write(f"- **{lbl}**: differs across drives on `{diffs}`\n")
        if any_asym:
            f.write('\n')


def _load_csv_timeseries(csv_path):
    """Read err_r_filt and err_p_filt + t_s from a digest timeseries
    CSV. Returns (t, err_r, err_p) numpy arrays, or (None, None, None)
    if the file is missing / unreadable."""
    if not csv_path.exists():
        return None, None, None
    try:
        rows = list(csv.DictReader(open(csv_path)))
    except Exception:
        return None, None, None
    if not rows:
        return None, None, None
    cols = rows[0].keys()
    if 'err_r_filt' not in cols or 'err_p_filt' not in cols:
        return None, None, None
    t = np.array([float(r['t_s']) for r in rows])
    err_r = np.array([float(r['err_r_filt']) for r in rows])
    err_p = np.array([float(r['err_p_filt']) for r in rows])
    return t, err_r, err_p


def render_overlay_plot(records, out_path):
    """Two-panel PNG: err_r over time for all bags overlaid (top),
    err_p (bottom). Each bag traced in a different color with the
    label-letter marker. Helps see limit-cycle character side-by-side."""
    if not _HAVE_MPL:
        return None
    fig, (ax_r, ax_p) = plt.subplots(2, 1, figsize=(13, 8),
                                     sharex=True, constrained_layout=True)
    cmap = plt.get_cmap('tab10')
    plotted = 0
    for i, r in enumerate(records):
        csvp = r.get('_csv_path')
        if csvp is None:
            continue
        t, err_r, err_p = _load_csv_timeseries(csvp)
        if t is None:
            continue
        c = cmap(i % 10)
        lbl = f"{chr(ord('A') + i)}  {r['_label']}"
        ax_r.plot(t, err_r, color=c, linewidth=0.8, label=lbl)
        ax_p.plot(t, err_p, color=c, linewidth=0.8, label=lbl)
        plotted += 1
    if plotted == 0:
        plt.close(fig)
        return None
    for ax, name in ((ax_r, 'err_r (roll)'), (ax_p, 'err_p (pitch)')):
        for thr, alpha, color in [(0.20, 0.06, 'orange'),
                                   (0.10, 0.10, 'gold'),
                                   (0.05, 0.18, 'lightgreen')]:
            ax.axhspan(-thr, thr, alpha=alpha, color=color)
        ax.axhline(0, color='black', linewidth=0.3)
        ax.set_ylabel(f'{name} (°)')
        ax.grid(True, alpha=0.3)
    ax_r.legend(loc='upper right', fontsize=8)
    ax_p.set_xlabel('t (s)')
    fig.suptitle('Bag comparison — overlay of error traces (green=±0.05°, '
                 'gold=±0.10°, orange=±0.20°)', fontsize=10)
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return str(out_path)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dirs', nargs='+',
                   help='one or more bag dirs OR sweep dirs to compare')
    p.add_argument('--out-md', default=None,
                   help='write the comparison as a markdown table here, '
                        'in addition to printing to stdout')
    p.add_argument('--out-plot', default=None,
                   help='write the err overlay PNG here. Default: '
                        'compare_<UTC>.png next to the first dir.')
    args = p.parse_args()

    # Walk dirs → bag list.
    pairs = []
    for d in args.dirs:
        dp = Path(d).expanduser().resolve()
        if not dp.exists():
            print(f"WARN: {dp} not found", file=sys.stderr)
            continue
        found = _find_bags_in_dir(dp)
        if not found:
            print(f"WARN: {dp} contained no bag digests", file=sys.stderr)
            continue
        pairs.extend(found)
    if not pairs:
        print("no bags found in any input dir", file=sys.stderr)
        return 1

    # Load summaries + extract metrics.
    records = []
    for label, summary_path, csv_path in pairs:
        try:
            summary = json.load(open(summary_path))
        except Exception as e:
            print(f"WARN: couldn't read {summary_path}: {e}", file=sys.stderr)
            continue
        rec = extract_metrics(summary)
        rec['_label'] = label
        rec['_csv_path'] = csv_path
        records.append(rec)
    if not records:
        print("no readable summaries", file=sys.stderr)
        return 1

    # Print sectioned table.
    render_table(records)

    # Optional markdown.
    if args.out_md:
        render_markdown(records, Path(args.out_md))
        print(f"\nwrote markdown: {args.out_md}")

    # Overlay plot.
    if args.out_plot is None:
        first_dir = Path(args.dirs[0]).expanduser().resolve()
        # If pointing at a single bag dir, drop the plot next to it.
        # If pointing at a sweep, drop in the sweep dir.
        out_plot = first_dir / f"compare_{len(records)}bags.png"
    else:
        out_plot = Path(args.out_plot).expanduser().resolve()
    plotted = render_overlay_plot(records, out_plot)
    if plotted:
        print(f"wrote plot:    {plotted}")
    elif _HAVE_MPL:
        print(f"(plot skipped — no readable timeseries CSVs)", file=sys.stderr)
    else:
        print(f"(plot skipped — matplotlib not installed)", file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
