# tuning_data/

Per-bag and per-sweep digests written by
`stewart_bringup/scripts/analyze_level_bag.py`. Two file types live here,
enforced by `.gitignore`:

- `*_summary.json` — gains snapshot, baseline RMS / FFT / saturation,
  per-step rise/overshoot/settling/ss-offset, aggregated.
- `*_timeseries.csv` — downsampled (50 Hz → 10 Hz) `t, roll, pitch,
  err_*, integ_*, pi_out_*, corr_*, target_*, clip_flags`.

Sweep dirs additionally have `<sweep>_sweep_summary.json` with the
Z-vs-{rms, settling, overshoot} rollup.

## How to populate

After a bench session on the Pi:

```bash
# single-bag digest:
python3 ~/stable_bot_repo/stewart_bringup/scripts/analyze_level_bag.py \
  ~/stable_bot_bags/<UTC>_<name>/

# sweep digest (processes every child bag + writes the rollup):
python3 ~/stable_bot_repo/stewart_bringup/scripts/analyze_level_bag.py \
  ~/stable_bot_bags/sweep_<UTC>_<prefix>/
```

Or click "Export digest" on the bag in the GUI's Recent-bags list,
which calls the same script via `gui_server.py`.

Then commit + push:

```bash
cd ~/stable_bot_repo
git add tuning_data/
git commit -m "Tuning bench session $(date -u +%Y-%m-%dT%H:%MZ)"
git push
```

## Why digests not raw bags

Raw bags are MCAP/SQLite, MB-to-GB. Useless to read remotely without
rosbag2, and bloats the repo. Digests are KB-scale, diff-able, and
human-readable. Raw bags stay on the Pi at `~/stable_bot_bags/`.
