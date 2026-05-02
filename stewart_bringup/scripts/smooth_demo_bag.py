#!/usr/bin/env python3
"""Offline non-causal smoother for ball position+velocity in a Demo 2 bag.

The live ball_kf_node is causal — it sees only past frames, which means
its velocity estimate is always lagged and noise-smoothed only from one
side. This script runs an offline forward+backward smoother that uses
both past AND future frames around each timestamp, producing a much
cleaner velocity signal.

Outputs (saved next to the bag, alongside digest.summary.json):
  - smoothed.json: time-series JSON with raw, smoothed, and live-KF
    position/velocity at each timestamp.
  - smoothed.png: 4-panel comparison plot — raw vs smoothed vs live-KF
    for x, y, vx, vy.
  - smoothed.summary.json: short stats on how much the live KF differs
    from the offline smoother (RMS difference per axis), plus suggested
    Q/R noise re-tuning if the divergence is large.

Why this matters:
  - Tuning analysis: when reading a digest, "what was the ball actually
    doing" becomes noise-free instead of KF-noise-floor limited.
  - Live KF re-tuning: knowing the ground-truth velocity, you can
    grid-search the live KF's process/measurement noise to minimize
    (live_kf_v - smoothed_v)² across all bags.
  - Future learned filter: the smoothed signal is the training label
    for any neural-network estimator we build later.

Method: Savitzky-Golay polynomial smoothing on raw position from
/ball_xy_mono. Window 11 frames (~330 ms at 30 Hz vision), order 3 —
preserves orbital-frequency motion (~3 s period) while killing
frame-rate noise. Velocity is the analytic derivative of the smoothed
polynomial, NOT a finite difference (so no extra noise amplification).

Usage:
  python3 smooth_demo_bag.py <bag_dir>
  python3 smooth_demo_bag.py tuning_data/20260502T025210Z_demo2

  # also print the comparison stats to stdout
  python3 smooth_demo_bag.py <bag_dir> --print-stats

  # different smoothing window (default 11 frames)
  python3 smooth_demo_bag.py <bag_dir> --window 15
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

# rosbag2 + ROS deserialization
from rosbag2_py import (SequentialReader, StorageOptions,
                        ConverterOptions)
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped, PointStamped

try:
    from scipy.signal import savgol_filter
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except ImportError:
    HAVE_MPL = False


def open_bag(bag_dir: str):
    """Same dual-mode open as digest_demo_bag.py — try the directory
    auto-detect first, fall back to the explicit .mcap file."""
    storage = StorageOptions(uri=bag_dir, storage_id='')
    converter = ConverterOptions('', '')
    reader = SequentialReader()
    try:
        reader.open(storage, converter)
        return reader
    except Exception as e_auto:
        mcaps = sorted(p for p in os.listdir(bag_dir) if p.endswith('.mcap'))
        last_err = e_auto
        for mf in mcaps:
            try:
                storage2 = StorageOptions(
                    uri=os.path.join(bag_dir, mf), storage_id='mcap')
                reader = SequentialReader()
                reader.open(storage2, converter)
                return reader
            except Exception as e_file:
                last_err = e_file
        raise RuntimeError(f"could not open bag at {bag_dir}: {last_err}")


def read_topics(bag_dir: str) -> dict:
    """Return {topic: (t_ns_array, data_array)} for the topics we care
    about. Position arrays are shape (N, 2); velocity arrays are
    shape (N, 2). Times are int64 ns since epoch."""
    reader = open_bag(bag_dir)
    topic_types = {t.name: t.type
                   for t in reader.get_all_topics_and_types()}
    classes = {
        '/ball_state':       PoseStamped,
        '/ball_xy_mono':     PointStamped,
        '/ball_xy_depth':    PointStamped,
        '/ball_ref':         PointStamped,
    }
    out = {k: ([], [], []) for k in classes}  # t, xy, [vx,vy if applicable]
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        cls = classes.get(topic)
        if cls is None or topic not in topic_types:
            continue
        try:
            msg = deserialize_message(raw, cls)
        except Exception:
            continue
        if topic == '/ball_state':
            out[topic][0].append(t_ns)
            out[topic][1].append([float(msg.pose.position.x),
                                  float(msg.pose.position.y)])
            out[topic][2].append([float(msg.pose.orientation.x),
                                  float(msg.pose.orientation.y)])
        else:
            out[topic][0].append(t_ns)
            out[topic][1].append([float(msg.point.x), float(msg.point.y)])

    result = {}
    for k, (t, xy, v) in out.items():
        if not t:
            result[k] = None
            continue
        t_arr = np.asarray(t, dtype=np.int64)
        xy_arr = np.asarray(xy, dtype=np.float64)
        v_arr = np.asarray(v, dtype=np.float64) if v else None
        result[k] = (t_arr, xy_arr, v_arr)
    return result


def smooth_position(t_ns: np.ndarray, xy: np.ndarray,
                    window: int = 11, order: int = 3
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Savitzky-Golay smoother on x and y independently. Returns
    (smoothed_xy, smoothed_v_xy_mm_per_s).

    Velocity comes from the analytic derivative of the same polynomial
    fit (savgol_filter with deriv=1), scaled by 1/dt. This is much
    cleaner than computing finite differences on the smoothed position.
    """
    if not HAVE_SCIPY:
        # Fallback: simple boxcar smoother + central differences.
        # Less clean than Sav-Gol but no scipy dependency.
        n = len(xy)
        w = window
        kernel = np.ones(w) / w
        smooth = np.empty_like(xy)
        smooth[:, 0] = np.convolve(xy[:, 0], kernel, mode='same')
        smooth[:, 1] = np.convolve(xy[:, 1], kernel, mode='same')
        # Edge fix-up: pad smoothed with raw values at the boundaries
        # where convolution is biased by zero-padding.
        half = w // 2
        smooth[:half] = xy[:half]
        smooth[-half:] = xy[-half:]
        # Central difference for velocity
        v = np.gradient(smooth, axis=0)
        dt_s = np.gradient(t_ns).astype(np.float64) * 1e-9
        v[:, 0] /= dt_s
        v[:, 1] /= dt_s
        return smooth, v

    n = len(xy)
    if n < window:
        # Window must be ≤ data length and odd. Adjust.
        if n < 5:
            # Not enough samples to smooth meaningfully — return raw.
            v = np.zeros_like(xy)
            return xy.copy(), v
        window = n - 1 if n % 2 == 0 else n - 2
        window = max(window, 5)
        order = min(order, window - 1)

    # Median dt for the polynomial-derivative scaling. Bag streams
    # aren't perfectly uniform, but the median is a good representative
    # for short windows.
    dt_s = float(np.median(np.diff(t_ns))) * 1e-9
    if dt_s <= 0:
        dt_s = 1.0 / 30.0

    smooth_x = savgol_filter(xy[:, 0], window, order, mode='nearest')
    smooth_y = savgol_filter(xy[:, 1], window, order, mode='nearest')
    # deriv=1 returns dx/d_index; divide by dt_s to get dx/dt in mm/s.
    vx = savgol_filter(xy[:, 0], window, order, deriv=1,
                       delta=dt_s, mode='nearest')
    vy = savgol_filter(xy[:, 1], window, order, deriv=1,
                       delta=dt_s, mode='nearest')
    smooth_xy = np.column_stack([smooth_x, smooth_y])
    smooth_v = np.column_stack([vx, vy])
    return smooth_xy, smooth_v


def compare_kf_vs_smooth(state_t: np.ndarray, state_xy: np.ndarray,
                         state_v: np.ndarray,
                         smooth_t: np.ndarray, smooth_xy: np.ndarray,
                         smooth_v: np.ndarray) -> dict:
    """Interpolate the offline-smoothed signal onto the live-KF
    timestamps and return per-axis RMS divergence."""
    if state_t is None or state_t.size == 0 \
            or smooth_t is None or smooth_t.size == 0:
        return {}
    t_ref = state_t.astype(np.float64) * 1e-9
    t_src = smooth_t.astype(np.float64) * 1e-9
    # Clip query times to source range so np.interp doesn't extrapolate
    t_q = np.clip(t_ref, t_src[0], t_src[-1])
    sx = np.interp(t_q, t_src, smooth_xy[:, 0])
    sy = np.interp(t_q, t_src, smooth_xy[:, 1])
    svx = np.interp(t_q, t_src, smooth_v[:, 0])
    svy = np.interp(t_q, t_src, smooth_v[:, 1])

    dx = state_xy[:, 0] - sx
    dy = state_xy[:, 1] - sy
    dvx = state_v[:, 0] - svx
    dvy = state_v[:, 1] - svy
    return {
        'pos_x_rms_mm':       float(np.sqrt(np.mean(dx**2))),
        'pos_y_rms_mm':       float(np.sqrt(np.mean(dy**2))),
        'vel_x_rms_mm_per_s': float(np.sqrt(np.mean(dvx**2))),
        'vel_y_rms_mm_per_s': float(np.sqrt(np.mean(dvy**2))),
        'pos_x_max_diff_mm':  float(np.max(np.abs(dx))),
        'pos_y_max_diff_mm':  float(np.max(np.abs(dy))),
        'vel_x_max_diff_mm_per_s': float(np.max(np.abs(dvx))),
        'vel_y_max_diff_mm_per_s': float(np.max(np.abs(dvy))),
        'n_samples':          int(state_t.size),
    }


def render_plot(bag_dir: Path,
                mono_t: np.ndarray, mono_xy: np.ndarray,
                smooth_t: np.ndarray, smooth_xy: np.ndarray,
                smooth_v: np.ndarray,
                state_t: np.ndarray | None, state_xy: np.ndarray | None,
                state_v: np.ndarray | None) -> None:
    """4-panel plot: x, y, vx, vy. Overlay raw / smoothed / live-KF."""
    if not HAVE_MPL:
        print('[smooth] matplotlib not available — skipping plot',
              file=sys.stderr)
        return
    t0 = mono_t[0] if mono_t.size else 0
    def secs(t): return (t - t0) * 1e-9 if t.size else np.array([])

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    titles = ['x position [mm]', 'y position [mm]',
              'vx [mm/s]', 'vy [mm/s]']
    raw_data = [mono_xy[:, 0], mono_xy[:, 1], None, None]
    smooth_data = [smooth_xy[:, 0], smooth_xy[:, 1],
                   smooth_v[:, 0], smooth_v[:, 1]]
    live_data = ([state_xy[:, 0], state_xy[:, 1],
                  state_v[:, 0], state_v[:, 1]]
                 if state_t is not None else [None]*4)

    t_mono = secs(mono_t)
    t_smooth = secs(smooth_t)
    t_state = secs(state_t) if state_t is not None else np.array([])

    for i, (ax, title) in enumerate(zip(axes, titles)):
        if raw_data[i] is not None:
            ax.plot(t_mono, raw_data[i], '.', ms=2,
                    color='lightgray', alpha=0.5,
                    label='raw vision')
        if live_data[i] is not None and t_state.size:
            ax.plot(t_state, live_data[i], '-',
                    color='tab:orange', lw=1.0, alpha=0.8,
                    label='live KF (causal)')
        ax.plot(t_smooth, smooth_data[i], '-',
                color='tab:blue', lw=1.5,
                label='offline smoother')
        ax.set_ylabel(title)
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(loc='upper right', fontsize=8)
    axes[-1].set_xlabel('time [s]')
    fig.suptitle(f'Offline smoother — {bag_dir.name}', fontsize=11)
    fig.tight_layout()
    out = bag_dir / 'smoothed.png'
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f'  → {out}')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag_dir', help='path to a Demo 2 bag directory')
    p.add_argument('--window', type=int, default=11,
                   help='Sav-Gol smoothing window in frames '
                        '(odd, default 11 ≈ 330 ms at 30 Hz)')
    p.add_argument('--order', type=int, default=3,
                   help='Sav-Gol polynomial order (default 3)')
    p.add_argument('--print-stats', action='store_true',
                   help='print KF-vs-smoother divergence stats')
    p.add_argument('--no-plot', action='store_true',
                   help='skip generating smoothed.png')
    args = p.parse_args()

    bag_dir = Path(args.bag_dir).resolve()
    if not bag_dir.is_dir():
        print(f'not a directory: {bag_dir}', file=sys.stderr)
        sys.exit(2)

    print(f'[smooth] reading {bag_dir}')
    topics = read_topics(str(bag_dir))

    mono = topics.get('/ball_xy_mono')
    state = topics.get('/ball_state')
    if mono is None:
        print('[smooth] no /ball_xy_mono in bag — nothing to smooth',
              file=sys.stderr)
        sys.exit(2)
    mono_t, mono_xy, _ = mono
    state_t, state_xy, state_v = state if state is not None else (None, None, None)

    print(f'[smooth]   /ball_xy_mono: {mono_t.size} samples')
    if state_t is not None:
        print(f'[smooth]   /ball_state:   {state_t.size} samples')

    if args.window % 2 == 0:
        args.window += 1
    smooth_xy, smooth_v = smooth_position(
        mono_t, mono_xy, window=args.window, order=args.order)
    smooth_t = mono_t  # same time base as the raw input

    # Stats on KF vs smoother divergence.
    stats = {}
    if state_t is not None and state_t.size > 0:
        stats = compare_kf_vs_smooth(
            state_t, state_xy, state_v,
            smooth_t, smooth_xy, smooth_v)

    # Save smoothed time-series JSON.
    out_json = bag_dir / 'smoothed.json'
    smoothed_data = {
        'bag': str(bag_dir),
        'window': args.window,
        'order': args.order,
        'smoother': 'savitzky-golay' if HAVE_SCIPY else 'boxcar',
        't_ns': mono_t.tolist(),
        'raw_x_mm': mono_xy[:, 0].tolist(),
        'raw_y_mm': mono_xy[:, 1].tolist(),
        'smooth_x_mm': smooth_xy[:, 0].tolist(),
        'smooth_y_mm': smooth_xy[:, 1].tolist(),
        'smooth_vx_mm_per_s': smooth_v[:, 0].tolist(),
        'smooth_vy_mm_per_s': smooth_v[:, 1].tolist(),
    }
    if state_t is not None:
        smoothed_data['state_t_ns'] = state_t.tolist()
        smoothed_data['state_x_mm'] = state_xy[:, 0].tolist()
        smoothed_data['state_y_mm'] = state_xy[:, 1].tolist()
        smoothed_data['state_vx_mm_per_s'] = state_v[:, 0].tolist()
        smoothed_data['state_vy_mm_per_s'] = state_v[:, 1].tolist()
    with out_json.open('w') as f:
        json.dump(smoothed_data, f)
    print(f'  → {out_json}')

    # Save short summary JSON.
    out_summary = bag_dir / 'smoothed.summary.json'
    summary = {
        'bag': str(bag_dir),
        'mono_n': int(mono_t.size),
        'state_n': int(state_t.size) if state_t is not None else 0,
        'window': args.window,
        'order': args.order,
        'smoother': 'savitzky-golay' if HAVE_SCIPY else 'boxcar',
        'kf_vs_smoother': stats,
    }
    if stats:
        # Heuristic Q/R re-tune suggestion. If velocity RMS divergence
        # is large compared to the smoother's typical signal, the live
        # KF is over- or under-trusting measurements.
        v_rms = np.sqrt(np.mean(smooth_v**2))
        v_div = (stats.get('vel_x_rms_mm_per_s', 0) +
                 stats.get('vel_y_rms_mm_per_s', 0)) / 2.0
        if v_rms > 0:
            ratio = v_div / v_rms
            if ratio > 0.3:
                summary['retune_hint'] = (
                    f'live KF velocity diverges from smoother by '
                    f'{ratio:.0%} of signal RMS — consider lowering '
                    f'measurement noise R (more trust in vision) or '
                    f'raising process noise Q (more trust in change).')
            else:
                summary['retune_hint'] = (
                    f'live KF tracks smoother to within {ratio:.0%} of '
                    f'signal RMS — Q/R are reasonable.')
    with out_summary.open('w') as f:
        json.dump(summary, f, indent=2)
    print(f'  → {out_summary}')

    if not args.no_plot:
        render_plot(bag_dir, mono_t, mono_xy,
                    smooth_t, smooth_xy, smooth_v,
                    state_t, state_xy, state_v)

    if args.print_stats and stats:
        print()
        print('KF vs offline smoother divergence:')
        for k, v in stats.items():
            print(f'  {k:30s}  {v:8.2f}'
                  if isinstance(v, float) else f'  {k:30s}  {v}')


if __name__ == '__main__':
    main()
