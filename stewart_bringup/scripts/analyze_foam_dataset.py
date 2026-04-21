#!/usr/bin/env python3
"""
Batch analyzer for labeled stall_home collect runs.

For each (canlog, label.json) pair, extract the main motion phase (vel_cmd
holding its original sign, before retract), segment it into "foam" and
"non-foam" windows per the label's foam_contact / foam_contact_time_s, and
compute features. Then compare feature distributions between the two classes
to see if a simple CAN-visible signature distinguishes foam contact.
"""
import argparse
import glob
import json
import os
import re
import statistics
import struct
import sys

CMD_GET_ENCODER         = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_VEL       = 0x00D
CMD_GET_IQ              = 0x014
CMD_SET_AXIS_STATE      = 0x007
STATE_IDLE = 1

LINE_RE = re.compile(r'\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)')


def parse_log(path, node_id=0):
    """Returns list of (ts, cmd, data_bytes) for matching node."""
    out = []
    with open(path) as f:
        for line in f:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            ts = float(m.group(1))
            arb = int(m.group(2), 16)
            if (arb >> 5) != node_id:
                continue
            cmd = arb & 0x1F
            data = bytes.fromhex(m.group(3)) if m.group(3) else b''
            out.append((ts, cmd, data))
    return out


def extract_series(frames, label_vel_sign):
    """Build a time-aligned series. Returns dict with lists of (t, val) per
    signal, plus the identified motion-start timestamp. Motion phase starts
    on first nonzero Set_Input_Vel with matching sign; ends on ANY of:
      - Set_Input_Vel goes back to zero or flips sign (vel-mode retract or disarm)
      - Set_Controller_Mode (entry to position-mode retract)
      - Set_Axis_State with state=IDLE (explicit disarm)
    Only samples during the motion phase are collected - this avoids mixing
    disarm/retract noise into the foam-signature analysis.
    """
    series = {'pos': [], 'vel': [], 'iq_meas': [], 'vel_cmd': []}
    motion_t0 = None
    vel_cmd_now = 0.0
    motion_started = False
    motion_ended = False

    for ts, cmd, data in frames:
        if motion_ended:
            break

        if cmd == CMD_SET_INPUT_VEL and len(data) >= 4:
            v, = struct.unpack_from('<f', data, 0)
            if not motion_started:
                if v != 0.0 and (v > 0) == (label_vel_sign > 0):
                    motion_started = True
                    motion_t0 = ts
                    vel_cmd_now = v
            else:
                if v == 0.0 or (v > 0) != (label_vel_sign > 0):
                    motion_ended = True
                else:
                    vel_cmd_now = v
            continue

        if cmd == CMD_SET_CONTROLLER_MODE and motion_started:
            motion_ended = True
            continue

        if cmd == CMD_SET_AXIS_STATE and len(data) >= 4 and motion_started:
            state, = struct.unpack_from('<I', data, 0)
            if state == STATE_IDLE:
                motion_ended = True
            continue

        if not motion_started:
            continue

        if cmd == CMD_GET_ENCODER and len(data) >= 8:
            p, vl = struct.unpack('<ff', data[:8])
            series['pos'].append((ts, p))
            series['vel'].append((ts, vl))
            series['vel_cmd'].append((ts, vel_cmd_now))
        elif cmd == CMD_GET_IQ and len(data) >= 8:
            _, iq_m = struct.unpack('<ff', data[:8])
            series['iq_meas'].append((ts, iq_m))

    return series, motion_t0


def segment_by_contact(series, motion_t0, foam_contact, contact_t):
    """Split series into pre-contact and post-contact windows using the user's
    contact_t (relative to motion start). Returns (pre_rows, post_rows) where
    each row is a (t_relative, pos, vel, vel_cmd, iq_meas) tuple or None if
    no aligned sample."""
    # Align the four signals by nearest-neighbor in time, walking pos as spine
    # (it's the densest broadcast). Keep only samples with all fields present.
    aligned = []
    ivel = iiq = icmd = 0
    for t_pos, p in series['pos']:
        while ivel + 1 < len(series['vel']) and series['vel'][ivel + 1][0] <= t_pos:
            ivel += 1
        while iiq + 1 < len(series['iq_meas']) and series['iq_meas'][iiq + 1][0] <= t_pos:
            iiq += 1
        while icmd + 1 < len(series['vel_cmd']) and series['vel_cmd'][icmd + 1][0] <= t_pos:
            icmd += 1
        if (ivel < len(series['vel']) and iiq < len(series['iq_meas'])
                and icmd < len(series['vel_cmd'])):
            _, vl = series['vel'][ivel]
            _, iq = series['iq_meas'][iiq]
            _, vc = series['vel_cmd'][icmd]
            aligned.append((t_pos - motion_t0, p, vl, vc, iq))

    if foam_contact != 'yes':
        return aligned, []  # all non-foam
    if contact_t is None or contact_t <= 0.05:
        return [], aligned  # all foam
    pre = [r for r in aligned if r[0] < contact_t]
    post = [r for r in aligned if r[0] >= contact_t]
    return pre, post


def _rolling_mean_time(times, values, window_s):
    """Time-based rolling mean. For each sample, average all samples within
    the preceding window_s seconds. Uses a deque-style sliding lower bound."""
    out = []
    lo = 0
    run_sum = 0.0
    for i in range(len(values)):
        t_hi = times[i]
        run_sum += values[i]
        while lo < i and (t_hi - times[lo]) > window_s:
            run_sum -= values[lo]
            lo += 1
        out.append(run_sum / (i - lo + 1))
    return out


def compute_features(rows):
    """rows: list of (t, pos, vel, vel_cmd, iq_meas). Returns feature dict."""
    if len(rows) < 10:
        return None
    times = [r[0] for r in rows]
    iq = [abs(r[4]) for r in rows]
    vel = [abs(r[2]) for r in rows]
    vel_cmd = [abs(r[3]) for r in rows]
    err = [abs(r[3] - r[2]) for r in rows]
    iv_raw = [abs(r[4]) / max(abs(r[2]), 0.02) for r in rows]

    # Rolling-window features: smooth the iq/vel ratio over 300 ms, then look
    # for peak and rise-rate. The motivation is that foam contact should
    # produce a localized SPIKE from baseline (detectable as peak or rise
    # rate), whereas mean-over-entire-window gets diluted by any free-motion
    # phase preceding the contact.
    iv_roll = _rolling_mean_time(times, iv_raw, window_s=0.3)
    iq_roll = _rolling_mean_time(times, iq, window_s=0.3)

    # Max rise rate of rolling iq/vel: foam contact = sudden rise.
    rise_rate = 0.0
    for i in range(1, len(iv_roll)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        r = (iv_roll[i] - iv_roll[i - 1]) / dt
        if r > rise_rate:
            rise_rate = r

    # Peak-to-baseline: difference between rolling-max and rolling-min.
    # Gradually-rising friction has small peak-minus-min; foam-contact spike
    # should have large peak-minus-min within the same window.
    iv_peak_to_min = max(iv_roll) - min(iv_roll)

    return {
        'n': len(rows),
        'iq_mean': statistics.mean(iq),
        'iq_std': statistics.stdev(iq) if len(iq) > 1 else 0.0,
        'iq_max': max(iq),
        'iq_p95': sorted(iq)[int(len(iq) * 0.95)],
        'iq_median': statistics.median(iq),
        'vel_mean': statistics.mean(vel),
        'vel_std': statistics.stdev(vel) if len(vel) > 1 else 0.0,
        'tracking_err_mean': statistics.mean(err),
        'vel_cmd_mean': statistics.mean(vel_cmd),
        'iq_over_vel': statistics.mean(iv_raw),
        # Time-localized features
        'iv_roll_max': max(iv_roll),
        'iv_roll_std': statistics.stdev(iv_roll) if len(iv_roll) > 1 else 0.0,
        'iv_rise_rate_max': rise_rate,
        'iv_peak_to_min': iv_peak_to_min,
        'iq_roll_max': max(iq_roll),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('logs_dir', help='directory containing .canlog and .label.json files')
    p.add_argument('--node', type=int, default=0)
    p.add_argument('--csv', help='optional per-window CSV output')
    args = p.parse_args()

    labels = sorted(glob.glob(os.path.join(args.logs_dir, '*.label.json')))
    if not labels:
        print(f"no .label.json files under {args.logs_dir}", file=sys.stderr)
        return 1

    per_window_rows = []   # (class, run, features)
    skipped = []

    for label_path in labels:
        stem = label_path.replace('.label.json', '')
        canlog = stem + '.canlog'
        if not os.path.exists(canlog):
            skipped.append((label_path, 'no matching canlog'))
            continue
        with open(label_path) as f:
            lbl = json.load(f)
        vel_cmd_cfg = lbl.get('vel_cmd')
        if vel_cmd_cfg is None or vel_cmd_cfg == 0:
            skipped.append((label_path, 'label missing vel_cmd'))
            continue
        frames = parse_log(canlog, node_id=args.node)
        series, motion_t0 = extract_series(frames, label_vel_sign=vel_cmd_cfg)
        if motion_t0 is None:
            skipped.append((label_path, 'no motion detected'))
            continue
        pre, post = segment_by_contact(
            series, motion_t0,
            foam_contact=lbl.get('foam_contact', 'no'),
            contact_t=lbl.get('foam_contact_time_s'),
        )
        for cls, rows in (('no_foam', pre), ('foam', post)):
            feats = compute_features(rows)
            if feats is None:
                continue
            feats['run'] = os.path.basename(stem)
            feats['class'] = cls
            feats['direction'] = 'down' if vel_cmd_cfg > 0 else 'up'
            per_window_rows.append(feats)

    print(f"parsed {len(per_window_rows)} windows from {len(labels)} runs "
          f"({len(skipped)} skipped)")
    if skipped:
        for p, why in skipped[:5]:
            print(f"  skipped {os.path.basename(p)}: {why}")

    # Aggregate by class
    def group(cls, direction=None):
        return [r for r in per_window_rows
                if r['class'] == cls and (direction is None or r['direction'] == direction)]

    feature_keys = ['iq_mean', 'iq_std', 'iq_max', 'iq_p95', 'iq_median',
                    'vel_mean', 'vel_std', 'tracking_err_mean', 'vel_cmd_mean',
                    'iq_over_vel',
                    'iv_roll_max', 'iv_roll_std', 'iv_rise_rate_max',
                    'iv_peak_to_min', 'iq_roll_max']

    def summary(rows, key):
        if not rows:
            return None
        vals = [r[key] for r in rows]
        m = statistics.mean(vals)
        s = statistics.stdev(vals) if len(vals) > 1 else 0.0
        return m, s, min(vals), max(vals), len(vals)

    def separability(a, b):
        """|mean_a - mean_b| / (std_a + std_b + eps). Higher = more separable."""
        ma, sa, *_ = a
        mb, sb, *_ = b
        return abs(ma - mb) / (sa + sb + 1e-6)

    print()
    print("=" * 90)
    print("Feature comparison: NO_FOAM vs FOAM windows (all directions combined)")
    print("=" * 90)
    print(f"{'feature':<20} {'no_foam_mean±std':<24} {'foam_mean±std':<24} "
          f"{'separability':<12}")
    no_foam = group('no_foam')
    foam = group('foam')
    print(f"  (counts: {len(no_foam)} no-foam windows, {len(foam)} foam windows)")
    print()
    ranked = []
    for k in feature_keys:
        a = summary(no_foam, k)
        b = summary(foam, k)
        if a is None or b is None:
            continue
        sep = separability(a, b)
        ranked.append((sep, k, a, b))
    ranked.sort(reverse=True)
    for sep, k, a, b in ranked:
        ma, sa, amin, amax, an = a
        mb, sb, bmin, bmax, bn = b
        print(f"{k:<20} {ma:>7.3f} ± {sa:<6.3f}         "
              f"{mb:>7.3f} ± {sb:<6.3f}         {sep:>6.2f}")

    # Per direction slice
    for dir_ in ('down', 'up'):
        print()
        print("-" * 90)
        print(f"Same comparison, {dir_.upper()} only:")
        print("-" * 90)
        nf = group('no_foam', dir_)
        f = group('foam', dir_)
        print(f"  (counts: {len(nf)} no-foam, {len(f)} foam)")
        for k in feature_keys:
            a = summary(nf, k)
            b = summary(f, k)
            if a is None or b is None:
                continue
            sep = separability(a, b)
            print(f"{k:<20} no_foam={a[0]:>6.3f}±{a[1]:<5.3f}  "
                  f"foam={b[0]:>6.3f}±{b[1]:<5.3f}  sep={sep:.2f}")

    if args.csv:
        import csv as csvmod
        with open(args.csv, 'w', newline='') as f:
            w = csvmod.DictWriter(f, fieldnames=['run', 'class', 'direction'] + feature_keys + ['n'])
            w.writeheader()
            w.writerows(per_window_rows)
        print(f"\nwrote per-window CSV to {args.csv}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
