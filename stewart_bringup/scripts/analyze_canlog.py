#!/usr/bin/env python3
"""
Parse a candump -L log and plot/export the ODrive-relevant time series:
position, velocity, commanded velocity, Iq (measured and setpoint),
heartbeat state. Intended for offline analysis of stall_home / spin test logs.

Usage:
  python3 analyze_canlog.py PATH.canlog               # show plot
  python3 analyze_canlog.py PATH.canlog --save out.png
  python3 analyze_canlog.py PATH.canlog --csv out.csv
  python3 analyze_canlog.py PATH.canlog --node 2      # other axis
"""
import argparse
import csv
import re
import struct
import sys

CMD_HEARTBEAT      = 0x001
CMD_GET_ENCODER    = 0x009
CMD_SET_INPUT_VEL  = 0x00D
CMD_GET_IQ         = 0x014

LINE_RE = re.compile(r'\(([\d.]+)\)\s+\S+\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)')


def parse_log(path, node_id):
    samples = []
    with open(path) as f:
        for line in f:
            m = LINE_RE.match(line.strip())
            if not m:
                continue
            ts, arb_hex, data_hex = m.groups()
            arb = int(arb_hex, 16)
            if (arb >> 5) != node_id:
                continue
            cmd = arb & 0x1F
            data = bytes.fromhex(data_hex) if data_hex else b''
            samples.append((float(ts), cmd, data))
    return samples


def build_rows(samples):
    """Build a list of rows. Each incoming frame updates a rolling state
    snapshot; we emit one row per frame so plots show transitions precisely."""
    state = dict(t=None, pos=None, vel=None, vel_cmd=None,
                 iq_meas=None, iq_set=None, axis_state=None, active_err=None)
    rows = []
    for ts, cmd, data in samples:
        updated = True
        if cmd == CMD_GET_ENCODER and len(data) >= 8:
            state['pos'], state['vel'] = struct.unpack('<ff', data[:8])
        elif cmd == CMD_GET_IQ and len(data) >= 8:
            state['iq_set'], state['iq_meas'] = struct.unpack('<ff', data[:8])
        elif cmd == CMD_SET_INPUT_VEL and len(data) >= 8:
            state['vel_cmd'], _ = struct.unpack('<ff', data[:8])
        elif cmd == CMD_HEARTBEAT and len(data) >= 8:
            state['active_err'] = struct.unpack_from('<I', data, 0)[0]
            state['axis_state'] = data[4]
        else:
            updated = False
        if updated:
            state['t'] = ts
            rows.append(dict(state))
    return rows


def to_csv(rows, path):
    keys = ['t', 'pos', 'vel', 'vel_cmd', 'iq_meas', 'iq_set',
            'axis_state', 'active_err']
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} rows to {path}")


def plot(rows, save_path=None):
    import matplotlib
    if save_path:
        matplotlib.use('Agg')
    else:
        # Try an interactive backend; fall back to Agg+save if none work.
        try:
            matplotlib.use('TkAgg')
        except Exception:
            matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    if not rows:
        print("no rows to plot", file=sys.stderr)
        return

    t0 = rows[0]['t']
    t = [r['t'] - t0 for r in rows]

    def col(key):
        return [r[key] if r[key] is not None else float('nan') for r in rows]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 9))

    axes[0].plot(t, col('pos'), color='steelblue')
    axes[0].set_ylabel('position (turns)')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('ODrive per-axis time series')

    axes[1].plot(t, col('vel'), color='darkgreen', label='actual', alpha=0.8)
    axes[1].plot(t, col('vel_cmd'), color='red', label='commanded',
                 alpha=0.6, linewidth=1)
    axes[1].set_ylabel('velocity (turns/s)')
    axes[1].legend(loc='upper left')
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, col('iq_meas'), color='darkorange', label='Iq measured')
    axes[2].plot(t, col('iq_set'), color='purple', label='Iq setpoint',
                 alpha=0.6, linewidth=1)
    axes[2].set_ylabel('current (A)')
    axes[2].set_xlabel('time (s)')
    axes[2].legend(loc='upper left')
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(0, color='black', linewidth=0.5, alpha=0.5)

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=120)
        print(f"saved plot to {save_path}")
    else:
        plt.show()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('canlog', help='path to .canlog file from start_*_*.sh')
    p.add_argument('--node', type=int, default=0, help='axis node ID (default 0)')
    p.add_argument('--csv', help='write CSV to this path')
    p.add_argument('--save', help='save plot to PNG at this path (non-interactive)')
    p.add_argument('--no-plot', action='store_true', help='skip plotting entirely')
    args = p.parse_args()

    samples = parse_log(args.canlog, args.node)
    rows = build_rows(samples)
    print(f"parsed {len(samples)} frames for node {args.node}, "
          f"{len(rows)} rows ({rows[-1]['t'] - rows[0]['t']:.2f} s span)" if rows
          else f"parsed {len(samples)} frames; no rows")

    if args.csv:
        to_csv(rows, args.csv)
    if not args.no_plot and rows:
        plot(rows, args.save)


if __name__ == '__main__':
    main()
