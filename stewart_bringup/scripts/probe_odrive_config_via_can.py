#!/usr/bin/env python3
"""
probe_odrive_config_via_can.py — find which endpoints differ between
two ODrive Pros on the CAN bus. Pure CAN, no USB, no library.

Use case (the one this was written for): nodes 2 and 5 on this bus
are silently overwriting writes to vel_integrator_gain — some config
on those drives is enabling drive-side autotune / gain scheduling.
This script enumerates SDO endpoints by ID across two nodes, compares
the raw 4-byte response payloads, and reports any endpoint where
they differ. Whichever endpoint is responsible for the autotune-like
behaviour will show up in the diff.

How it works:
  - For each endpoint_id in [start, end), send Tx_SDO read to "good"
    node and "bad" node, wait for the matching Rx_SDO response.
  - Record the 4-byte response payload.
  - At the end: print every endpoint where the two payloads differ,
    interpreting the bytes multiple ways (float32 / int32 / uint32 /
    bool) since we don't know the type from the protocol alone.

For endpoints we already named in stewart_bringup/data/odrive_endpoints.json,
the diff line shows the path. Unknown IDs just get the integer.

Usage:
  sudo systemctl stop stable_bot
  python3 probe_odrive_config_via_can.py
  # default: good=node 0, bad=node 2, scan 0..600

  # smaller window, more verbose:
  python3 probe_odrive_config_via_can.py --good-node 0 --bad-node 2 \
      --start 200 --end 500 --verbose

  # compare three drives at once:
  python3 probe_odrive_config_via_can.py --nodes 0,2,5 --end 600

Wall-clock: ~50 ms per (node, endpoint) read. Scanning 0..600 across
2 nodes → ~60 s. Bumping to all 6 nodes triples that.
"""
import argparse
import json
import struct
import sys
import time
from pathlib import Path

try:
    import can
except ImportError:
    sys.exit("python-can not installed.\n"
             "  pip3 install --user --break-system-packages python-can")

CMD_RX_SDO = 0x04
CMD_TX_SDO = 0x05
OPCODE_READ = 0x00


class SdoReader:
    def __init__(self, channel='can0'):
        self.bus = can.interface.Bus(channel=channel, interface='socketcan')

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _drain(self, max_drain=64):
        for _ in range(max_drain):
            m = self.bus.recv(timeout=0.0)
            if m is None:
                return

    def read_raw(self, node_id, ep_id, timeout=0.10):
        """Return raw 4-byte response payload, or None on timeout."""
        self._drain()
        arb_send = (node_id << 5) | CMD_RX_SDO
        prologue = struct.pack('<BHB', OPCODE_READ, ep_id, 0)
        data = prologue + b'\x00' * 4
        msg = can.Message(arbitration_id=arb_send, data=data,
                          is_extended_id=False)
        try:
            self.bus.send(msg, timeout=0.2)
        except Exception:
            return None

        want_arb = (node_id << 5) | CMD_TX_SDO
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self.bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if m is None:
                return None
            if m.arbitration_id != want_arb or m.dlc < 4:
                continue
            opcode, resp_ep, _reserved = struct.unpack('<BHB', m.data[:4])
            if resp_ep != ep_id:
                continue
            return bytes(m.data[4:8]) if m.dlc >= 8 else bytes(m.data[4:])
        return None


def interpret_bytes(b):
    """Try multiple type interpretations so a human can pattern-match
    what kind of value this endpoint holds."""
    if b is None:
        return "<no response>"
    if len(b) < 4:
        return f"<short: {b.hex()}>"
    f32 = struct.unpack('<f', b[:4])[0]
    i32 = struct.unpack('<i', b[:4])[0]
    u32 = struct.unpack('<I', b[:4])[0]
    bool_lo = bool(b[0])
    parts = [
        f"hex={b.hex()}",
        f"f32={f32:.4g}",
    ]
    # Only show int interpretations if they're plausible-looking
    if u32 < 100000:
        parts.append(f"u32={u32}")
    if -1000 <= i32 <= 1000:
        parts.append(f"i32={i32}")
    if u32 in (0, 1):
        parts.append(f"bool={bool_lo}")
    return "  ".join(parts)


def load_known_names(path):
    """Return {endpoint_id: (path_name, type_tag)} from
    odrive_endpoints.json if it exists."""
    p = Path(path).expanduser()
    if not p.exists():
        return {}
    try:
        data = json.loads(p.read_text())
    except Exception:
        return {}
    out = {}
    for name, ent in (data.get('endpoints') or {}).items():
        out[int(ent['id'])] = (name, ent.get('type', '?'))
    return out


def _default_known_path():
    here = Path(__file__).resolve()
    cand = here.parent.parent / 'data' / 'odrive_endpoints.json'
    return cand if cand.exists() else here.parent.parent.parent / 'stewart_bringup' / 'data' / 'odrive_endpoints.json'


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--nodes', type=lambda s: [int(x) for x in s.split(',')],
                   default=None,
                   help='comma-separated node IDs to compare (default: '
                        'good-node + bad-node)')
    p.add_argument('--good-node', type=int, default=0)
    p.add_argument('--bad-node', type=int, default=2)
    p.add_argument('--start', type=int, default=0,
                   help='first endpoint id to probe (default 0)')
    p.add_argument('--end', type=int, default=600,
                   help='last endpoint id to probe, exclusive (default 600)')
    p.add_argument('--channel', default='can0')
    p.add_argument('--timeout', type=float, default=0.08,
                   help='per-read timeout (s); raise if drives are slow')
    p.add_argument('--known',
                   help='path to odrive_endpoints.json for naming '
                        '(default: stewart_bringup/data/odrive_endpoints.json)')
    p.add_argument('--verbose', '-v', action='store_true',
                   help='also print per-endpoint values, not just diffs')
    p.add_argument('--show-only-on-one', action='store_true',
                   help='also list endpoints that respond on only one node')
    args = p.parse_args()

    nodes = args.nodes if args.nodes else [args.good_node, args.bad_node]
    if len(nodes) < 2:
        sys.exit("need at least 2 nodes to compare")

    known_path = Path(args.known).expanduser() if args.known else _default_known_path()
    known = load_known_names(known_path)
    print(f"using known endpoint names from {known_path}  ({len(known)} entries)")
    print(f"comparing nodes: {nodes}  range: [{args.start}, {args.end})")

    rdr = SdoReader(channel=args.channel)
    try:
        # values[ep_id] = [raw_bytes_per_node]
        values = {}
        responsive_count = {n: 0 for n in nodes}
        last_progress = time.monotonic()
        for ep in range(args.start, args.end):
            row = []
            for n in nodes:
                v = rdr.read_raw(n, ep, timeout=args.timeout)
                row.append(v)
                if v is not None:
                    responsive_count[n] += 1
            values[ep] = row
            now = time.monotonic()
            if now - last_progress > 5.0:
                done = ep - args.start + 1
                total = args.end - args.start
                print(f"  ...scanned {done}/{total}", flush=True)
                last_progress = now
    finally:
        rdr.close()

    print(f"\nresponsive endpoints per node: {responsive_count}")

    diffs = []
    only_one = []
    for ep, row in values.items():
        nonnone = [v for v in row if v is not None]
        if not nonnone:
            continue
        if len(set(map(bytes, nonnone))) == 1 and len(nonnone) == len(row):
            continue   # all responded with the same value
        if any(v is None for v in row):
            only_one.append((ep, row))
        if len(set(map(bytes, nonnone))) > 1:
            diffs.append((ep, row))

    print(f"\n=== ENDPOINTS WITH DIFFERENT VALUES (n={len(diffs)}) ===")
    for ep, row in diffs:
        name, kind = known.get(ep, (f'<id {ep}>', '?'))
        print(f"\nendpoint {ep}  [{name}]  type={kind}")
        for n, v in zip(nodes, row):
            print(f"  node {n}: {interpret_bytes(v)}")

    if args.show_only_on_one:
        print(f"\n=== ENDPOINTS RESPONDING ON ONLY SOME NODES (n={len(only_one)}) ===")
        for ep, row in only_one:
            name, _ = known.get(ep, (f'<id {ep}>', '?'))
            present = [n for n, v in zip(nodes, row) if v is not None]
            absent = [n for n, v in zip(nodes, row) if v is None]
            print(f"  ep {ep}  [{name}]  on nodes {present}, missing {absent}")

    if args.verbose:
        print(f"\n=== ALL RESPONSIVE ENDPOINTS ===")
        for ep, row in sorted(values.items()):
            if all(v is None for v in row):
                continue
            name, _ = known.get(ep, (f'<id {ep}>', '?'))
            same = (len({bytes(v) for v in row if v is not None}) == 1)
            tag = '=' if same else '!'
            print(f"  {tag} ep {ep}  [{name}]  " +
                  "  ".join(f"n{n}={(v[:4].hex() if v else 'nil')}"
                            for n, v in zip(nodes, row)))


if __name__ == '__main__':
    main()
