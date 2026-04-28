#!/usr/bin/env python3
"""
set_odrive_feedforward_via_can.py — configure ODrive feedforward on
all 6 drives in one shot, purely over CAN. No USB cables.

Reads endpoint IDs from stewart_bringup/data/odrive_endpoints.json
(generated once by dump_odrive_endpoints.py from a USB connection)
and writes the parameters via the ODrive CAN SDO protocol:

  RxSdo (we send, arb_id = (node_id << 5) | 0x04):
    byte 0     : opcode (0x00=read, 0x01=write)
    bytes 1-2  : endpoint_id (uint16 LE)
    byte 3     : reserved (0x00)
    bytes 4-7  : value (typed; e.g. <f for float, single byte for bool)

  TxSdo (drive responds, arb_id = (node_id << 5) | 0x05): same prologue,
    plus the read response value.

Per ODrive's docs, function endpoints (e.g. save_configuration) are
invoked by writing to them — we use opcode 0x01 with no value.

Usage:
  sudo systemctl stop stable_bot   # release can0
  python3 set_odrive_feedforward_via_can.py [--apply | --revert | --status]
  sudo systemctl start stable_bot

Modes:
  --status (default if nothing else): SDO-read each parameter on each
           node, print current values
  --apply : back up current values, write new values, save_configuration
  --revert: restore from backup, save_configuration

The backup file at ~/.stable_bot_odrive_ff_backup.json is keyed by
CAN node id (since serial_number isn't easily readable over CAN
without a separate exchange).
"""
import argparse
import json
import os
import struct
import sys
import time
from pathlib import Path

try:
    import can
except ImportError:
    sys.exit("ERROR: python-can not installed.\n"
             "  pip3 install --user --break-system-packages python-can")


# CAN command IDs from ODrive's protocol.
CMD_RX_SDO = 0x04   # we send (write or read request)
CMD_TX_SDO = 0x05   # drive responds
OPCODE_READ  = 0x00
OPCODE_WRITE = 0x01

DEFAULT_TARGETS = [
    # (endpoint name in JSON, value to write on --apply, type-cast)
    # 2026-04-28 step 3: wL_FF_enable disabled. Step 2 (FF on, default
    # integrator gain) showed mixed Z-dependent benefit — Z=40/55 were
    # better than iter-3 pre-fix, but Z=35/50 were 2-4× worse. wL_FF's
    # contribution is non-monotonic in Z, which means it's the wrong
    # tradeoff at any single tuning point. Dropping it returns the
    # inner loop to ODrive factory defaults; from there we can use
    # high-rate capture to diagnose residual phase-lag issues without
    # FF as a confound.
    ('axis0.controller.config.vel_integrator_gain', 0.333, float),
    ('axis0.config.motor.wL_FF_enable',             False, bool),
]
SAVE_ENDPOINT = 'save_configuration'

# Where dump_odrive_endpoints.py wrote the IDs.
def _default_endpoint_json():
    here = Path(__file__).resolve()
    repo = here.parent.parent.parent
    cand = repo / 'stewart_bringup' / 'data' / 'odrive_endpoints.json'
    if cand.exists():
        return cand
    # Fall back to anywhere in the cwd
    return Path.cwd() / 'odrive_endpoints.json'

BACKUP_PATH = Path.home() / '.stable_bot_odrive_ff_backup.json'


def _pack_value(value, kind):
    """Pack a Python value into bytes for the SDO write payload, per
    the endpoint's type. The full SDO payload is:
       struct.pack('<BHB', OPCODE, endpoint_id, reserved) + value_bytes
    """
    if kind == 'float32':
        return struct.pack('<f', float(value))
    if kind == 'float64':
        return struct.pack('<d', float(value))
    if kind == 'bool':
        return struct.pack('<B', 1 if value else 0)
    if kind in ('uint8',):
        return struct.pack('<B', int(value))
    if kind in ('uint16',):
        return struct.pack('<H', int(value))
    if kind in ('uint32',):
        return struct.pack('<I', int(value))
    if kind in ('int8',):
        return struct.pack('<b', int(value))
    if kind in ('int16',):
        return struct.pack('<h', int(value))
    if kind in ('int32',):
        return struct.pack('<i', int(value))
    if kind == 'function':
        return b''   # function call: write with empty payload
    raise ValueError(f"unsupported endpoint type: {kind}")


def _unpack_value(data, kind):
    """Unpack a typed value from a TxSdo payload (the bytes after the
    4-byte prologue). Returns the value or None if data is too short."""
    if kind == 'float32':
        return struct.unpack('<f', data[:4])[0] if len(data) >= 4 else None
    if kind == 'float64':
        return struct.unpack('<d', data[:8])[0] if len(data) >= 8 else None
    if kind == 'bool':
        return bool(data[0]) if len(data) >= 1 else None
    if kind in ('uint8', 'int8'):
        return data[0] if len(data) >= 1 else None
    if kind in ('uint16',):
        return struct.unpack('<H', data[:2])[0] if len(data) >= 2 else None
    if kind in ('uint32',):
        return struct.unpack('<I', data[:4])[0] if len(data) >= 4 else None
    if kind in ('int16',):
        return struct.unpack('<h', data[:2])[0] if len(data) >= 2 else None
    if kind in ('int32',):
        return struct.unpack('<i', data[:4])[0] if len(data) >= 4 else None
    return None


class SdoClient:
    """Minimal Tx/Rx_SDO client over python-can on a socketcan bus."""

    def __init__(self, channel='can0', bitrate=1_000_000):
        self.bus = can.interface.Bus(channel=channel, interface='socketcan')

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _send_rx_sdo(self, node_id, opcode, endpoint_id, value_bytes=b''):
        arb = (node_id << 5) | CMD_RX_SDO
        prologue = struct.pack('<BHB', opcode, endpoint_id, 0)
        data = prologue + value_bytes
        # CAN classic frames are at most 8 bytes. SDOs that fit in
        # the 4 prologue + ≤4 value bytes are single-frame.
        if len(data) > 8:
            raise ValueError(f"SDO payload too long: {len(data)} bytes")
        # Pad to the minimum (some adapters dislike sub-8-byte writes)
        if len(data) < 8:
            data = data + b'\x00' * (8 - len(data))
        msg = can.Message(arbitration_id=arb, data=data,
                          is_extended_id=False)
        self.bus.send(msg, timeout=0.2)

    def _recv_tx_sdo(self, node_id, expected_endpoint_id, timeout=1.0):
        """Wait for a TxSdo response matching node_id + endpoint_id.
        Returns the payload bytes after the 4-byte prologue, or None."""
        deadline = time.monotonic() + timeout
        want_arb = (node_id << 5) | CMD_TX_SDO
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            msg = self.bus.recv(timeout=remaining)
            if msg is None:
                return None
            if msg.arbitration_id != want_arb or msg.dlc < 4:
                continue
            opcode, ep_id, _reserved = struct.unpack('<BHB', msg.data[:4])
            if ep_id != expected_endpoint_id:
                continue
            return bytes(msg.data[4:])
        return None

    def read(self, node_id, endpoint_id, kind, timeout=1.0):
        # Drain any stale frames before the read so the response we
        # match isn't a leftover.
        self._drain()
        self._send_rx_sdo(node_id, OPCODE_READ, endpoint_id)
        data = self._recv_tx_sdo(node_id, endpoint_id, timeout=timeout)
        if data is None:
            return None
        return _unpack_value(data, kind)

    def write(self, node_id, endpoint_id, value, kind):
        payload = _pack_value(value, kind)
        self._send_rx_sdo(node_id, OPCODE_WRITE, endpoint_id, payload)
        # writes have no response

    def call_function(self, node_id, endpoint_id):
        """Trigger a function endpoint (no value)."""
        self._send_rx_sdo(node_id, OPCODE_WRITE, endpoint_id, b'')

    def _drain(self, max_drained=64):
        """Eat any pending frames so a fresh read isn't matched against
        an old leftover."""
        for _ in range(max_drained):
            m = self.bus.recv(timeout=0.0)
            if m is None:
                return


def load_endpoints(path):
    if not path.exists():
        sys.exit(
            f"endpoint JSON not found: {path}\n"
            "Run dump_odrive_endpoints.py first (USB plug-in to ONE drive, "
            "one-time per firmware version) and commit the result.")
    with open(path) as f:
        return json.load(f)


def _resolve_targets(eps, targets):
    """Map (path, value, cast) targets to (endpoint_id, type, value)."""
    out = []
    missing = []
    for path, val, cast in targets:
        ent = eps['endpoints'].get(path)
        if ent is None:
            missing.append(path)
            continue
        out.append((path, ent['id'], ent['type'], cast(val)))
    if missing:
        sys.exit(f"endpoint(s) missing from JSON: {missing}\n"
                 "Re-run dump_odrive_endpoints.py and commit the new JSON.")
    return out


def _resolve_save(eps):
    ent = eps['endpoints'].get(SAVE_ENDPOINT)
    if ent is None:
        sys.exit(f"endpoint missing: {SAVE_ENDPOINT}")
    return ent['id']


def cmd_status(client, eps, node_ids):
    targets = _resolve_targets(eps, DEFAULT_TARGETS)
    print(f"reading current values from nodes {list(node_ids)}...")
    for n in node_ids:
        print(f"\nnode {n}:")
        for path, ep_id, kind, _new in targets:
            val = client.read(n, ep_id, kind, timeout=0.8)
            if val is None:
                print(f"  {path}: (no response — node alive on bus?)")
            else:
                print(f"  {path}: {val}")


def cmd_apply(client, eps, node_ids):
    targets = _resolve_targets(eps, DEFAULT_TARGETS)
    save_id = _resolve_save(eps)

    # Read previous values for backup BEFORE writing anything.
    print("backing up current values...")
    if BACKUP_PATH.exists():
        try:
            with open(BACKUP_PATH) as f:
                backup = json.load(f)
        except Exception:
            backup = {}
    else:
        backup = {}
    for n in node_ids:
        per_node = backup.setdefault(str(n), {})
        for path, ep_id, kind, _new in targets:
            cur = client.read(n, ep_id, kind, timeout=0.8)
            if cur is None:
                print(f"  WARN: node {n} {path}: no response; skipping backup")
                continue
            if path not in per_node:
                per_node[path] = cur
                per_node[f'{path}__type'] = kind
        per_node['captured_unix'] = time.time()
    with open(BACKUP_PATH, 'w') as f:
        json.dump(backup, f, indent=2)
    print(f"  backup → {BACKUP_PATH}")

    print("\nwriting new values...")
    for n in node_ids:
        print(f"node {n}:")
        for path, ep_id, kind, new_val in targets:
            client.write(n, ep_id, new_val, kind)
            print(f"  ✓ wrote {path} = {new_val}")
        # Verify by reading back (skip booleans? read-back always works)
        time.sleep(0.05)
        for path, ep_id, kind, new_val in targets:
            verify = client.read(n, ep_id, kind, timeout=0.8)
            if verify is None:
                print(f"  ? could not verify {path}")
            else:
                ok = (verify == new_val) or (
                    isinstance(new_val, float)
                    and abs(verify - new_val) < 1e-6)
                print(f"  {'✓' if ok else '✗'} verify {path}: {verify}")
        # Save configuration. The drive will reboot.
        client.call_function(n, save_id)
        print(f"  ✓ save_configuration() — node will reboot, waiting 3s")
        time.sleep(3.0)
    print("\nDone.")


def cmd_revert(client, eps, node_ids):
    save_id = _resolve_save(eps)
    if not BACKUP_PATH.exists():
        sys.exit(f"no backup found at {BACKUP_PATH}")
    with open(BACKUP_PATH) as f:
        backup = json.load(f)

    for n in node_ids:
        per = backup.get(str(n))
        if per is None:
            print(f"node {n}: no backup, skipping")
            continue
        print(f"node {n}: reverting")
        for path, _val, _cast in DEFAULT_TARGETS:
            if path not in per:
                continue
            ep_id = eps['endpoints'][path]['id']
            kind = per.get(f'{path}__type') or eps['endpoints'][path]['type']
            old = per[path]
            client.write(n, ep_id, old, kind)
            print(f"  ✓ wrote {path} = {old}")
        client.call_function(n, save_id)
        print(f"  ✓ save_configuration()")
        time.sleep(3.0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--status', action='store_true',
                   help='read + print current values, no changes (default)')
    g.add_argument('--apply', action='store_true',
                   help='write new values + save_configuration')
    g.add_argument('--revert', action='store_true',
                   help='restore previous values from backup')
    p.add_argument('--channel', default='can0',
                   help='socketcan interface (default: can0)')
    p.add_argument('--node-ids', type=lambda s: [int(x) for x in s.split(',')],
                   default=[0, 1, 2, 3, 4, 5],
                   help='comma-separated CAN node IDs (default: 0,1,2,3,4,5)')
    p.add_argument('--endpoints',
                   help=f'path to endpoints JSON (default: '
                        f'<repo>/stewart_bringup/data/odrive_endpoints.json)')
    args = p.parse_args()

    if not (args.status or args.apply or args.revert):
        args.status = True

    eps_path = Path(args.endpoints).expanduser().resolve() if args.endpoints \
        else _default_endpoint_json()
    eps = load_endpoints(eps_path)
    print(f"using endpoints from {eps_path}  (fw={eps.get('firmware_version')})")

    client = SdoClient(channel=args.channel)
    try:
        if args.apply:
            cmd_apply(client, eps, args.node_ids)
        elif args.revert:
            cmd_revert(client, eps, args.node_ids)
        else:
            cmd_status(client, eps, args.node_ids)
    finally:
        client.close()


if __name__ == '__main__':
    main()
