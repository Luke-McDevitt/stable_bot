#!/usr/bin/env python3
"""calibrate_via_can.py — re-run an ODrive Pro motor+encoder calibration on
ONE leg over CAN and persist it, without touching any other configuration.

After reassembling an actuator the motor/encoder commutation offset is lost
(the motor "doesn't rotate as commanded"). This fires the standard
FULL_CALIBRATION_SEQUENCE (axis state 3) — which re-measures phase
resistance/inductance and the encoder offset — polls the heartbeat until it
finishes, checks the result, and only then calls save_configuration().

It writes NO config of its own. node_id, CAN settings, vel/current limits,
control/input mode and gains are left exactly as they booted from flash;
save_configuration() simply persists the fresh calibration alongside the
unchanged config. (Stop the stack first so the RAM config == flash config and
nothing else is on the bus.) On fw 0.6.11 the calibration sets the
phase_*_valid / encoder-offset flags itself, and startup_*_calibration is
already false, so calibrate + save is all that's needed — no pre_calibrated
write.

SAFETY — calibration ROTATES the motor (the encoder-offset lockin). On a
lead-screw leg that means real leg travel. Run it with the leg DISCONNECTED
from the platform and free to rotate, ONE node at a time, and watch it. If the
calibration can't complete (e.g. it hits a travel limit) it reports a non-zero
procedure_result and this script will NOT save.

  sudo systemctl stop stable_bot                       # free the bus
  python3 scripts/calibrate_via_can.py --node 2        # dry-run preview
  python3 scripts/calibrate_via_can.py --node 2 --run  # actually calibrate
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed — pip3 install --user python-can")

_HERE = os.path.dirname(os.path.abspath(__file__))
ENDPOINTS_PATH = os.path.join(os.path.dirname(_HERE), 'data', 'odrive_endpoints.json')

# ODrive standard CAN command IDs (firmware-stable; NOT SDO endpoints).
CMD_HEARTBEAT      = 0x001
CMD_RX_SDO         = 0x004
CMD_SET_AXIS_STATE = 0x007
CMD_CLEAR_ERRORS   = 0x018
OPCODE_WRITE       = 0x01

STATE_IDLE = 1
STATE_FULL_CALIBRATION_SEQUENCE = 3
PROCEDURE_SUCCESS = 0


def _send(bus, node, cmd, data=b''):
    data = (data + b'\x00' * 8)[:8]
    bus.send(can.Message(arbitration_id=(node << 5) | cmd, data=data,
                         is_extended_id=False), timeout=0.2)


def _call_function(bus, node, endpoint_id):
    """Trigger a function endpoint (e.g. save_configuration) via Tx_SDO: a
    write opcode with no value."""
    prologue = struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0)
    _send(bus, node, CMD_RX_SDO, prologue)


def _read_heartbeat(bus, node, timeout):
    """Return (axis_state, procedure_result) from the next heartbeat for this
    node, or (None, None) on timeout. Heartbeat layout (0.6.x): [4]=axis_state,
    [5]=procedure_result."""
    want = (node << 5) | CMD_HEARTBEAT
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if m is None:
            break
        if m.arbitration_id == want and len(m.data) >= 6:
            return int(m.data[4]), int(m.data[5])
    return None, None


def _load_save_endpoint():
    if not os.path.isfile(ENDPOINTS_PATH):
        sys.exit(f"endpoint table not found: {ENDPOINTS_PATH}\n"
                 "Run dump_odrive_endpoints.py (USB, one-time per firmware).")
    with open(ENDPOINTS_PATH) as f:
        eps = json.load(f)
    ent = (eps.get('endpoints') or {}).get('save_configuration')
    if ent is None:
        sys.exit("save_configuration missing from odrive_endpoints.json")
    return int(ent['id'])


def calibrate_node(node, channel, do_run):
    save_id = _load_save_endpoint()
    bus = can.interface.Bus(channel=channel, interface='socketcan')
    try:
        st, pr = _read_heartbeat(bus, node, timeout=2.0)
        if st is None:
            sys.exit(f"no heartbeat from node {node} on {channel} — is it "
                     f"powered and on the bus, and is the stack stopped?")
        print(f"node {node}: alive (axis_state={st}). save endpoint={save_id}.")
        if not do_run:
            print("DRY-RUN — would clear_errors, run FULL_CALIBRATION_SEQUENCE, "
                  "wait for IDLE+success, then save_configuration().")
            print("It writes nothing else. Re-run with --run, leg DISCONNECTED "
                  "and free to rotate.")
            return

        print(f"node {node}: calibrating in 3s — Ctrl-C to abort "
              "(the motor WILL spin)...")
        for i in (3, 2, 1):
            print(f"  {i}...")
            time.sleep(1.0)

        _send(bus, node, CMD_CLEAR_ERRORS, b'\x00')
        time.sleep(0.3)
        _send(bus, node, CMD_SET_AXIS_STATE,
              struct.pack('<I', STATE_FULL_CALIBRATION_SEQUENCE))
        print(f"node {node}: FULL_CALIBRATION_SEQUENCE started...")
        time.sleep(1.0)                      # let it leave IDLE

        deadline = time.monotonic() + 45.0
        last_state, result = None, None
        while time.monotonic() < deadline:
            st, pr = _read_heartbeat(bus, node, timeout=2.0)
            if st is None:
                continue
            if st != last_state:
                print(f"  axis_state={st} procedure_result={pr}")
                last_state = st
            if st == STATE_IDLE:
                result = pr
                break
        if result is None:
            sys.exit(f"node {node}: calibration did not return to IDLE within "
                     f"45s. Check the drive (it may still be running or faulted).")
        if result != PROCEDURE_SUCCESS:
            sys.exit(f"node {node}: calibration FAILED (procedure_result="
                     f"{result}). NOT saving. Check motor/encoder wiring and "
                     f"that the leg can rotate freely, clear errors, retry.")

        print(f"node {node}: calibration SUCCESS. save_configuration() — the "
              f"drive will reboot (~18s)...")
        _call_function(bus, node, save_id)
        time.sleep(18.0)
        print(f"node {node}: done. Restart the stack and verify the leg rotates "
              f"correctly under command.")
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--node', type=int, required=True,
                    help='CAN node id of ONE leg (0-5)')
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--run', action='store_true',
                    help='actually calibrate (default: dry-run preview only)')
    a = ap.parse_args()
    if not 0 <= a.node <= 5:
        sys.exit("--node must be 0-5")
    calibrate_node(a.node, a.channel, a.run)


if __name__ == '__main__':
    main()
