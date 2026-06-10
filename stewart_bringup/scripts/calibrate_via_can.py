#!/usr/bin/env python3
"""calibrate_via_can.py — per-leg ODrive Pro motor+encoder calibration over
CAN, with a before/after parameter dump pushed to git and a built-in verify.

After reassembling an actuator the motor/encoder commutation offset is lost
(the motor "doesn't rotate as commanded"). This fires the standard
FULL_CALIBRATION_SEQUENCE (axis state 3) — re-measuring phase
resistance/inductance and the encoder offset — confirms the axis actually
enters the calibration, polls until it finishes, checks the result, and only
then calls save_configuration().

It writes NO config of its own; save_configuration() persists the fresh
calibration alongside the unchanged flash config. On fw 0.6.11 the RS-485
absolute encoder trusts the result via the *_valid flags the calibration sets,
and startup_*_calibration is already false, so calibrate + save is enough.

Around the calibration it dumps every readable `.config.` parameter over SDO
to `tuning_data/odrive_cal/<utc>_node<N>_{before,after}.json` and pushes both
to git, then (verify, ON by default) diffs them: it lists what the calibration
changed and explicitly confirms the protected config (node_id, vel/current
limits, control/input mode, gains) did NOT move.

SAFETY — calibration ROTATES the motor (the encoder-offset lockin); on a
lead-screw leg that is real travel. Run with the leg DISCONNECTED from the
platform and free to rotate, ONE node at a time, and watch it. If the axis
never leaves IDLE (a rejected state, usually an active error) or returns a
non-zero procedure_result, the script aborts WITHOUT saving.

  sudo systemctl stop stable_bot                       # free the bus
  python3 scripts/calibrate_via_can.py --node 2        # dry-run preview
  python3 scripts/calibrate_via_can.py --node 2 --run  # calibrate (+dump/push/verify)
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import struct
import subprocess
import sys
import time

try:
    import can
except ImportError:
    sys.exit("python-can not installed — pip3 install --user python-can")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from set_odrive_feedforward_via_can import SdoClient            # noqa: E402

_DATA = os.path.join(os.path.dirname(_HERE), 'data')
ENDPOINTS_PATH = os.path.join(_DATA, 'odrive_endpoints.json')           # has save_configuration
FLAT_PATH = os.path.join(_DATA, 'odrive_flat_endpoints_0.6.11-1.json')  # full param table

# ODrive standard CAN command IDs (firmware-stable; not SDO endpoints).
CMD_HEARTBEAT      = 0x001
CMD_SET_AXIS_STATE = 0x007
CMD_CLEAR_ERRORS   = 0x018
STATE_IDLE = 1
STATE_FULL_CALIBRATION_SEQUENCE = 3
PROCEDURE_SUCCESS = 0

# Flat-table type string -> the kind SdoClient._unpack_value understands.
_READABLE_KINDS = {
    'float': 'float32', 'float32': 'float32', 'float64': 'float64', 'bool': 'bool',
    'uint8': 'uint8', 'uint16': 'uint16', 'uint32': 'uint32',
    'int8': 'int8', 'int16': 'int16', 'int32': 'int32',
}

# Config that calibration must NOT touch — the verify flags any that move.
_PROTECTED = (
    'axis0.config.can.node_id',
    'axis0.controller.config.vel_limit',
    'axis0.config.motor.current_soft_max',
    'axis0.controller.config.control_mode',
    'axis0.controller.config.input_mode',
    'axis0.controller.config.pos_gain',
    'axis0.controller.config.vel_gain',
    'axis0.controller.config.vel_integrator_gain',
    'axis0.config.enable_watchdog',     # temporarily toggled, must end up ON
)


def _ts():
    return datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def _send(bus, node, cmd, data=b''):
    # Send the EXACT payload (matches the control node's _send_cmd). ODrive
    # validates the DLC on standard commands like Set_Axis_State (4 bytes), so
    # padding to 8 makes the drive silently ignore the command.
    bus.send(can.Message(arbitration_id=(node << 5) | cmd, data=data,
                         is_extended_id=False), timeout=0.2)


def _read_heartbeat(bus, node, timeout):
    """Next heartbeat for node -> (axis_state, procedure_result, axis_error),
    else (None, None, None). Layout (0.6.x): [0:4]=axis_error (legacy),
    [4]=axis_state, [5]=procedure_result."""
    want = (node << 5) | CMD_HEARTBEAT
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        m = bus.recv(timeout=max(0.0, deadline - time.monotonic()))
        if m is None:
            break
        if m.arbitration_id == want and len(m.data) >= 6:
            err = struct.unpack('<I', bytes(m.data[0:4]))[0]
            return int(m.data[4]), int(m.data[5]), err
    return None, None, None


def _load_save_endpoint():
    with open(ENDPOINTS_PATH) as f:
        ent = (json.load(f).get('endpoints') or {}).get('save_configuration')
    if ent is None:
        sys.exit("save_configuration missing from odrive_endpoints.json")
    return int(ent['id'])


def _load_flat_config_endpoints():
    """Return [(path, id, kind)] for every readable scalar `.config.` endpoint."""
    with open(FLAT_PATH) as f:
        eps = json.load(f).get('endpoints') or {}
    out = []
    for path, ent in eps.items():
        if '.config.' not in path:
            continue
        kind = _READABLE_KINDS.get(ent.get('type'))
        if kind is None or 'r' not in (ent.get('access') or ''):
            continue
        out.append((path, int(ent['id']), kind))
    return out


def _find_ep(flat_eps, path):
    """(id, kind) for a config path in the flat list, else (None, None)."""
    for p, ep_id, kind in flat_eps:
        if p == path:
            return ep_id, kind
    return None, None


def _dump_config(client, node, flat_eps):
    params = {}
    for path, ep_id, kind in flat_eps:
        try:
            v = client.read(node, ep_id, kind, timeout=0.25)
        except Exception:
            v = None
        if v is None:
            continue
        params[path] = round(v, 9) if isinstance(v, float) else v
    return params


def _repo_root():
    try:
        r = subprocess.run(['git', '-C', _HERE, 'rev-parse', '--show-toplevel'],
                           capture_output=True, text=True, timeout=15)
        if r.returncode == 0:
            return r.stdout.strip()
    except Exception:
        pass
    return None


def _write_dump(repo, run_ts, node, phase, params, fw):
    rel = os.path.join('tuning_data', 'odrive_cal',
                       f"{run_ts}_node{node}_{phase}.json")
    full = os.path.join(repo, rel)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, 'w') as f:
        json.dump({'node': node, 'phase': phase, 'utc': run_ts, 'fw': fw,
                   'params': params}, f, indent=2, sort_keys=True)
    print(f"  wrote {rel} ({len(params)} params)")
    return rel


def _git_push(repo, rel, msg):
    def g(*a, check=True):
        return subprocess.run(['git', '-C', repo, *a], capture_output=True,
                              text=True, timeout=90, check=check)
    try:
        g('add', '--', rel)
        g('commit', '-q', '-m', msg)
        g('pull', '--rebase', '--autostash', '-q', check=False)
        g('push', '-q')
        print(f"  pushed {rel}")
    except Exception as e:
        print(f"  git push failed ({e}) — dump saved locally, push it manually later")


def _changed(b, a):
    """True if b != a, treating nan==nan as unchanged (Python's nan != nan)."""
    if (isinstance(b, float) and isinstance(a, float)
            and math.isnan(b) and math.isnan(a)):
        return False
    return b != a


def _verify(before, after):
    keys = sorted(set(before) | set(after))
    changed = [(k, before.get(k), after.get(k)) for k in keys
               if _changed(before.get(k), after.get(k))]
    print(f"\n=== VERIFY: {len(changed)} parameter(s) changed by calibration ===")
    for k, b, a in changed:
        tag = '   [PROTECTED]' if k in _PROTECTED else ''
        print(f"  {k}: {b} -> {a}{tag}")
    prot = [k for k, _b, _a in changed if k in _PROTECTED]
    if prot:
        print(f"  *** WARNING: protected config changed: {prot} — investigate. ***")
    else:
        print("  protected config (node_id / limits / mode / gains) UNCHANGED ✓")
    rv = after.get('axis0.config.motor.phase_resistance_valid')
    iv = after.get('axis0.config.motor.phase_inductance_valid')
    if rv is not None or iv is not None:
        ok = bool(rv) and bool(iv)
        print(f"  motor calibration valid: R={rv} L={iv} -> "
              f"{'OK' if ok else 'NOT VALID — re-run'}")
    if not changed:
        print("  (nothing changed — calibration may not have run; check the log above)")


def calibrate_node(node, channel, do_run, do_push, do_verify):
    save_id = _load_save_endpoint()
    flat_eps = _load_flat_config_endpoints()
    fw = '0.6.11'
    repo = _repo_root() if do_run else None
    run_ts = _ts()
    client = SdoClient(channel=channel)
    bus = client.bus
    wd_ep, wd_kind = _find_ep(flat_eps, 'axis0.config.enable_watchdog')
    wd_orig, wd_disabled = None, False
    try:
        st, _pr, _e = _read_heartbeat(bus, node, timeout=2.0)
        if st is None:
            sys.exit(f"no heartbeat from node {node} on {channel} — powered, on "
                     f"the bus, and stack stopped?")
        print(f"node {node}: alive (axis_state={st}); {len(flat_eps)} config "
              f"params to dump; save endpoint={save_id}.")
        if not do_run:
            print("DRY-RUN — would: dump+push BEFORE, FULL_CALIBRATION_SEQUENCE, "
                  "save, dump+push AFTER, verify. Re-run with --run, leg free.")
            return
        if repo is None:
            print("  WARNING: not inside a git repo — dumps will be local only.")
            do_push = False

        # ---- BEFORE dump (+push immediately, so the pre-state is safe) ----
        print(f"node {node}: dumping parameters (BEFORE)...")
        before = _dump_config(client, node, flat_eps)
        before_rel = _write_dump(repo, run_ts, node, 'before', before, fw) if repo else None
        if do_push and before_rel:
            _git_push(repo, before_rel, f"odrive cal node {node}: pre-calibration param dump")

        # ---- Watchdog: with the stack stopped nothing feeds the safety
        # watchdog, so it expires and the drive rejects calibration
        # (WATCHDOG_TIMER_EXPIRED, axis_error 0x01000000). Temporarily disable
        # it in RAM, then RESTORE it before save_configuration so the persisted
        # config keeps the watchdog ON exactly as configured. ----
        if wd_ep is not None:
            try:
                wd_orig = client.read(node, wd_ep, wd_kind, timeout=0.5)
            except Exception:
                wd_orig = None
            if wd_orig:
                client.write(node, wd_ep, False, wd_kind)
                wd_disabled = True
                print("  enable_watchdog temporarily OFF for calibration "
                      "(restored + saved ON afterwards)")

        # ---- Calibrate ----
        print(f"node {node}: calibrating in 3s — Ctrl-C to abort (motor WILL spin)...")
        for i in (3, 2, 1):
            print(f"  {i}..."); time.sleep(1.0)
        _send(bus, node, CMD_CLEAR_ERRORS, b'\x00')
        time.sleep(0.3)
        _send(bus, node, CMD_SET_AXIS_STATE,
              struct.pack('<I', STATE_FULL_CALIBRATION_SEQUENCE))
        print(f"node {node}: FULL_CALIBRATION_SEQUENCE requested...")

        # Phase 1 — confirm it actually STARTS (leaves IDLE). If the drive
        # rejects the state (almost always an active error / precondition) it
        # just stays in IDLE.
        left_idle = False
        start_deadline = time.monotonic() + 5.0
        while time.monotonic() < start_deadline:
            s, pr, err = _read_heartbeat(bus, node, timeout=1.0)
            if s is not None and s != STATE_IDLE:
                left_idle = True
                print(f"  calibration running (axis_state={s})...")
                break
        if not left_idle:
            s, pr, err = _read_heartbeat(bus, node, timeout=1.0)
            sys.exit(
                f"node {node}: calibration did NOT start — axis stayed IDLE, so the "
                f"drive rejected the request (heartbeat axis_error=0x{(err or 0):08X}, "
                f"procedure_result={pr}). Almost always an active error or a "
                f"precondition. Open the GUI errors panel (or odrivetool) on node "
                f"{node}, clear errors, check the motor + RS-485 encoder wiring, then "
                f"retry. Nothing was saved.")

        # Phase 2 — wait for it to finish (return to IDLE) + read the result.
        deadline = time.monotonic() + 60.0
        result = None
        while time.monotonic() < deadline:
            s, pr, err = _read_heartbeat(bus, node, timeout=2.0)
            if s == STATE_IDLE:
                result = pr
                break
        if result is None:
            sys.exit(f"node {node}: calibration didn't finish within 60s — check the drive.")
        if result != PROCEDURE_SUCCESS:
            sys.exit(f"node {node}: calibration FAILED (procedure_result={result}). "
                     f"NOT saving. Check wiring + that the motor can rotate freely, "
                     f"clear errors, retry.")
        print(f"node {node}: calibration SUCCESS.")
        if wd_disabled:                  # restore BEFORE save so flash keeps it ON
            client.write(node, wd_ep, wd_orig, wd_kind)
            wd_disabled = False
            print("  enable_watchdog restored ON")
        print("  save_configuration() (reboot ~18s)...")
        client.call_function(node, save_id)

        # ---- wait for the reboot, then AFTER dump (+push) + verify ----
        time.sleep(18.0)
        back_deadline = time.monotonic() + 15.0
        while time.monotonic() < back_deadline:
            s, _pr, _e = _read_heartbeat(bus, node, timeout=2.0)
            if s is not None:
                break
        print(f"node {node}: dumping parameters (AFTER)...")
        after = _dump_config(client, node, flat_eps)
        after_rel = _write_dump(repo, run_ts, node, 'after', after, fw) if repo else None
        if do_push and after_rel:
            _git_push(repo, after_rel, f"odrive cal node {node}: post-calibration param dump")
        if do_verify:
            _verify(before, after)
        print(f"node {node}: done. Restart the stack and verify the leg rotates "
              f"correctly under command.")
    finally:
        if wd_disabled and wd_ep is not None:   # early exit before the restore
            try:
                client.write(node, wd_ep, wd_orig if wd_orig is not None else True, wd_kind)
                print("  enable_watchdog restored ON (after early exit; flash unchanged)")
            except Exception:
                pass
        try:
            client.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--node', type=int, required=True, help='CAN node id of ONE leg (0-5)')
    ap.add_argument('--channel', default='can0')
    ap.add_argument('--run', action='store_true',
                    help='actually calibrate (default: dry-run preview only)')
    ap.add_argument('--no-push', action='store_true',
                    help='write the before/after dumps locally but do not git push')
    ap.add_argument('--no-verify', action='store_true',
                    help='skip the before/after diff report (verify is on by default)')
    a = ap.parse_args()
    if not 0 <= a.node <= 5:
        sys.exit("--node must be 0-5")
    calibrate_node(a.node, a.channel, a.run,
                   do_push=not a.no_push, do_verify=not a.no_verify)


if __name__ == '__main__':
    main()
