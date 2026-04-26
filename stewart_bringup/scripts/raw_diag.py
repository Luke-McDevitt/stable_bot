#!/usr/bin/env python3
"""raw_diag.py — direct-CAN ODrive Pro diagnostic + torque/velocity test.

Three things this script does:

  1. PASSIVE STATE READOUT (default, no motion):
     - Listens for ODrive heartbeats for 2 s and prints the latest
       axis_state + axis_error decoded.
     - Reads encoder (pos, vel) via Get_Encoder_Estimates RTR.
     - Reads Iq (commanded vs measured) via Get_Iq RTR.

  2. DIRECT TORQUE TEST (--torque <Nm>):
     Bypasses the position and velocity controllers entirely. Sets
     CONTROL_MODE=TORQUE_CONTROL, INPUT_MODE=PASSTHROUGH, ramps torque
     from 0 to the requested value over 1 s, holds 1 s, ramps back to
     0. Prints encoder pos/vel and Iq throughout.

     Interpretation:
       - Encoder moves smoothly (any direction) + Iq tracks command:
         commutation is FINE. Position-control issues are in gain tuning.
       - Encoder doesn't move + Iq is at the requested level:
         commutation is BROKEN. Motor draws current but produces no
         net torque. Recalibration required.
       - Iq doesn't track command at all: ODrive in error state, won't
         arm, or comms issue.

  3. DIRECT VELOCITY TEST (--vel <turns/s>):
     Sets CONTROL_MODE=VELOCITY_CONTROL, INPUT_MODE=VEL_RAMP. Commands
     the requested velocity for 2 s. Skips the position controller
     entirely.

Usage:
  python3 raw_diag.py 1                    # passive readout only
  python3 raw_diag.py 1 --torque 0.15      # 0.15 Nm direct torque test
  python3 raw_diag.py 1 --vel 0.1          # 0.1 turns/s direct vel test

Sign conventions:
  - +torque on node 0 = leg moves DOWN per project memory.
  - Other nodes TBD; check empirically.

If a test gets weird / motor whines / leg jerks, hit Ctrl-C — script
disarms on exit. Always run ONE leg at a time, and physically
position the leg at mid-stroke (away from either endstop) so it has
room to move in either direction without hitting hardware.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    print("python-can not installed: pip install python-can", file=sys.stderr)
    sys.exit(2)


# --- ODrive Pro CAN command IDs ---
CMD_HEARTBEAT           = 0x001
CMD_SET_AXIS_STATE      = 0x007
CMD_GET_ENCODER         = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS       = 0x00C
CMD_SET_INPUT_VEL       = 0x00D
CMD_SET_INPUT_TORQUE    = 0x00E
CMD_SET_LIMITS          = 0x00F
CMD_GET_IQ              = 0x014
CMD_CLEAR_ERRORS        = 0x018

# --- Axis states ---
STATE_UNDEFINED         = 0
STATE_IDLE              = 1
STATE_STARTUP           = 2
STATE_FULL_CALIBRATION  = 3
STATE_MOTOR_CALIBRATION = 4
STATE_ENCODER_OFFSET_CALIBRATION = 7
STATE_CLOSED_LOOP       = 8

AXIS_STATE_NAMES = {
    0: 'UNDEFINED',
    1: 'IDLE',
    2: 'STARTUP_SEQUENCE',
    3: 'FULL_CALIBRATION_SEQUENCE',
    4: 'MOTOR_CALIBRATION',
    7: 'ENCODER_OFFSET_CALIBRATION',
    8: 'CLOSED_LOOP_CONTROL',
}

# --- Control modes ---
CONTROL_MODE_VOLTAGE   = 0
CONTROL_MODE_TORQUE    = 1
CONTROL_MODE_VELOCITY  = 2
CONTROL_MODE_POSITION  = 3

# --- Input modes ---
INPUT_MODE_INACTIVE     = 0
INPUT_MODE_PASSTHROUGH  = 1
INPUT_MODE_VEL_RAMP     = 2
INPUT_MODE_POS_FILTER   = 3
INPUT_MODE_TRAP_TRAJ    = 5
INPUT_MODE_TORQUE_RAMP  = 6


# Common axis_error bit names — partial list; firmware-specific.
AXIS_ERROR_BITS = [
    (0x00000001, 'INVALID_STATE'),
    (0x00000020, 'WATCHDOG_TIMER_EXPIRED'),
    (0x00000040, 'MIN_ENDSTOP_PRESSED'),
    (0x00000080, 'MAX_ENDSTOP_PRESSED'),
    (0x00000100, 'ESTOP_REQUESTED'),
    (0x00000200, 'HOMING_WITHOUT_ENDSTOP'),
    (0x00000400, 'OVER_TEMP'),
    (0x00000800, 'UNKNOWN_POSITION'),
    (0x00001000, 'CONTROLLER_FAILED'),
    # 0x01000000 is what we saw in the homing trace earlier.
    (0x01000000, 'SPINOUT_DETECTED_OR_DISARMED'),
]


def decode_axis_error(err):
    if err == 0:
        return 'no error'
    bits = []
    for mask, name in AXIS_ERROR_BITS:
        if err & mask:
            bits.append(name)
    leftover = err
    for mask, _ in AXIS_ERROR_BITS:
        leftover &= ~mask
    if leftover:
        bits.append(f'unknown_bits=0x{leftover:08X}')
    return ', '.join(bits) if bits else f'unknown=0x{err:08X}'


def cmd_send(bus, node_id, cmd_id, data):
    arb = (node_id << 5) | cmd_id
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False),
             timeout=0.1)


def rtr_send(bus, node_id, cmd_id):
    arb = (node_id << 5) | cmd_id
    bus.send(can.Message(arbitration_id=arb, is_extended_id=False,
                         is_remote_frame=True), timeout=0.1)


def listen_heartbeats(bus, node_id, duration_s=2.0):
    """Listens for 'duration_s' seconds and returns the LATEST heartbeat
    received for the given node, or None if none seen.

    ODrive Pro 0.6.x heartbeat layout (subject to firmware version):
      bytes 0-3: axis_error (uint32)
      byte 4:    axis_state (uint8)
      byte 5:    procedure_result (uint8)
      byte 6:    trajectory_done_flag (uint8)
    """
    deadline = time.monotonic() + duration_s
    latest = None
    count = 0
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if (msg.arbitration_id >> 5) != node_id:
            continue
        if (msg.arbitration_id & 0x1F) != CMD_HEARTBEAT:
            continue
        if len(msg.data) < 5:
            continue
        axis_err = struct.unpack('<I', msg.data[:4])[0]
        axis_state = msg.data[4] if len(msg.data) > 4 else 0
        proc_result = msg.data[5] if len(msg.data) > 5 else 0
        traj_done = msg.data[6] if len(msg.data) > 6 else 0
        latest = (axis_err, axis_state, proc_result, traj_done)
        count += 1
    return latest, count


def read_encoder(bus, node_id, timeout=1.0):
    rtr_send(bus, node_id, CMD_GET_ENCODER)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if (msg.arbitration_id >> 5) != node_id:
            continue
        if (msg.arbitration_id & 0x1F) != CMD_GET_ENCODER:
            continue
        if len(msg.data) < 8:
            continue
        pos, vel = struct.unpack('<ff', msg.data[:8])
        return float(pos), float(vel)
    return None


def read_iq(bus, node_id, timeout=1.0):
    rtr_send(bus, node_id, CMD_GET_IQ)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.is_extended_id:
            continue
        if (msg.arbitration_id >> 5) != node_id:
            continue
        if (msg.arbitration_id & 0x1F) != CMD_GET_IQ:
            continue
        if len(msg.data) < 8:
            continue
        iq_set, iq_meas = struct.unpack('<ff', msg.data[:8])
        return float(iq_set), float(iq_meas)
    return None


def disarm(bus, node_id):
    try:
        cmd_send(bus, node_id, CMD_SET_AXIS_STATE,
                 struct.pack('<I', STATE_IDLE))
    except Exception:
        pass


def passive_diag(bus, node):
    print(f"=== Passive diagnostic for node {node} ===")
    print("Listening for heartbeats (2 s) ...")
    latest, count = listen_heartbeats(bus, node, duration_s=2.0)
    if latest is None:
        print(f"  ⚠  NO heartbeat received from node {node} in 2 s.")
        print(f"     ODrive likely off, on wrong CAN bus, or not configured "
              f"with this node ID.")
    else:
        ax_err, ax_state, proc, traj = latest
        state_name = AXIS_STATE_NAMES.get(ax_state, f'unknown({ax_state})')
        print(f"  heartbeat count       : {count} (~{count/2:.1f} Hz)")
        print(f"  axis_state            : {ax_state} ({state_name})")
        print(f"  axis_error            : 0x{ax_err:08X} "
              f"({decode_axis_error(ax_err)})")
        print(f"  procedure_result      : {proc}")
        print(f"  trajectory_done_flag  : {traj}")
    print()
    enc = read_encoder(bus, node, timeout=1.0)
    if enc is None:
        print("  ⚠  Encoder read timed out.")
    else:
        print(f"  encoder pos           : {enc[0]:+.4f} turns")
        print(f"  encoder vel           : {enc[1]:+.4f} turns/s")
    iq = read_iq(bus, node, timeout=1.0)
    if iq is None:
        print("  Iq read timed out.")
    else:
        print(f"  Iq commanded          : {iq[0]:+.3f} A")
        print(f"  Iq measured           : {iq[1]:+.3f} A")
    print()


def torque_test(bus, node, torque_nm, current_a):
    """Direct torque control test. Bypasses position + velocity controllers."""
    print(f"=== TORQUE TEST: node {node}, target = {torque_nm:+.3f} Nm ===")
    enc0 = read_encoder(bus, node, timeout=1.0)
    if enc0 is None:
        print("ERROR: can't read encoder, aborting.")
        return
    print(f"  starting pos = {enc0[0]:+.4f} turns")

    # Configure torque control + passthrough.
    print("Configuring CONTROL_MODE=TORQUE_CONTROL, INPUT_MODE=PASSTHROUGH ...")
    cmd_send(bus, node, CMD_CLEAR_ERRORS, b'\x00')
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_LIMITS,
             struct.pack('<ff', 5.0, float(current_a)))   # vel_limit=5 turns/s
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_CONTROLLER_MODE,
             struct.pack('<II', CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH))
    time.sleep(0.05)
    # Seed input_torque = 0 so we don't lurch on CLOSED_LOOP entry.
    cmd_send(bus, node, CMD_SET_INPUT_TORQUE,
             struct.pack('<f', 0.0))
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_AXIS_STATE,
             struct.pack('<I', STATE_CLOSED_LOOP))
    time.sleep(0.1)

    # Verify state.
    print("Verifying state via heartbeat ...")
    latest, _ = listen_heartbeats(bus, node, duration_s=0.5)
    if latest is None:
        print("  ⚠  no heartbeat — bailing out.")
        disarm(bus, node)
        return
    ax_err, ax_state, _, _ = latest
    state_name = AXIS_STATE_NAMES.get(ax_state, f'unknown({ax_state})')
    print(f"  axis_state = {ax_state} ({state_name})")
    print(f"  axis_error = 0x{ax_err:08X} ({decode_axis_error(ax_err)})")
    if ax_state != STATE_CLOSED_LOOP:
        print(f"  ⚠  axis didn't enter CLOSED_LOOP (state={state_name}).")
        print(f"     Likely cause: motor or encoder calibration not pre_calibrated.")
        print(f"     Disarming.")
        disarm(bus, node)
        return

    # Software-ramp the torque.
    TICK = 0.02
    PHASE = 1.0   # ramp time in s
    HOLD = 1.0    # hold time in s
    print(f"Ramping torque 0 → {torque_nm:+.3f} Nm over {PHASE:.1f} s, "
          f"hold {HOLD:.1f} s, ramp back to 0 ...")
    t0 = time.monotonic()
    last_print = t0
    try:
        # Ramp up
        n_ticks_ramp = int(PHASE / TICK)
        for i in range(n_ticks_ramp + 1):
            tau = (i / n_ticks_ramp) * torque_nm
            cmd_send(bus, node, CMD_SET_INPUT_TORQUE,
                     struct.pack('<f', float(tau)))
            time.sleep(TICK)
            now = time.monotonic()
            if now - last_print > 0.25:
                last_print = now
                enc = read_encoder(bus, node, timeout=0.1)
                iq = read_iq(bus, node, timeout=0.1)
                pos_str = f"{enc[0]:+.4f}" if enc else "  ?  "
                vel_str = f"{enc[1]:+.3f}" if enc else "  ?  "
                iq_str  = f"{iq[1]:+.3f}" if iq else "  ?  "
                print(f"  t={now-t0:5.2f}s  cmd_torque={tau:+.3f}  "
                      f"pos={pos_str}  vel={vel_str}  Iq_meas={iq_str}")

        # Hold
        n_ticks_hold = int(HOLD / TICK)
        for i in range(n_ticks_hold):
            cmd_send(bus, node, CMD_SET_INPUT_TORQUE,
                     struct.pack('<f', float(torque_nm)))
            time.sleep(TICK)
            now = time.monotonic()
            if now - last_print > 0.25:
                last_print = now
                enc = read_encoder(bus, node, timeout=0.1)
                iq = read_iq(bus, node, timeout=0.1)
                pos_str = f"{enc[0]:+.4f}" if enc else "  ?  "
                vel_str = f"{enc[1]:+.3f}" if enc else "  ?  "
                iq_str  = f"{iq[1]:+.3f}" if iq else "  ?  "
                print(f"  t={now-t0:5.2f}s  cmd_torque={torque_nm:+.3f}  "
                      f"pos={pos_str}  vel={vel_str}  Iq_meas={iq_str}")

        # Ramp down
        for i in range(n_ticks_ramp + 1):
            tau = (1.0 - i / n_ticks_ramp) * torque_nm
            cmd_send(bus, node, CMD_SET_INPUT_TORQUE,
                     struct.pack('<f', float(tau)))
            time.sleep(TICK)

        # Final readings
        enc1 = read_encoder(bus, node, timeout=1.0)
        if enc0 is not None and enc1 is not None:
            print(f"FINAL: pos = {enc1[0]:+.4f} turns "
                  f"(moved {enc1[0] - enc0[0]:+.4f} turns "
                  f"= {(enc1[0] - enc0[0]) * 71.047:+.1f} mm)")

    except KeyboardInterrupt:
        print("\nCtrl-C — disarming.")
    finally:
        # Zero torque before disarm
        try:
            cmd_send(bus, node, CMD_SET_INPUT_TORQUE,
                     struct.pack('<f', 0.0))
        except Exception:
            pass
        disarm(bus, node)
        time.sleep(0.1)
    print()


def velocity_test(bus, node, vel_target, current_a):
    """Direct velocity control test. Skips position controller."""
    print(f"=== VELOCITY TEST: node {node}, target = {vel_target:+.3f} turns/s ===")
    enc0 = read_encoder(bus, node, timeout=1.0)
    if enc0 is None:
        print("ERROR: can't read encoder, aborting.")
        return
    print(f"  starting pos = {enc0[0]:+.4f} turns")
    print("Configuring CONTROL_MODE=VELOCITY_CONTROL, INPUT_MODE=VEL_RAMP ...")
    cmd_send(bus, node, CMD_CLEAR_ERRORS, b'\x00')
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_LIMITS,
             struct.pack('<ff', max(abs(vel_target) * 1.5, 0.5), float(current_a)))
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_CONTROLLER_MODE,
             struct.pack('<II', CONTROL_MODE_VELOCITY, INPUT_MODE_VEL_RAMP))
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_INPUT_VEL, struct.pack('<ff', 0.0, 0.0))
    time.sleep(0.05)
    cmd_send(bus, node, CMD_SET_AXIS_STATE,
             struct.pack('<I', STATE_CLOSED_LOOP))
    time.sleep(0.1)

    latest, _ = listen_heartbeats(bus, node, duration_s=0.5)
    if latest is None:
        print("  ⚠  no heartbeat — bailing out.")
        disarm(bus, node)
        return
    ax_err, ax_state, _, _ = latest
    state_name = AXIS_STATE_NAMES.get(ax_state, f'unknown({ax_state})')
    print(f"  axis_state = {ax_state} ({state_name})")
    print(f"  axis_error = 0x{ax_err:08X} ({decode_axis_error(ax_err)})")
    if ax_state != STATE_CLOSED_LOOP:
        print(f"  ⚠  axis didn't enter CLOSED_LOOP. Bailing.")
        disarm(bus, node)
        return

    print(f"Commanding {vel_target:+.3f} turns/s for 2 s ...")
    t0 = time.monotonic()
    last_print = t0
    try:
        cmd_send(bus, node, CMD_SET_INPUT_VEL,
                 struct.pack('<ff', float(vel_target), 0.0))
        while time.monotonic() - t0 < 2.0:
            time.sleep(0.02)
            now = time.monotonic()
            if now - last_print > 0.25:
                last_print = now
                enc = read_encoder(bus, node, timeout=0.1)
                iq = read_iq(bus, node, timeout=0.1)
                pos_str = f"{enc[0]:+.4f}" if enc else "  ?  "
                vel_str = f"{enc[1]:+.3f}" if enc else "  ?  "
                iq_str  = f"{iq[1]:+.3f}" if iq else "  ?  "
                print(f"  t={now-t0:5.2f}s  cmd_vel={vel_target:+.3f}  "
                      f"pos={pos_str}  vel={vel_str}  Iq_meas={iq_str}")
        # Ramp back to 0
        cmd_send(bus, node, CMD_SET_INPUT_VEL, struct.pack('<ff', 0.0, 0.0))
        time.sleep(0.5)
        enc1 = read_encoder(bus, node, timeout=1.0)
        if enc0 and enc1:
            print(f"FINAL: pos = {enc1[0]:+.4f} turns "
                  f"(moved {enc1[0] - enc0[0]:+.4f} turns)")
    except KeyboardInterrupt:
        print("\nCtrl-C — disarming.")
    finally:
        try:
            cmd_send(bus, node, CMD_SET_INPUT_VEL, struct.pack('<ff', 0.0, 0.0))
        except Exception:
            pass
        disarm(bus, node)
    print()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node_id', type=int, help='ODrive CAN node ID (0..5).')
    ap.add_argument('--torque', type=float, default=None,
                    help='Run direct-torque test with this Nm target.')
    ap.add_argument('--vel', type=float, default=None,
                    help='Run direct-velocity test with this turns/s target.')
    ap.add_argument('--current', type=float, default=4.0,
                    help='ODrive current limit (A; default 4.0).')
    ap.add_argument('--bus', default='can0', help='CAN interface (default can0).')
    ap.add_argument('--bitrate', type=int, default=1_000_000)
    args = ap.parse_args()

    if not 0 <= args.node_id <= 5:
        print(f"node_id {args.node_id} out of range.", file=sys.stderr)
        return 2

    print(f"Opening {args.bus} at {args.bitrate} bps ...")
    try:
        bus = can.Bus(interface='socketcan', channel=args.bus,
                      bitrate=args.bitrate, state=can.BusState.ACTIVE)
    except Exception as e:
        print(f"failed to open {args.bus}: {e}", file=sys.stderr)
        print("hint: sudo ip link set can0 up type can bitrate 1000000",
              file=sys.stderr)
        return 3

    try:
        # Drain RX
        for _ in range(40):
            if bus.recv(timeout=0.005) is None:
                break

        passive_diag(bus, args.node_id)

        if args.torque is not None:
            torque_test(bus, args.node_id, args.torque, args.current)
        elif args.vel is not None:
            velocity_test(bus, args.node_id, args.vel, args.current)
        else:
            print("(passive mode only — pass --torque or --vel for motion test.)")

    finally:
        disarm(bus, args.node_id)
        try:
            bus.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
