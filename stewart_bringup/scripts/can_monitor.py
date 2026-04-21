#!/usr/bin/env python3
"""
SocketCAN listener for bring-up debugging. Prints every received frame with:
  - monotonic timestamp (seconds since program start)
  - arbitration ID (hex)
  - ODrive decode: node_id and command name from the standard
    (node_id << 5) | cmd_id encoding used by ODrive Pro
  - DLC and data bytes (hex)

Usage:
  python3 can_monitor.py              # listens on can0
  python3 can_monitor.py can0
  python3 can_monitor.py --raw        # skip ODrive decode, just frames
"""

import argparse
import socket
import struct
import sys
import time


ODRIVE_CMDS = {
    0x000: "GET_VERSION",
    0x001: "HEARTBEAT",
    0x002: "ESTOP",
    0x003: "GET_ERROR",
    0x004: "RxSdo",
    0x005: "TxSdo",
    0x006: "ADDRESS",
    0x007: "SET_AXIS_STATE",
    0x009: "GET_ENCODER_ESTIMATES",
    0x00B: "SET_CONTROLLER_MODE",
    0x00C: "SET_INPUT_POS",
    0x00D: "SET_INPUT_VEL",
    0x00E: "SET_INPUT_TORQUE",
    0x00F: "SET_LIMITS",
    0x011: "SET_TRAJ_VEL_LIMIT",
    0x012: "SET_TRAJ_ACCEL_LIMITS",
    0x013: "SET_TRAJ_INERTIA",
    0x014: "GET_IQ",
    0x015: "GET_TEMPERATURE",
    0x016: "REBOOT",
    0x017: "GET_BUS_VOLTAGE_CURRENT",
    0x018: "CLEAR_ERRORS",
    0x019: "SET_ABSOLUTE_POSITION",
    0x01A: "SET_POS_GAIN",
    0x01B: "SET_VEL_GAINS",
    0x01C: "GET_TORQUES",
    0x01D: "GET_POWERS",
    0x01F: "ENTER_DFU_MODE",
}

ODRIVE_STATES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQ", 3: "FULL_CAL",
    4: "MOTOR_CAL", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CAL",
    8: "CLOSED_LOOP", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
    12: "ENCODER_HALL_POL", 13: "ENCODER_HALL_PHASE",
}

ODRIVE_PROC_RESULTS = {
    0: "SUCCESS", 1: "BUSY", 2: "CANCELLED", 3: "DISARMED", 4: "NO_RESPONSE",
    5: "POLE_PAIRS_CPR_MISMATCH", 6: "PHASE_R_OOR", 7: "PHASE_L_OOR",
    8: "UNBALANCED_PHASES", 9: "INVALID_MOTOR_TYPE", 10: "ENCODER_READING_BAD",
    11: "ENCODER_ESTIMATION_BAD", 12: "FAILED_HOMING", 13: "TIMEOUT",
    14: "HOMING_WITHOUT_ENDSTOP", 15: "INVALID_STATE", 16: "NOT_CALIBRATED",
    17: "NOT_CONVERGING",
}


def decode_heartbeat(data):
    if len(data) < 7:
        return ""
    axis_error = struct.unpack_from("<I", data, 0)[0]
    axis_state = data[4]
    proc_result = data[5]
    traj_done = data[6]
    state_name = ODRIVE_STATES.get(axis_state, f"state={axis_state}")
    proc_name = ODRIVE_PROC_RESULTS.get(proc_result, f"proc={proc_result}")
    err_str = f"err=0x{axis_error:08X}" if axis_error else "err=0"
    return f" [{state_name} / {proc_name} / {err_str} / traj_done={traj_done}]"


def decode_odrive(arb_id):
    node_id = (arb_id >> 5) & 0x3F
    cmd_id = arb_id & 0x1F
    name = ODRIVE_CMDS.get(cmd_id, f"cmd=0x{cmd_id:02X}")
    return node_id, name


def main():
    p = argparse.ArgumentParser(description="SocketCAN listener with ODrive decode")
    p.add_argument("iface", nargs="?", default="can0")
    p.add_argument("--raw", action="store_true", help="skip ODrive decode")
    args = p.parse_args()

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.iface,))
    except OSError as e:
        print(f"bind to {args.iface} failed: {e}", file=sys.stderr)
        print(f"verify with: ip -details link show {args.iface}", file=sys.stderr)
        sys.exit(1)

    print(f"listening on {args.iface} (Ctrl+C to stop)")
    t0 = time.monotonic()
    count = 0
    try:
        while True:
            frame = sock.recv(16)
            can_id, dlc = struct.unpack_from("<IB", frame, 0)
            data = frame[8:8 + dlc]
            arb = can_id & 0x7FF
            t = time.monotonic() - t0
            hex_data = " ".join(f"{b:02X}" for b in data)
            if args.raw:
                print(f"{t:9.3f}  id=0x{arb:03X}  dlc={dlc}  {hex_data}")
            else:
                node_id, name = decode_odrive(arb)
                extra = decode_heartbeat(data) if (arb & 0x1F) == 0x001 else ""
                print(f"{t:9.3f}  id=0x{arb:03X}  node={node_id:02d} {name:<22}  dlc={dlc}  {hex_data}{extra}")
            count += 1
    except KeyboardInterrupt:
        print(f"\nstopped after {count} frames")


if __name__ == "__main__":
    main()
