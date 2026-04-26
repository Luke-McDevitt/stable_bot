#!/usr/bin/env python3
"""raw_jog.py — minimal direct-CAN jog for ONE ODrive Pro.

Bypasses everything: no leg_limits.yaml, no stewart_control_node, no
GUI, no soft limits, no homing. Just opens can0, reads the requested
node's encoder, ramps input_pos by your specified delta at your
specified rate, and disarms.

Requires only `python-can` (already installed for stewart_control_node).

Sign convention from the project memory: for node 0, positive vel
direction = leg goes DOWN (encoder INCREASES). Other nodes are TBD —
test with a small delta first to verify direction on each node.

Usage:
  python3 raw_jog.py <node_id> <delta_turns> [--rate R] [--current A] [--bus can0]

Examples:
  # Read encoder + show current position for node 1, no motion.
  python3 raw_jog.py 1 0

  # Move node 1 by -1.0 turns (encoder DECREASE = leg up on user's
  # hardware) at 0.2 turns/s, with 4 A current limit.
  python3 raw_jog.py 1 -1.0 --rate 0.2 --current 4.0

  # Move node 0 by +0.5 turns (encoder INCREASE = leg down) at 0.1 turns/s.
  python3 raw_jog.py 0 0.5 --rate 0.1

Safety:
  - Reads encoder BEFORE any motion. Refuses if no reply.
  - Seeds input_pos with current encoder before STATE_CLOSED_LOOP.
  - Software-ramps input_pos at the requested rate (no instant jumps).
  - Disarms (STATE_IDLE) on completion, on Ctrl-C, on exception.
  - Velocity feedforward = sign * rate (matches the position ramp).

If can0 isn't up:
    sudo ip link set can0 up type can bitrate 1000000
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import can
except ImportError:
    print("python-can not installed: pip install python-can",
          file=sys.stderr)
    sys.exit(2)


# --- ODrive Pro CAN command IDs ---
CMD_GET_ENCODER         = 0x009
CMD_SET_AXIS_STATE      = 0x007
CMD_SET_LIMITS          = 0x00F
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS       = 0x00C
CMD_CLEAR_ERRORS        = 0x018

# --- Modes ---
STATE_IDLE              = 1
STATE_CLOSED_LOOP       = 8
CONTROL_MODE_POSITION   = 3
INPUT_MODE_PASSTHROUGH  = 1


def cmd_send(bus, node_id, cmd_id, data):
    arb = (node_id << 5) | cmd_id
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False),
             timeout=0.1)


def read_encoder(bus, node_id, timeout=2.0):
    """Send RTR for GET_ENCODER and wait for the reply. Returns (pos, vel)."""
    arb = (node_id << 5) | CMD_GET_ENCODER
    bus.send(can.Message(arbitration_id=arb, is_extended_id=False,
                         is_remote_frame=True), timeout=0.1)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
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
    raise TimeoutError(
        f"node {node_id}: no encoder reply within {timeout:.1f} s")


def disarm(bus, node_id):
    """Best-effort STATE_IDLE."""
    try:
        cmd_send(bus, node_id, CMD_SET_AXIS_STATE,
                 struct.pack('<I', STATE_IDLE))
    except Exception as e:
        print(f"  warn: disarm failed: {e}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node_id', type=int,
                    help='ODrive CAN node ID (0..5).')
    ap.add_argument('delta_turns', type=float,
                    help='Distance to move (turns). + = encoder INCREASE, - = encoder DECREASE.')
    ap.add_argument('--rate', type=float, default=0.2,
                    help='Ramp rate (turns/s; default 0.2). Lower = gentler.')
    ap.add_argument('--current', type=float, default=4.0,
                    help='ODrive current limit (A; default 4.0).')
    ap.add_argument('--vel-cap', type=float, default=None,
                    help='ODrive vel_limit (default = 1.5 * rate, min 0.5).')
    ap.add_argument('--bus', default='can0', help='CAN interface (default can0).')
    ap.add_argument('--bitrate', type=int, default=1_000_000,
                    help='CAN bitrate (default 1 Mbps).')
    ap.add_argument('--timeout', type=float, default=20.0,
                    help='Max wall-clock time (s; default 20).')
    args = ap.parse_args()

    if not 0 <= args.node_id <= 5:
        print(f"node_id {args.node_id} out of range (0..5).", file=sys.stderr)
        return 2
    vel_cap = args.vel_cap if args.vel_cap is not None else max(args.rate * 1.5, 0.5)

    print(f"Opening {args.bus} at {args.bitrate} bps ...")
    try:
        bus = can.Bus(interface='socketcan', channel=args.bus,
                      bitrate=args.bitrate, state=can.BusState.ACTIVE)
    except Exception as e:
        print(f"failed to open {args.bus}: {e}", file=sys.stderr)
        print("hint: sudo ip link set can0 up type can bitrate 1000000",
              file=sys.stderr)
        return 3

    node = args.node_id
    try:
        # Drain any pending RX buffer.
        for _ in range(40):
            if bus.recv(timeout=0.005) is None:
                break

        # 1) Read current encoder.
        try:
            pos, vel = read_encoder(bus, node, timeout=2.0)
        except TimeoutError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            print(f"  Is ODrive node {node} powered and on the bus?",
                  file=sys.stderr)
            return 4
        print(f"node {node}: current pos = {pos:+.4f} turns, "
              f"vel = {vel:+.4f} turns/s")

        # If delta is zero, just print encoder and exit.
        if abs(args.delta_turns) < 1e-6:
            print("delta=0, nothing to do.")
            return 0

        target = pos + args.delta_turns
        sign = 1 if args.delta_turns > 0 else -1
        print(f"  target  = {target:+.4f} turns "
              f"({'+' if args.delta_turns >= 0 else ''}{args.delta_turns:.4f} delta)")
        print(f"  rate    = {args.rate:.3f} turns/s")
        print(f"  vel_cap = {vel_cap:.3f} turns/s")
        print(f"  current = {args.current:.2f} A")
        print(f"  est. time = {abs(args.delta_turns) / args.rate:.2f} s")

        # 2) Configure: clear errors, set limits, set position mode.
        print("Configuring ODrive ...")
        cmd_send(bus, node, CMD_CLEAR_ERRORS, b'\x00')
        time.sleep(0.05)
        cmd_send(bus, node, CMD_SET_LIMITS,
                 struct.pack('<ff', float(vel_cap), float(args.current)))
        time.sleep(0.05)
        cmd_send(bus, node, CMD_SET_CONTROLLER_MODE,
                 struct.pack('<II', CONTROL_MODE_POSITION,
                             INPUT_MODE_PASSTHROUGH))
        time.sleep(0.05)
        # Seed input_pos at current encoder so CLOSED_LOOP doesn't lurch.
        cmd_send(bus, node, CMD_SET_INPUT_POS,
                 struct.pack('<fhh', float(pos), 0, 0))
        time.sleep(0.05)

        # 3) Enter CLOSED_LOOP_CONTROL.
        print("CLOSED_LOOP_CONTROL ...")
        cmd_send(bus, node, CMD_SET_AXIS_STATE,
                 struct.pack('<I', STATE_CLOSED_LOOP))
        time.sleep(0.1)

        # 4) Software-ramp input_pos toward target at args.rate.
        TICK = 0.02  # 50 Hz update rate
        commanded = pos
        t_start = time.monotonic()
        last_print = t_start
        try:
            while abs(commanded - target) > 1e-4:
                step = sign * args.rate * TICK
                if abs(target - commanded) < abs(step):
                    commanded = target
                else:
                    commanded += step
                vel_i16 = max(-32768, min(32767,
                                          int(round(sign * args.rate * 1000))))
                cmd_send(bus, node, CMD_SET_INPUT_POS,
                         struct.pack('<fhh', float(commanded), vel_i16, 0))
                time.sleep(TICK)
                now = time.monotonic()
                if now - last_print > 0.5:
                    last_print = now
                    try:
                        cur, cur_vel = read_encoder(bus, node, timeout=0.15)
                        err = target - cur
                        print(f"  t={now-t_start:5.2f}s  cmd={commanded:+.4f}  "
                              f"enc={cur:+.4f}  err={err:+.4f}  "
                              f"actual_vel={cur_vel:+.3f}")
                    except TimeoutError:
                        pass
                if (now - t_start) > args.timeout:
                    print(f"TIMEOUT after {args.timeout:.1f} s — stopping.",
                          file=sys.stderr)
                    break

            # 5) Hold at final target for 0.5 s so position settles.
            print("Holding at target for 0.5 s ...")
            for _ in range(25):
                cmd_send(bus, node, CMD_SET_INPUT_POS,
                         struct.pack('<fhh', float(target), 0, 0))
                time.sleep(0.02)

            # 6) Final encoder readout.
            try:
                final_pos, final_vel = read_encoder(bus, node, timeout=0.5)
                err = target - final_pos
                print(f"FINAL: enc={final_pos:+.4f}  target={target:+.4f}  "
                      f"err={err:+.4f} turns "
                      f"(~{err * 71.047:+.1f} mm at MM_PER_REV=71.047)")
            except TimeoutError:
                print("FINAL: no encoder readback")

        except KeyboardInterrupt:
            print("\nCtrl-C — disarming.", file=sys.stderr)

        # 7) Disarm.
        print("Disarming ...")
        disarm(bus, node)
        time.sleep(0.1)

    finally:
        try:
            disarm(bus, args.node_id)
        except Exception:
            pass
        try:
            bus.shutdown()
        except Exception:
            pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
