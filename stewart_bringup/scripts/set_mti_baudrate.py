#!/usr/bin/env python3
"""
Change an MTi-600-series (e.g. MTi-630) UART baudrate using SetPortConfig
(XBus MID 0x8C). The older SetBaudrate (MID 0x18) is DEPRECATED on
600-series devices and will be silently ignored.

Flow:
  1. Open serial at the device's current baudrate (default 115200).
  2. Send GoToConfig, wait for GoToConfigAck.
  3. Send ReqPortConfig (MID 0x8C with no data), read 12 bytes of current
     port config (three 32-bit big-endian words - UART, RS232, other).
  4. Modify only the baud-code byte of port 0 (bits 0:7 of word 0).
  5. Send SetPortConfig (MID 0x8C with the 12 bytes) to write.
  6. Send Reset so the new baud takes effect.

After running, the MTi will be at the new baud across resets/power cycles.

Usage:
  python3 set_mti_baudrate.py                            # default port, 115200 -> 921600
  python3 set_mti_baudrate.py --current-baud 115200 --target 921600
  python3 set_mti_baudrate.py --port /dev/imu_mti630_b   # change the secondary
"""
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed; pip install --user pyserial")

# XBus message IDs
MID_WAKEUP          = 0x3E
MID_WAKEUP_ACK      = 0x3A
MID_GOTO_CONFIG     = 0x30
MID_GOTO_CONFIG_ACK = 0x31
MID_RESET           = 0x40
MID_PORT_CONFIG     = 0x8C   # same ID for request and set; data length distinguishes
MID_PORT_CONFIG_ACK = 0x8D

# Current-generation XBC codes (per xspublic/xstypes/xsbaudcode.h).
# Note: 0x80 is the LEGACY code for 921k6 - the modern code is 0x0A.
BAUD_CODE = {
    115200:  0x02,   # XBC_115k2
    230400:  0x01,   # XBC_230k4
    460800:  0x00,   # XBC_460k8
    921600:  0x0A,   # XBC_921k6
    2000000: 0x0C,   # XBC_2MegaBaud
    76800:   0x03,   # XBC_76k8
    57600:   0x04,   # XBC_57k6
    38400:   0x05,   # XBC_38k4
    28800:   0x06,   # XBC_28k8
    19200:   0x07,   # XBC_19k2
    14400:   0x08,   # XBC_14k4
    9600:    0x09,   # XBC_9k6
    4800:    0x0B,   # XBC_4k8
}


def xbus(mid, data=b''):
    body = bytes([mid, len(data)]) + data
    cs = (-sum(body)) & 0xFF
    return b'\xFA\xFF' + body + bytes([cs])


def find_frame(buf, expected_mid):
    """Scan buf for FA FF <MID> <LEN> <DATA> <CS>. Returns (data, consumed)
    or (None, 0)."""
    i = 0
    while i <= len(buf) - 5:
        if buf[i] == 0xFA and buf[i + 1] == 0xFF:
            if i + 4 > len(buf):
                return None, 0
            mid = buf[i + 2]
            length = buf[i + 3]
            if i + 4 + length + 1 > len(buf):
                return None, 0
            data = bytes(buf[i + 4:i + 4 + length])
            # We don't strictly verify the checksum; the kernel USB layer
            # is already clean enough.
            if mid == expected_mid:
                return data, i + 4 + length + 1
            i += 4 + length + 1
        else:
            i += 1
    return None, 0


def wait_for(ser, expected_mid, timeout=1.5):
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.01)
        data, _ = find_frame(buf, expected_mid)
        if data is not None:
            return data
    return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='/dev/imu_mti630')
    p.add_argument('--current-baud', type=int, default=115200,
                   help='Current baud the device is at (default 115200)')
    p.add_argument('--target', type=int, default=921600)
    args = p.parse_args()

    if args.target not in BAUD_CODE:
        sys.exit(f"unsupported target {args.target}. "
                 f"Choose from {sorted(BAUD_CODE)}")

    print(f"opening {args.port} at {args.current_baud} baud...")
    try:
        ser = serial.Serial(args.port, args.current_baud, timeout=1.0)
    except Exception as e:
        sys.exit(f"open failed: {e}")

    # FTDI DTR pulses on open; give the MTi time to boot through its
    # wakeup window and settle (in measurement mode by default).
    time.sleep(1.0)

    with ser:
        ser.reset_input_buffer()

        # 1. GoToConfig (stops measurement streaming; device enters config mode)
        print("-> GoToConfig")
        ser.write(xbus(MID_GOTO_CONFIG))
        ser.flush()
        ack = wait_for(ser, MID_GOTO_CONFIG_ACK, timeout=2.0)
        if ack is None:
            print("!! no GoToConfigAck. Either not at this baud, or the")
            print("   device is stuck. Try physically unplug/replug the USB")
            print("   cable, then re-attach via usbipd and retry.")
            return 1
        print("   config mode OK")

        # 2. ReqPortConfig (MID 0x8C, no data = read current)
        print("-> ReqPortConfig")
        ser.reset_input_buffer()
        ser.write(xbus(MID_PORT_CONFIG))
        ser.flush()
        cfg = wait_for(ser, MID_PORT_CONFIG_ACK, timeout=2.0)
        if cfg is None or len(cfg) < 12:
            print(f"!! ReqPortConfig failed (got {len(cfg) if cfg else 0} bytes)")
            return 1
        p0, p1, p2 = struct.unpack('>III', cfg[:12])
        print(f"   current: p0=0x{p0:08X}  p1=0x{p1:08X}  p2=0x{p2:08X}")
        old_code = p0 & 0xFF
        print(f"   port 0 current baud code: 0x{old_code:02X}")

        # 3. Replace only the baud-code byte of port 0 (bits 0:7)
        new_code = BAUD_CODE[args.target]
        new_p0 = (p0 & ~0xFF) | new_code
        print(f"   new p0 = 0x{new_p0:08X} (baud code 0x{new_code:02X} "
              f"= {args.target} bps)")

        # 4. SetPortConfig (MID 0x8C WITH 12 bytes of data = write)
        print("-> SetPortConfig")
        payload = struct.pack('>III', new_p0, p1, p2)
        ser.reset_input_buffer()
        ser.write(xbus(MID_PORT_CONFIG, payload))
        ser.flush()
        ack = wait_for(ser, MID_PORT_CONFIG_ACK, timeout=3.0)
        if ack is None:
            print("!! no SetPortConfigAck - write may have failed")
            return 1
        print("   port config written")

        # 5. Reset so the device comes up at the new baud
        print("-> Reset")
        ser.write(xbus(MID_RESET))
        ser.flush()
        time.sleep(0.3)

    print()
    print(f"done. MTi should now be at {args.target} baud.")
    print("Next: update xsens_mti_stewart.yaml to baudrate 921600, rebuild,")
    print("and launch.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
