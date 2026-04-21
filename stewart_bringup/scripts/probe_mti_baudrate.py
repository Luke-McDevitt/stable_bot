#!/usr/bin/env python3
"""
Probe the MTi: open the FTDI port ONCE, try each baud by setting
serial.baudrate (avoids re-resetting the MTi on every attempt via DTR).
Listens passively first, then sends GoToConfig if nothing streams.
"""
import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed; pip install --user pyserial")

BAUD_RATES_TO_TRY = [921600, 115200, 460800, 230400, 2000000, 57600, 38400, 19200, 9600]


def xbus_frame(mid, data=b''):
    body = bytes([mid, len(data)]) + data
    checksum = (-sum(body)) & 0xFF
    return bytes([0xFA, 0xFF]) + body + bytes([checksum])


def try_baud(ser, baud):
    ser.baudrate = baud
    ser.reset_input_buffer()
    # 1) Listen passively for any streaming data for 300 ms
    t_end = time.monotonic() + 0.3
    buf = bytearray()
    while time.monotonic() < t_end:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        else:
            time.sleep(0.02)
    idx = buf.find(b'\xFA\xFF')
    if idx >= 0:
        return True, f"streaming {len(buf)} bytes, first frame at byte {idx}"
    # 2) Active probe: send GoToConfig, look for an ack
    ser.reset_input_buffer()
    ser.write(xbus_frame(0x30))  # GoToConfig
    ser.flush()
    time.sleep(0.2)
    n = ser.in_waiting
    if n:
        resp = ser.read(n)
        if b'\xFA\xFF' in resp:
            return True, f"GoToConfig ack, got {len(resp)} bytes: {resp[:16].hex()}"
        return False, f"reply {len(resp)} bytes, no preamble: {resp[:16].hex()}"
    return False, "no streaming, no reply to GoToConfig"


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--port', default='/dev/imu_mti630')
    args = p.parse_args()

    print(f"opening {args.port} once and cycling baud rates...")
    try:
        ser = serial.Serial(args.port, 115200, timeout=0.3)
    except Exception as e:
        sys.exit(f"open failed: {e}")

    # Give FTDI time to settle after the open-triggered DTR pulse
    time.sleep(0.5)

    hits = []
    with ser:
        for baud in BAUD_RATES_TO_TRY:
            ok, info = try_baud(ser, baud)
            flag = "YES" if ok else "no"
            print(f"  {baud:>8} : {flag}  ({info})")
            if ok:
                hits.append(baud)

    print()
    if hits:
        print(f"device responding at: {hits}")
    else:
        print("!! no response at any baud after single-open probe.")
        print("   Try usbipd detach + attach from Windows, then retry.")


if __name__ == '__main__':
    main()
