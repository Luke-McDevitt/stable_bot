#!/usr/bin/env python3
"""
set_odrive_feedforward.py — turn ODrive feedforward features on (or off)
across all attached ODrives in one shot.

Per Solomon @ ODrive: when the level loop drives fast leg moves, the
ODrive position controller has a tracking error proportional to leg
velocity (back-EMF effect) and a small steady-state position error
(no inner-loop integral). Enabling:

  controller.config.vel_integrator_gain  — adds inner-loop position-
                                           tracking integral action
  config.motor.wL_FF_enable              — back-EMF feedforward in
                                           the current loop

reduces both. This script flips them on every visible ODrive in one
go AND writes a JSON backup of the previous values so the change is
trivially reversible.

Usage:
  # Apply (default). Discovers every USB-attached ODrive and configures
  # its axis. Writes ~/.stable_bot_odrive_ff_backup.json before changing.
  python3 set_odrive_feedforward.py

  # Show what's currently set, change nothing:
  python3 set_odrive_feedforward.py --status

  # Restore previous values from the backup file:
  python3 set_odrive_feedforward.py --revert

Connection model
----------------
This connects via USB (the standard `odrivetool` transport). Two
practical setups work:

  (a) All 6 ODrives plugged into a USB hub on the Pi at the same time.
      One run of this script configures all six.

  (b) Single USB cable, swap between ODrives. Run the script once
      per drive — the backup file accumulates entries keyed by
      serial number, so --revert across all six still works.

CAN-only setups would need a different transport. odrivetool's CAN
backend changes name across versions; ask before using on a no-USB
setup.

Pre-condition: every axis must be in IDLE state before changing
config. The script aborts if any are not idle (you'd need to reboot
each ODrive to clear an active config write anyway). Run with
stable_bot.service stopped or with the level loop off.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

try:
    import odrive
    from odrive.enums import AXIS_STATE_IDLE
except ImportError:
    sys.exit(
        "ERROR: 'odrive' Python library not installed.\n"
        "  pip3 install --user odrive\n"
        "  (it's the same library odrivetool uses)")


BACKUP_PATH = Path.home() / '.stable_bot_odrive_ff_backup.json'

# (dotted path on the ODrive object, value to apply, type-cast)
TARGETS = [
    ('axis0.controller.config.vel_integrator_gain', 0.05, float),
    ('axis0.config.motor.wL_FF_enable',             True, bool),
]


def get_attr(obj, dotted):
    for p in dotted.split('.'):
        obj = getattr(obj, p)
    return obj


def set_attr(obj, dotted, value):
    parts = dotted.split('.')
    for p in parts[:-1]:
        obj = getattr(obj, p)
    setattr(obj, parts[-1], value)


def discover_drives(timeout_per=8.0, max_drives=6):
    """Collect every distinct ODrive (by serial) that responds within
    timeout. Calls odrive.find_any() repeatedly because the library
    returns one match at a time. Stops when a find times out OR we hit
    max_drives."""
    drives = []
    seen = set()
    print(f"discovering ODrives (timeout {timeout_per}s per find, up to {max_drives})...")
    while len(drives) < max_drives:
        try:
            d = odrive.find_any(timeout=timeout_per)
        except Exception:
            break
        if d is None:
            break
        sn = getattr(d, 'serial_number', None)
        if sn is None or sn in seen:
            # Same drive matched twice; we're done (no new ones available)
            break
        seen.add(sn)
        drives.append(d)
        print(f"  [{len(drives)}] sn={hex(sn)}  fw={d.fw_version_major}.{d.fw_version_minor}.{d.fw_version_revision}")
    if not drives:
        print("  (none found)")
    return drives


def all_axes_idle(drives):
    bad = []
    for i, d in enumerate(drives):
        try:
            state = d.axis0.current_state
            if state != AXIS_STATE_IDLE:
                bad.append((i, hex(d.serial_number), state))
        except Exception as e:
            bad.append((i, hex(getattr(d, 'serial_number', 0)), f"err: {e}"))
    return bad


def cmd_status(drives):
    for i, d in enumerate(drives):
        sn = hex(d.serial_number)
        print(f"\ndrive [{i}] sn={sn}")
        for path, _new, _cast in TARGETS:
            try:
                cur = get_attr(d, path)
                print(f"  {path:55s} = {cur}")
            except Exception as e:
                print(f"  {path:55s} = (read failed: {e})")


def cmd_apply(drives):
    bad = all_axes_idle(drives)
    if bad:
        print("ERROR: not all axes are IDLE — disarm the platform first:", file=sys.stderr)
        for i, sn, st in bad:
            print(f"  drive [{i}] sn={sn}: state={st}", file=sys.stderr)
        sys.exit(2)

    # Build / load existing backup so multiple runs (one drive at a time)
    # accumulate rather than overwrite.
    if BACKUP_PATH.exists():
        try:
            with open(BACKUP_PATH) as f:
                backup = json.load(f)
        except Exception:
            backup = []
    else:
        backup = []
    by_serial = {entry['serial']: entry for entry in backup if 'serial' in entry}

    for i, d in enumerate(drives):
        sn = hex(d.serial_number)
        prev = by_serial.get(sn, {'serial': sn})
        prev['captured_unix'] = time.time()
        for path, new, cast in TARGETS:
            cur = get_attr(d, path)
            # Only stash the *original* value the first time we see this drive
            if path not in prev:
                prev[path] = cur
        by_serial[sn] = prev

    with open(BACKUP_PATH, 'w') as f:
        json.dump(list(by_serial.values()), f, indent=2)
    print(f"\nbackup of pre-change values: {BACKUP_PATH}")

    for i, d in enumerate(drives):
        sn = hex(d.serial_number)
        print(f"\ndrive [{i}] sn={sn}: applying")
        for path, new, cast in TARGETS:
            old = get_attr(d, path)
            set_attr(d, path, cast(new))
            verify = get_attr(d, path)
            tag = "✓" if verify == cast(new) else "?"
            print(f"  {tag} {path}: {old} -> {verify}")
        try:
            d.save_configuration()
            print("  ✓ save_configuration()")
        except Exception as e:
            # save_configuration triggers a reboot; the connection drop is
            # normal and not a failure.
            msg = str(e).lower()
            if 'lost' in msg or 'disconnected' in msg or 'channel' in msg:
                print("  ✓ save_configuration() (drive rebooted, connection dropped — expected)")
            else:
                print(f"  ! save_configuration: {e}", file=sys.stderr)

    print("\nDone. To revert: python3", sys.argv[0], "--revert")


def cmd_revert(drives):
    if not BACKUP_PATH.exists():
        sys.exit(f"no backup found at {BACKUP_PATH}")
    with open(BACKUP_PATH) as f:
        backup = json.load(f)
    by_serial = {b['serial']: b for b in backup}

    bad = all_axes_idle(drives)
    if bad:
        print("ERROR: not all axes are IDLE — disarm first.", file=sys.stderr)
        sys.exit(2)

    for i, d in enumerate(drives):
        sn = hex(d.serial_number)
        prev = by_serial.get(sn)
        if prev is None:
            print(f"drive [{i}] sn={sn}: no backup entry, skipping")
            continue
        print(f"\ndrive [{i}] sn={sn}: reverting")
        for path, _new, cast in TARGETS:
            if path not in prev:
                continue
            old = get_attr(d, path)
            set_attr(d, path, cast(prev[path]))
            verify = get_attr(d, path)
            print(f"  {path}: {old} -> {verify}")
        try:
            d.save_configuration()
            print("  ✓ save_configuration()")
        except Exception as e:
            msg = str(e).lower()
            if 'lost' in msg or 'disconnected' in msg or 'channel' in msg:
                print("  ✓ save_configuration() (drive rebooted, connection dropped — expected)")
            else:
                print(f"  ! save_configuration: {e}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--apply', action='store_true', help='apply (default)')
    g.add_argument('--revert', action='store_true', help='restore from backup')
    g.add_argument('--status', action='store_true', help='show only')
    p.add_argument('--max-drives', type=int, default=6,
                   help='stop after finding this many distinct drives (default 6)')
    p.add_argument('--timeout', type=float, default=8.0,
                   help='per-find timeout in seconds (default 8)')
    args = p.parse_args()

    drives = discover_drives(timeout_per=args.timeout, max_drives=args.max_drives)
    if not drives:
        sys.exit("no ODrives found over USB. Check power + cable.")

    if args.status:
        cmd_status(drives)
    elif args.revert:
        cmd_revert(drives)
    else:
        cmd_apply(drives)


if __name__ == '__main__':
    main()
