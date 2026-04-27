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
  # Apply over USB (default). Discovers every USB-attached ODrive.
  python3 set_odrive_feedforward.py

  # Apply over CAN — talks to all 6 ODrives via can0 in one shot.
  # MUST stop stable_bot.service first (the EncoderListener / ODriveFeeder
  # share can0 and will collide with endpoint discovery):
  sudo systemctl stop stable_bot
  python3 set_odrive_feedforward.py --transport can
  sudo systemctl start stable_bot

  # Show what's currently set, change nothing:
  python3 set_odrive_feedforward.py --status [--transport can]

  # Restore previous values from the backup file:
  python3 set_odrive_feedforward.py --revert [--transport can]

Connection model
----------------
USB transport (default): every ODrive currently visible on USB is
configured. Two practical setups:

  (a) All 6 ODrives plugged into a USB hub on the Pi at the same time
      → one run does all six.
  (b) Single USB cable, swap between ODrives → run once per drive,
      the backup file accumulates entries keyed by serial number so
      --revert still covers all six.

CAN transport (--transport can): NOT SUPPORTED by odrive 0.6.x —
the Python library rewrite shipped without a CAN backend (only USB
and serial). The flag is left in place so that if a future release
restores CAN transport, this script will pick it up via the runtime
detection in `_odrive_lib_supports_can()`. For now, a clear error
message tells the user to use USB instead.

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


def discover_drives_usb(timeout_per=8.0, max_drives=6):
    """Collect every distinct ODrive (by serial) that responds within
    timeout. Calls odrive.find_any() repeatedly because the library
    returns one match at a time. Stops when a find times out OR we hit
    max_drives."""
    drives = []
    seen = set()
    print(f"discovering ODrives over USB (timeout {timeout_per}s per find, up to {max_drives})...")
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


# Path-string variants the odrive library has accepted across versions.
# The first one that connects wins; we try them in order so this script
# isn't fragile to minor library updates.
_CAN_PATH_VARIANTS = (
    "can:{iface},node_id={node}",
    "can:{iface}:{node}",
    "can:can_iface={iface},node_id={node}",
)


def _discover_one_can(node_id, iface, timeout):
    last_err = None
    for tmpl in _CAN_PATH_VARIANTS:
        path = tmpl.format(iface=iface, node=node_id)
        try:
            return odrive.find_any(path=path, timeout=timeout), path
        except Exception as e:
            last_err = e
    return None, last_err


def discover_drives_can(node_ids, iface='can0', timeout_per=8.0):
    """Connect to each node ID over the CAN bus. Requires whatever else
    was using can0 (notably stable_bot.service via the EncoderListener +
    ODriveFeeder) to have been stopped — the odrive library's endpoint
    discovery is request/response and gets confused if the bus is busy
    with other RTRs."""
    drives = []
    print(f"discovering ODrives over CAN ({iface}, node_ids={list(node_ids)}, "
          f"timeout {timeout_per}s each)...")
    for n in node_ids:
        d, path_or_err = _discover_one_can(n, iface, timeout_per)
        if d is None:
            print(f"  [n={n}] no response  ({path_or_err})")
            continue
        sn = getattr(d, 'serial_number', None)
        try:
            fw = f"{d.fw_version_major}.{d.fw_version_minor}.{d.fw_version_revision}"
        except Exception:
            fw = "?"
        sn_str = hex(sn) if sn is not None else '?'
        drives.append(d)
        print(f"  [n={n}] sn={sn_str}  fw={fw}  via {path_or_err}")
    if not drives:
        print("  (no drives reachable on CAN — is stable_bot.service stopped?)")
    return drives


def _odrive_lib_supports_can():
    """Detect whether the installed odrive Python library actually has a
    CAN backend. As of odrive 0.6.10, find_sync's signature does not
    include a path/transport argument and DeviceManager has no add_can_*
    method — the 0.6.x rewrite ships USB+serial only. Older 0.5.x had
    CAN; future versions may restore it."""
    import inspect as _inspect
    sig = _inspect.signature(odrive.find_sync)
    if 'path' in sig.parameters or 'transport' in sig.parameters:
        return True
    try:
        dm = odrive.device_manager.get_device_manager()
        if any(n.startswith('add_can') or 'can_channel' in n
               for n in dir(dm)):
            return True
    except Exception:
        pass
    return False


def discover_drives(transport, timeout_per, max_drives, can_iface, node_ids):
    if transport == 'usb':
        return discover_drives_usb(timeout_per=timeout_per, max_drives=max_drives)
    if transport == 'can':
        if not _odrive_lib_supports_can():
            sys.exit(
                "ERROR: the installed odrive Python library doesn't expose a\n"
                "CAN transport (this is the case for odrive 0.6.x — the rewrite\n"
                "shipped USB+serial only). odrivetool's --path also lists only\n"
                "usb and serial. Options:\n"
                "  1. Use --transport usb (default). Plug into each ODrive in\n"
                "     turn, or all 6 via a USB hub for one-shot.\n"
                "  2. Set the params manually with `odrivetool` over USB.\n"
                "  3. Hand-roll Tx_SDO writes via python-can — significant\n"
                "     extra code, only worth it if USB really isn't available.\n"
                "If a future odrive release restores CAN transport, this\n"
                "function's detection path will pick it up automatically.")
        ids = node_ids if node_ids else list(range(max_drives))
        return discover_drives_can(ids, iface=can_iface, timeout_per=timeout_per)
    raise ValueError(f"unknown transport: {transport!r}")


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
    p.add_argument('--transport', choices=('usb', 'can'), default='usb',
                   help='how to talk to the ODrives (default: usb)')
    p.add_argument('--can-iface', default='can0',
                   help='CAN interface name when --transport can (default: can0)')
    p.add_argument('--node-ids', type=lambda s: [int(x) for x in s.split(',')],
                   default=None,
                   help='comma-separated CAN node IDs to configure '
                        '(default: 0..max_drives-1, i.e. 0,1,2,3,4,5)')
    p.add_argument('--max-drives', type=int, default=6,
                   help='stop after finding this many distinct drives (default 6)')
    p.add_argument('--timeout', type=float, default=8.0,
                   help='per-find timeout in seconds (default 8)')
    args = p.parse_args()

    drives = discover_drives(
        transport=args.transport,
        timeout_per=args.timeout,
        max_drives=args.max_drives,
        can_iface=args.can_iface,
        node_ids=args.node_ids,
    )
    if not drives:
        if args.transport == 'usb':
            sys.exit("no ODrives found over USB. Check power + cable.")
        else:
            sys.exit("no ODrives found on CAN. Stop stable_bot.service "
                     "(`sudo systemctl stop stable_bot`) and verify can0 is up "
                     "(`ip -brief link show can0`).")

    if args.status:
        cmd_status(drives)
    elif args.revert:
        cmd_revert(drives)
    else:
        cmd_apply(drives)


if __name__ == '__main__':
    main()
