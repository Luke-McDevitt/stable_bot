#!/usr/bin/env python3
"""
inventory_and_backup_odrive.py — per-drive identity + config backup.

Plug USB to one ODrive, run this script, unplug, repeat. Saves:

  <out>/odrv_node<N>_sn<SN>.txt   — sn, hw, fw, commit_hash, can_node
  <out>/odrv_node<N>_sn<SN>.json  — full odrivetool backup-config output

Both files are keyed by node + serial so the eventual restore step
(after DFU) can match each drive to its own backup unambiguously.

Default output dir: ~/odrive_backups_YYYYMMDD/
Override with --out <path>.

Implementation note: the identity read is done in a subprocess, not
inline. This is because the odrive 0.6.10 library's libodrive C++
runtime keeps USB claimed until the *process* exits — `del` + `gc`
isn't enough. Running the read as a child process lets us release
the device cleanly before invoking `odrivetool backup-config`.
"""
import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


# Inline Python that the child subprocess runs to enumerate one ODrive
# and print its identity in a parseable form. Single line per drive,
# pipe-delimited so we can split robustly.
_IDENTITY_PROBE = r"""
import odrive, sys
try:
    d = odrive.find_any(timeout=__TIMEOUT__)
except Exception as e:
    sys.exit(f"FIND_FAIL:{e}")
print("{0}|{1}.{2}|{3}.{4}.{5}|{6}|{7}|{8}".format(
    hex(d.serial_number),
    d.hw_version_major, d.hw_version_minor,
    d.fw_version_major, d.fw_version_minor, d.fw_version_revision,
    d.fw_version_unreleased,
    hex(d.commit_hash),
    d.axis0.config.can.node_id,
))
"""


def read_identity(timeout=15.0):
    """Spawn a subprocess that opens the ODrive, prints identity, and
    exits — so the parent never holds the USB handle when we later
    invoke odrivetool backup-config."""
    code = _IDENTITY_PROBE.replace('__TIMEOUT__', str(int(timeout)))
    try:
        r = subprocess.run([sys.executable, '-c', code],
                           capture_output=True, text=True,
                           timeout=timeout + 5)
    except subprocess.TimeoutExpired:
        sys.exit(f"identity read subprocess timed out (USB connect failed?)")
    if r.returncode != 0 or not r.stdout.strip():
        msg = (r.stdout + '\n' + r.stderr).strip()
        sys.exit(f"identity read failed:\n{msg[-1500:]}")
    parts = r.stdout.strip().splitlines()[-1].split('|')
    if len(parts) != 6:
        sys.exit(f"identity read returned unexpected format: {r.stdout!r}")
    return {
        'serial':       parts[0],
        'hw':           parts[1],
        'fw':           parts[2],
        'fw_unreleased': parts[3],
        'commit':       parts[4],
        'can_node':     parts[5],
    }


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    default_out = str(Path.home() / f'odrive_backups_{time.strftime("%Y%m%d")}')
    p.add_argument('--out', default=default_out,
                   help=f'output directory (default: {default_out})')
    p.add_argument('--timeout', type=float, default=15.0,
                   help='USB find timeout in seconds (default 15)')
    p.add_argument('--no-backup', action='store_true',
                   help='only write identity .txt, skip odrivetool backup-config')
    args = p.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"output dir: {out_dir}")

    print(f"reading identity (timeout {args.timeout}s, in subprocess)...")
    info = read_identity(timeout=args.timeout)
    print(f"  sn={info['serial']}  node={info['can_node']}  "
          f"hw={info['hw']}  fw={info['fw']}-{info['fw_unreleased']}  "
          f"commit={info['commit']}")

    base = f"odrv_node{info['can_node']}_sn{info['serial']}"
    ident_path = out_dir / f"{base}.txt"
    config_path = out_dir / f"{base}.json"

    with open(ident_path, 'w') as f:
        for k, v in info.items():
            f.write(f"{k}={v}\n")
        f.write(f"captured_unix={time.time()}\n")
        f.write(f"captured_iso={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}\n")
    print(f"  ✓ wrote {ident_path}")

    if args.no_backup:
        print("\n--no-backup: skipping odrivetool backup-config.")
        return

    # Brief settle so libusb fully releases between the child process
    # exiting and odrivetool re-claiming.
    time.sleep(0.5)

    print(f"  running odrivetool backup-config → {config_path} ...")
    try:
        result = subprocess.run(
            ['odrivetool', 'backup-config', str(config_path)],
            capture_output=True, text=True, timeout=120,
        )
    except FileNotFoundError:
        sys.exit("ERROR: 'odrivetool' not in PATH (expected in ~/.local/bin/).")
    except subprocess.TimeoutExpired:
        sys.exit("ERROR: odrivetool backup-config timed out after 120s "
                 "(USB still claimed? unplug + replug + retry).")

    if result.returncode != 0:
        print("  ✗ backup-config FAILED:", file=sys.stderr)
        if result.stderr.strip():
            print("    stderr:", result.stderr.strip()[-1000:], file=sys.stderr)
        if result.stdout.strip():
            print("    stdout:", result.stdout.strip()[-500:], file=sys.stderr)
        sys.exit(1)

    sz = config_path.stat().st_size if config_path.exists() else 0
    print(f"  ✓ wrote {config_path} ({sz} bytes)")
    print(f"\nDONE — drive at node {info['can_node']} "
          f"(sn {info['serial']}, fw {info['fw']}-{info['fw_unreleased']}, "
          f"commit {info['commit']}).")
    print("Unplug USB, plug to next drive, run this script again.")


if __name__ == '__main__':
    main()
