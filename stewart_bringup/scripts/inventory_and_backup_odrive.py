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

This skips the copy-paste of placeholders that the manual procedure
required — script reads the identity itself and computes the right
filename automatically.
"""
import argparse
import gc
import os
import subprocess
import sys
import time
from pathlib import Path

try:
    import odrive
except ImportError:
    sys.exit(
        "ERROR: 'odrive' Python library not installed.\n"
        "  pip3 install --user --break-system-packages odrive")


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    default_out = str(Path.home() / f'odrive_backups_{time.strftime("%Y%m%d")}')
    p.add_argument('--out', default=default_out,
                   help=f'output directory (default: {default_out})')
    p.add_argument('--timeout', type=float, default=15.0,
                   help='USB find timeout in seconds (default 15)')
    args = p.parse_args()

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"output dir: {out_dir}")
    print(f"connecting to ODrive over USB (timeout {args.timeout}s)...")

    try:
        drv = odrive.find_any(timeout=args.timeout)
    except Exception as e:
        sys.exit(f"USB connect failed: {e}")

    info = {
        'serial': hex(drv.serial_number),
        'hw': f"{drv.hw_version_major}.{drv.hw_version_minor}",
        'fw': f"{drv.fw_version_major}.{drv.fw_version_minor}.{drv.fw_version_revision}",
        'fw_unreleased': drv.fw_version_unreleased,
        'commit': hex(drv.commit_hash),
        'can_node': drv.axis0.config.can.node_id,
    }
    print(f"  sn={info['serial']}  node={info['can_node']}  "
          f"hw={info['hw']}  fw={info['fw']}-{info['fw_unreleased']}  "
          f"commit={info['commit']}")

    base = f"odrv_node{info['can_node']}_sn{info['serial']}"
    ident_path = out_dir / f"{base}.txt"
    config_path = out_dir / f"{base}.json"

    # Write identity .txt — so we can match firmware version + commit_hash
    # across all 6 drives by scanning the directory afterwards.
    with open(ident_path, 'w') as f:
        for k, v in info.items():
            f.write(f"{k}={v}\n")
        f.write(f"captured_unix={time.time()}\n")
        f.write(f"captured_iso={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}\n")
    print(f"  ✓ wrote {ident_path}")

    # Release the USB connection before shelling out to odrivetool —
    # otherwise libusb sees the device as already-claimed and the
    # subprocess fails to enumerate.
    drv = None
    gc.collect()
    time.sleep(1.0)

    print(f"  running odrivetool backup-config → {config_path} ...")
    try:
        result = subprocess.run(
            ['odrivetool', 'backup-config', str(config_path)],
            capture_output=True, text=True, timeout=120,
        )
    except FileNotFoundError:
        sys.exit("ERROR: 'odrivetool' not in PATH. Should be in ~/.local/bin/.")
    except subprocess.TimeoutExpired:
        sys.exit("ERROR: odrivetool backup-config timed out after 120s.")

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
