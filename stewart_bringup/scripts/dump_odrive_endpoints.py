#!/usr/bin/env python3
"""
dump_odrive_endpoints.py — extract the endpoint IDs we need to write
ODrive parameters via CAN, by USB-connecting to ONE drive once.

Endpoint IDs are baked into firmware at build time. They're identical
across all ODrives running the same firmware version, so doing this
once for one of your 6 ODrive Pros is enough — everything else can
go via CAN forever after.

Output: stewart_bringup/data/odrive_endpoints.json, with the IDs that
set_odrive_feedforward_via_can.py reads at runtime.

Usage:
  # Plug ONE USB cable to ANY of your 6 ODrives.
  sudo systemctl stop stable_bot   # so the bus / drives aren't busy
  python3 dump_odrive_endpoints.py

  # The JSON gets written to stewart_bringup/data/odrive_endpoints.json
  # and is meant to be committed. Don't run this again unless the
  # ODrive firmware changes.

The script is intentionally verbose. If it can't find an endpoint ID
through any of the reflection paths it tries, it dumps the parent
object's attribute table so we can patch the extraction logic without
needing another USB-connect cycle.
"""
import argparse
import json
import os
import sys
from pathlib import Path

try:
    import odrive
except ImportError:
    sys.exit(
        "ERROR: 'odrive' Python library not installed.\n"
        "  pip3 install --user --break-system-packages odrive")


# Paths we need IDs for. The (path, expected_type) pairs let us
# also grab the type info into the JSON, which the CAN write needs
# to format the value correctly.
TARGETS = [
    ('axis0.controller.config.vel_integrator_gain', 'float32'),
    ('axis0.config.motor.wL_FF_enable',             'bool'),
    # Function endpoint — invoked by writing to it (any value).
    ('save_configuration',                          'function'),
]

# Reflection paths the odrive library has used across versions to
# expose the underlying endpoint ID. We try each in order; the first
# that yields a non-None integer wins. Add new ones here when this
# script fails to find an ID — the verbose dump in --on-fail makes it
# obvious which attribute name to add.
_ID_ATTRS = (
    '_id', '_endpoint_id', '__id__',
    '_remote_attr_id', 'endpoint_id', 'id',
)
_PARENT_TABLE_ATTRS = (
    '_remote_attributes', '_endpoint_table', '_endpoints',
    '__remote_attributes__', '__endpoints__',
)


def _walk(root, dotted):
    """Walk a dotted path on `root`, returning (parent_obj, leaf_name)
    where leaf_name is the final attr we want to find an ID for."""
    parts = dotted.split('.')
    obj = root
    for p in parts[:-1]:
        obj = getattr(obj, p)
    return obj, parts[-1]


def _extract_id(parent, leaf_name):
    """Try several reflection paths to extract the endpoint ID for
    `parent.leaf_name`. Returns (id, where_we_found_it) or (None, ...)."""
    # 1. Look in the parent's "remote attributes" table by name.
    for table_attr in _PARENT_TABLE_ATTRS:
        table = getattr(parent, table_attr, None)
        if table is None:
            continue
        try:
            entry = table[leaf_name]
        except (KeyError, TypeError):
            continue
        # Found the entry; pull an ID off it.
        for id_attr in _ID_ATTRS:
            v = getattr(entry, id_attr, None)
            if isinstance(v, int):
                return v, f'parent.{table_attr}["{leaf_name}"].{id_attr}'
        if isinstance(entry, int):
            return entry, f'parent.{table_attr}["{leaf_name}"]'
    # 2. Get the leaf as an object and look for an ID directly on it.
    try:
        leaf = getattr(parent, leaf_name)
    except Exception:
        leaf = None
    if leaf is not None:
        for id_attr in _ID_ATTRS:
            v = getattr(leaf, id_attr, None)
            if isinstance(v, int):
                return v, f'leaf.{id_attr}'
        # Walk leaf.__dict__ for anything int-named like an id
        d = getattr(leaf, '__dict__', None) or {}
        for k, v in d.items():
            if 'id' in k.lower() and isinstance(v, int):
                return v, f'leaf.__dict__["{k}"]'
    # 3. The library may store endpoints on the *parent* under a name
    #    derived from leaf_name (e.g. _vel_integrator_gain or similar).
    for prefix in ('', '_'):
        for cand in (f'{prefix}{leaf_name}_id',
                     f'{prefix}{leaf_name}_endpoint_id'):
            v = getattr(parent, cand, None)
            if isinstance(v, int):
                return v, f'parent.{cand}'
    return None, None


def _diagnostic_dump(parent, leaf_name):
    """Print everything we can see on the parent — used when we can't
    find the ID, so we have data to fix the extraction next iteration."""
    print(f"\n  -- diagnostic dump (couldn't find id for {leaf_name}) --")
    print(f"     parent type: {type(parent).__name__!s}")
    print(f"     parent module: {type(parent).__module__!s}")
    attrs = sorted(n for n in dir(parent)
                   if not n.startswith('__'))
    interesting = [n for n in attrs
                   if 'endpoint' in n.lower() or 'remote' in n.lower()
                   or 'attr' in n.lower() or 'id' in n.lower()
                   or 'tree' in n.lower()]
    print(f"     interesting parent attrs: {interesting}")
    if hasattr(parent, '__dict__'):
        keys = sorted(parent.__dict__.keys())
        underscores = [k for k in keys if k.startswith('_')]
        print(f"     parent.__dict__ underscore keys: {underscores}")
    try:
        leaf = getattr(parent, leaf_name)
        print(f"     leaf type: {type(leaf).__name__!s}")
        leaf_attrs = sorted(n for n in dir(leaf) if not n.startswith('__'))
        print(f"     leaf attrs: {leaf_attrs}")
    except Exception as e:
        print(f"     leaf access failed: {e}")


def _firmware_version(drv):
    try:
        return f"{drv.fw_version_major}.{drv.fw_version_minor}.{drv.fw_version_revision}"
    except Exception:
        return "unknown"


def _hw_version(drv):
    try:
        return f"{drv.hw_version_major}.{drv.hw_version_minor}-{drv.hw_version_variant}"
    except Exception:
        return "unknown"


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--out', default=None,
                   help='output path (default: <repo>/stewart_bringup/data/'
                        'odrive_endpoints.json)')
    p.add_argument('--timeout', type=float, default=10.0,
                   help='USB connect timeout (s)')
    args = p.parse_args()

    # Default out path: locate the repo, drop a JSON in stewart_bringup/data.
    if args.out:
        out_path = Path(args.out).expanduser().resolve()
    else:
        here = Path(__file__).resolve()
        # scripts → stewart_bringup → repo root → tuning_data is a sibling
        repo = here.parent.parent.parent
        if not (repo / 'jugglebot_interfaces').exists():
            repo = Path.cwd()
        out_path = repo / 'stewart_bringup' / 'data' / 'odrive_endpoints.json'
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"connecting to ODrive over USB (timeout {args.timeout}s)...")
    print("  (plug ONE USB cable to any of the 6 drives — they all run the same firmware)")
    try:
        drv = odrive.find_any(timeout=args.timeout)
    except Exception as e:
        sys.exit(f"USB connect failed: {e}")

    fw = _firmware_version(drv)
    hw = _hw_version(drv)
    sn = hex(getattr(drv, 'serial_number', 0))
    print(f"connected: serial={sn}  hw={hw}  fw={fw}")

    out = {
        'firmware_version': fw,
        'hw_version': hw,
        'extracted_from_serial': sn,
        'extracted_at_unix': __import__('time').time(),
        'endpoints': {},
    }
    failures = []
    for path, kind in TARGETS:
        try:
            parent, leaf = _walk(drv, path)
        except Exception as e:
            print(f"\n[{path}] could not walk path: {e}")
            failures.append(path)
            continue
        ep_id, source = _extract_id(parent, leaf)
        if ep_id is None:
            print(f"\n[{path}] could not extract endpoint id")
            _diagnostic_dump(parent, leaf)
            failures.append(path)
            continue
        out['endpoints'][path] = {'id': ep_id, 'type': kind}
        print(f"[{path}] id={ep_id}  type={kind}  (via {source})")

    print(f"\nwriting {out_path}")
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)

    if failures:
        sys.exit(
            f"\nWARN: {len(failures)} endpoint(s) failed extraction: {failures}\n"
            "Paste the diagnostic dumps above and I'll patch the script "
            "without needing another USB-connect cycle.")
    print("\nDone. Commit the JSON and the CAN-only configurator can take "
          "it from here:\n  set_odrive_feedforward_via_can.py")


if __name__ == '__main__':
    main()
