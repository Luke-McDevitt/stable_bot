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


_CODEC_NAME_TO_TYPE = {
    # ODrive's codec_name → our JSON 'type' tag (matches what the CAN
    # script's _pack_value / _unpack_value expect).
    'float':  'float32',
    'float32': 'float32',
    'float64': 'float64',
    'bool':   'bool',
    'uint8':  'uint8',
    'uint16': 'uint16',
    'uint32': 'uint32',
    'int8':   'int8',
    'int16':  'int16',
    'int32':  'int32',
}


def _extract_id_and_type(parent, leaf_name):
    """Get (endpoint_id, type_tag, source_string) for a parameter on
    `parent` named `leaf_name`. Returns (None, None, None) on failure.

    odrive 0.6.10 (sync_tree / async_tree) stores property metadata
    behind a `_<leaf_name>_property` sibling on the parent — the leaf
    itself, when accessed, fetches and returns the typed value. Function
    endpoints behave differently: the leaf IS the function object, and
    its metadata lives on `leaf._info`. We try both."""
    # Path A: typed property — parent._<leaf_name>_property._info
    prop = getattr(parent, f'_{leaf_name}_property', None)
    if prop is not None:
        info = getattr(prop, '_info', None)
        if info is not None:
            ep_id = getattr(info, 'endpoint_id', None)
            codec_name = getattr(info, 'codec_name', None)
            if isinstance(ep_id, int):
                kind = _CODEC_NAME_TO_TYPE.get(codec_name, codec_name or 'unknown')
                return ep_id, kind, f'parent._{leaf_name}_property._info'

    # Path B: function endpoint — leaf._info
    try:
        leaf = getattr(parent, leaf_name)
    except Exception:
        leaf = None
    if leaf is not None:
        info = getattr(leaf, '_info', None)
        if info is not None:
            ep_id = getattr(info, 'endpoint_id', None)
            if isinstance(ep_id, int):
                # FunctionInfo has inputs/outputs lists; PropertyInfo
                # has codec_name. Distinguish them by the first.
                if hasattr(info, 'inputs') and hasattr(info, 'outputs'):
                    return ep_id, 'function', 'leaf._info (function)'
                codec_name = getattr(info, 'codec_name', None)
                kind = _CODEC_NAME_TO_TYPE.get(codec_name, codec_name or 'unknown')
                return ep_id, kind, 'leaf._info'

    # Path C (fallbacks): older library layouts that exposed the id
    # directly on the leaf or on a remote-attributes table.
    if leaf is not None:
        for id_attr in _ID_ATTRS:
            v = getattr(leaf, id_attr, None)
            if isinstance(v, int):
                return v, 'unknown', f'leaf.{id_attr}'
    for table_attr in _PARENT_TABLE_ATTRS:
        table = getattr(parent, table_attr, None)
        if table is None:
            continue
        try:
            entry = table[leaf_name]
        except (KeyError, TypeError):
            continue
        for id_attr in _ID_ATTRS:
            v = getattr(entry, id_attr, None)
            if isinstance(v, int):
                return v, 'unknown', f'parent.{table_attr}["{leaf_name}"].{id_attr}'

    return None, None, None


# Back-compat shim for callers that only care about the id.
def _extract_id(parent, leaf_name):
    ep_id, _kind, source = _extract_id_and_type(parent, leaf_name)
    return ep_id, source


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
    for path, expected_kind in TARGETS:
        try:
            parent, leaf = _walk(drv, path)
        except Exception as e:
            print(f"\n[{path}] could not walk path: {e}")
            failures.append(path)
            continue
        ep_id, kind_seen, source = _extract_id_and_type(parent, leaf)
        if ep_id is None:
            print(f"\n[{path}] could not extract endpoint id")
            _diagnostic_dump(parent, leaf)
            failures.append(path)
            continue
        # Prefer the type discovered from the device (PropertyInfo.codec_name)
        # over the type we expected — they should match, but the device is
        # the source of truth.
        kind = kind_seen if kind_seen and kind_seen != 'unknown' else expected_kind
        out['endpoints'][path] = {'id': ep_id, 'type': kind}
        if expected_kind != 'function' and kind_seen and kind_seen != expected_kind:
            print(f"  WARN: expected type {expected_kind}, device reports {kind_seen}")
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
