#!/usr/bin/env python3
"""One-shot diagnostic: connect via USB, dump the contents of the
'_<name>_property' indirection objects so I can see exactly which
attribute on them holds the endpoint ID + type tag in odrive 0.6.10.

Run with one ODrive plugged into USB:
  python3 probe_odrive_property.py

Output goes to stdout — paste it back. Doesn't write any files,
doesn't change any state on the drive."""
import sys

try:
    import odrive
except ImportError:
    sys.exit("odrive Python library not installed; pip install --user --break-system-packages odrive")


def dump(label, obj):
    print(f"\n=== {label} ===")
    if obj is None:
        print("  (None)")
        return
    print(f"  type: {type(obj).__name__}  ({type(obj).__module__})")
    for k in sorted(dir(obj)):
        if k.startswith('__'):
            continue
        try:
            v = getattr(obj, k)
        except Exception as e:
            print(f"  {k}: <error: {e}>")
            continue
        if callable(v):
            continue
        print(f"  {k} = {v!r}"[:140])


def main():
    print("connecting (timeout 10s)...")
    d = odrive.find_any(timeout=10)
    print(f"connected: serial={hex(d.serial_number)}")

    # Property indirection object for a typed parameter
    parent = d.axis0.controller.config
    prop = getattr(parent, '_vel_integrator_gain_property', None)
    dump('axis0.controller.config._vel_integrator_gain_property', prop)

    # Same for the bool parameter
    parent2 = d.axis0.config.motor
    prop2 = getattr(parent2, '_wL_FF_enable_property', None)
    dump('axis0.config.motor._wL_FF_enable_property', prop2)

    # Function endpoint
    fn = getattr(d, 'save_configuration', None)
    dump('save_configuration (the SyncFunction)', fn)
    info = getattr(fn, '_info', None) if fn is not None else None
    dump('save_configuration._info', info)


if __name__ == '__main__':
    main()
