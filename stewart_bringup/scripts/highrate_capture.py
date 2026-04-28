#!/usr/bin/env python3
"""
highrate_capture.py — capture ODrive's internal control-loop signals
at the native 8 kHz rate (vs our 50 Hz /level_diag) for one axis.

Use case: the bag-level diag shows "platform doesn't settle" but
can't tell us *why* — at 50 Hz we can't see the inner loop's tracking
behaviour, current saturation, or commanded-vs-actual position lag.
Recorder runs on the ODrive itself at 8 kHz, captures the signals
you specify into a buffer on the drive, and dumps them at the end.

Workflow:
  sudo systemctl stop stable_bot   # release CAN; USB doesn't conflict but stop anyway
  # plug USB to ONE ODrive (the one you want to inspect — node 0 is fine
  # for a "well-behaved" leg, or pick a misbehaving one to compare).

  python3 highrate_capture.py --duration 5 --out /tmp/hr.csv

  # While the script is recording (`Recording for 5 s...` prints), you
  # have a window to trigger an event from another terminal — typically
  # a step test from the GUI, OR drag the platform manually, OR command
  # a Z change. The recorder captures everything the drive does during
  # that window.

  # When done it dumps a CSV with one row per 8 kHz sample, columns
  # named after the signals.

By default captures the signals most useful for diagnosing
position-tracking issues:
  axis0.pos_estimate          — actual leg position (turns)
  axis0.controller.pos_setpoint  — what the controller is commanding
  axis0.vel_estimate          — actual leg velocity (turns/s)
  axis0.controller.vel_setpoint  — commanded velocity (incl. ramp)
  axis0.controller.input_pos  — what we (stewart_control_node) set as target
  axis0.motor.foc.iq_setpoint   — current loop's commanded torque current
  axis0.motor.foc.iq_measured   — actual measured torque current

Override with --signals "comma,separated,paths" if you want different
fields. Buffer length on ODrive is fixed (~8000 samples = ~1 s at
8 kHz with 7 channels). Longer durations cause earliest samples to
be overwritten; the dumped CSV always has the most recent N samples.
"""
import argparse
import os
import sys
import time
from pathlib import Path

try:
    import odrive
    import odrive.device_manager as _dm
except ImportError:
    sys.exit(
        "ERROR: 'odrive' Python library not installed.\n"
        "  pip3 install --user --break-system-packages odrive")

try:
    # The recorder module ships with the odrive package on 0.6.x.
    from odrive.recorder import Recorder
except ImportError as e:
    sys.exit(
        f"ERROR: couldn't import odrive.recorder ({e}). "
        f"Recorder may not exist in your odrive library version. "
        f"Check `python3 -c 'import odrive; print(odrive.__file__)'`'s "
        f"sibling for recorder.py.")


DEFAULT_SIGNALS = [
    'axis0.pos_estimate',
    'axis0.controller.pos_setpoint',
    'axis0.vel_estimate',
    'axis0.controller.vel_setpoint',
    'axis0.controller.input_pos',
    'axis0.motor.foc.iq_setpoint',
    'axis0.motor.foc.iq_measured',
]


def _walk(root, dotted):
    obj = root
    for part in dotted.split('.'):
        obj = getattr(obj, part)
    return obj


def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration', type=float, default=5.0,
                   help='record for N seconds (default 5)')
    p.add_argument('--signals', default=None,
                   help='comma-separated dotted paths to capture '
                        '(default: pos/vel/iq tracking pair)')
    p.add_argument('--out', default=None,
                   help='output CSV path (default: '
                        '<repo>/tuning_data/highrate/<UTC>_<sn>.csv)')
    p.add_argument('--timeout', type=float, default=15.0,
                   help='USB find timeout (default 15)')
    p.add_argument('--countdown', type=float, default=2.0,
                   help='seconds to wait before starting recording, '
                        'so you have time to switch to the GUI (default 2)')
    args = p.parse_args()

    signals = (args.signals.split(',') if args.signals
               else list(DEFAULT_SIGNALS))
    print(f"signals to capture ({len(signals)}):")
    for s in signals:
        print(f"  {s}")

    print(f"\nconnecting to ODrive over USB (timeout {args.timeout}s)...")
    try:
        drv = odrive.find_any(timeout=args.timeout)
    except Exception as e:
        sys.exit(f"USB connect failed: {e}")
    sn = hex(drv.serial_number)
    fw = (f"{drv.fw_version_major}.{drv.fw_version_minor}."
          f"{drv.fw_version_revision}-{drv.fw_version_unreleased}")
    print(f"connected: sn={sn}  fw={fw}")

    # Resolve dotted paths to actual property objects on this drive
    refs = []
    for path in signals:
        try:
            refs.append((path, _walk(drv, path)))
        except Exception as e:
            print(f"  WARN: couldn't resolve {path}: {e}", file=sys.stderr)
    if not refs:
        sys.exit("no signals to capture — all paths failed to resolve")

    rec = Recorder(drv)
    for path, obj in refs:
        try:
            rec.add(obj)
        except Exception as e:
            print(f"  WARN: recorder.add({path}) failed: {e}", file=sys.stderr)

    if args.countdown > 0:
        for i in range(int(args.countdown), 0, -1):
            print(f"starting in {i}...", flush=True)
            time.sleep(1.0)

    print(f"\n>>> Recording for {args.duration:.1f} s. Trigger your event NOW.")
    rec.start()
    time.sleep(args.duration)
    rec.stop()
    print("<<< Recording stopped.")

    # rec.data is conventionally a dict of {name: numpy_array}, all the
    # same length. If the API differs in this lib version we'll print
    # whatever it actually returned and let the user see.
    data = getattr(rec, 'data', None)
    if data is None:
        sys.exit("Recorder.data is None — capture may have failed. "
                 "Try `dir(rec)` in odrivetool to find the right attr.")
    if not isinstance(data, dict):
        print(f"WARN: rec.data is type {type(data).__name__}, not dict — "
              f"dumping raw repr.", file=sys.stderr)
        print(repr(data)[:2000])
        sys.exit(1)

    n_samples = max((len(v) for v in data.values()), default=0)
    print(f"\ngot {n_samples} samples × {len(data)} channels "
          f"({n_samples / 8000.0:.2f} s of capture at 8 kHz)")

    # Output path
    if args.out:
        out_path = Path(args.out).expanduser()
    else:
        here = Path(__file__).resolve()
        repo = here.parent.parent.parent
        if not (repo / 'jugglebot_interfaces').exists():
            repo = Path.cwd()
        out_dir = repo / 'tuning_data' / 'highrate'
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())
        out_path = out_dir / f"{stamp}_sn{sn[2:]}.csv"

    out_path.parent.mkdir(parents=True, exist_ok=True)

    # Write CSV. Sample index → time in seconds at 8 kHz (1/8000 s per row).
    keys = list(data.keys())
    with open(out_path, 'w') as f:
        f.write('t_s,' + ','.join(keys) + '\n')
        for i in range(n_samples):
            row = [f"{i / 8000.0:.6f}"]
            for k in keys:
                arr = data[k]
                row.append(f"{float(arr[i]):.6f}" if i < len(arr) else '')
            f.write(','.join(row) + '\n')

    sz = out_path.stat().st_size
    print(f"wrote {out_path} ({sz} bytes)")
    print(f"\nDone. Cleanup:")
    print(f"  - The drive remains in normal operating state (no persistent change).")
    print(f"  - Commit the CSV to push for offline review:")
    print(f"      git add {out_path}")

    # Release USB cleanly so a follow-up odrivetool call works
    try:
        _dm.close_device_manager()
    except Exception:
        pass


if __name__ == '__main__':
    main()
