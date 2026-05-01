#!/usr/bin/env python3
"""Extract YOLO training frames from a vision-debug ROS 2 bag.

Pulls /oak/rgb/image_compressed messages from a vision-debug bag,
samples them at a configurable time interval (default: every 0.5 s),
and dumps each as a 540p JPEG ready for Roboflow / Ultralytics.

The bag's image messages are already JPEG-encoded by the OAK driver
(MJPEG bitstream → CompressedImage), so we don't decode + re-encode —
just write the bytes straight to disk. Lossless, fast.

Usage (typical, after recording via GUI's "● Record vision debug bag"):

    python3 stewart_vision/scripts/extract_v1_training_frames.py \\
        ~/stable_bot_repo/tuning_data/20260501T020000Z_vision_debug

That writes ~360 frames to:
    training_data/v1/20260501T020000Z_vision_debug/frame_0001.jpg ...

Options:
    --interval SEC   Time between sampled frames (default 0.5)
    --out DIR        Output dir root (default: training_data/v1)
    --max N          Cap output count (default: no cap)
    --skip-first SEC Skip the first N seconds of the bag (default 1.0;
                     the first second is often boot transients)

After running, zip the output directory and drag-drop into Roboflow,
or use `roboflow upload` from the CLI.

Recommended dataset composition for the V1 ball detector (~200 frames):
    ~120 frames: ball at varied positions on the platform
     ~30 frames: empty platform (markers visible, no ball)
     ~20 frames: ball + your hand visible (hard negative)
     ~15 frames: ball off the platform (table edge, on ground)
     ~15 frames: ball near platform edge / partially occluded

Capture this by recording 2-3 distinct clips and concatenating their
output directories. Or do one long take that walks through each
scenario.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def extract(bag_path: Path, out_root: Path,
            interval_s: float, max_frames: int, skip_first_s: float) -> int:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import (SequentialReader, StorageOptions,
                                ConverterOptions)
        from sensor_msgs.msg import CompressedImage
    except ImportError as e:
        print(f"ERROR: missing ROS 2 packages ({e}).\n"
              f"Run from a ROS-sourced shell: "
              f"  source /opt/ros/kilted/setup.bash", file=sys.stderr)
        return 2

    if not bag_path.is_dir():
        print(f"ERROR: bag dir not found: {bag_path}", file=sys.stderr)
        return 2

    # Pick the storage format from the metadata. Vision-debug bags
    # are MCAP (gui_server records with `-s mcap`).
    storage_id = 'mcap'
    metadata = bag_path / 'metadata.yaml'
    if metadata.is_file():
        try:
            txt = metadata.read_text()
            if 'storage_identifier: sqlite3' in txt:
                storage_id = 'sqlite3'
        except Exception:
            pass

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id=storage_id),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))

    topics_meta = reader.get_all_topics_and_types()
    rgb_topic = '/oak/rgb/image_compressed'
    if not any(t.name == rgb_topic for t in topics_meta):
        print(f"ERROR: {rgb_topic} not in bag. Topics found:",
              file=sys.stderr)
        for t in topics_meta:
            print(f"  {t.name}  ({t.type})", file=sys.stderr)
        return 2

    # Output directory mirrors the bag name.
    out_dir = out_root / bag_path.name
    out_dir.mkdir(parents=True, exist_ok=True)

    interval_ns = int(interval_s * 1e9)
    skip_first_ns = int(skip_first_s * 1e9)
    last_saved_ns = -interval_ns  # so the first eligible frame fires
    bag_start_ns: int | None = None
    saved = 0
    seen = 0

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic != rgb_topic:
            continue
        seen += 1
        if bag_start_ns is None:
            bag_start_ns = t_ns
        # Skip warm-up window.
        if t_ns - bag_start_ns < skip_first_ns:
            continue
        if t_ns - last_saved_ns < interval_ns:
            continue
        msg = deserialize_message(data, CompressedImage)
        # Sanity: format should be 'jpeg' (the OAK encoder outputs MJPEG).
        if 'jpeg' not in (msg.format or '').lower():
            print(f"WARN: msg.format={msg.format!r} — expected jpeg; "
                  f"writing raw bytes anyway", file=sys.stderr)
        out_path = out_dir / f'frame_{saved:04d}.jpg'
        out_path.write_bytes(bytes(msg.data))
        last_saved_ns = t_ns
        saved += 1
        if max_frames and saved >= max_frames:
            break

    print(f"\nDone.")
    print(f"  bag         : {bag_path}")
    print(f"  rgb msgs    : {seen}")
    print(f"  written     : {saved} frames")
    print(f"  out dir     : {out_dir}")
    print(f"  interval    : {interval_s:.2f} s "
          f"(~{1.0/max(interval_s, 1e-3):.1f} sampled fps)")
    if saved == 0:
        print("  → no frames written; check --interval / --skip-first / "
              "bag duration",
              file=sys.stderr)
        return 1
    print(f"\nNext: zip {out_dir} and upload to Roboflow, or use "
          f"`roboflow upload` from the CLI.")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='Path to a ros2 bag directory '
                                '(e.g. tuning_data/<UTC>Z_vision_debug)')
    ap.add_argument('--interval', type=float, default=0.5,
                    help='Seconds between sampled frames (default 0.5)')
    ap.add_argument('--out', default='training_data/v1',
                    help='Output root (default training_data/v1)')
    ap.add_argument('--max', type=int, default=0,
                    help='Cap output frame count (default: no cap)')
    ap.add_argument('--skip-first', type=float, default=1.0,
                    help='Skip the first N seconds of the bag (default 1.0)')
    args = ap.parse_args()
    rc = extract(Path(args.bag).expanduser().resolve(),
                 Path(args.out).expanduser().resolve(),
                 args.interval, args.max, args.skip_first)
    sys.exit(rc)


if __name__ == '__main__':
    main()
