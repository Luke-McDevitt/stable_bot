#!/usr/bin/env python3
"""capture_training_frames.py — bag training frames for YOLOv5-nano.

Records one RGB frame every 0.5 s (2 Hz default) from the OAK during a
free-rolling-ball session. Frames go to <out_dir>/img_<UTC>.jpg with a
companion frames.csv mapping filename → ROS clock + monotonic ns.

A 5–6 minute session at 2 Hz produces 600–700 frames — enough for
YOLOv5-nano to learn an orange ball on the platform under varied
lighting and ball positions.

Usage:
  python3 capture_training_frames.py --out_dir ./yolo_dataset --duration 360

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §8 (Q20).
"""
from __future__ import annotations

import argparse
import os
import sys
import time


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out_dir', required=True)
    p.add_argument('--duration', type=float, default=360.0,
                   help='Capture duration in seconds (default 360 = 6 min).')
    p.add_argument('--rate_hz', type=float, default=2.0)
    p.add_argument('--resolution', default='1080p',
                   choices=['720p', '1080p', '4k'])
    args = p.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    # TODO:
    #   1. Open OAK via DepthAI: ColorCamera at args.resolution, 30 fps.
    #   2. Open frames.csv for append.
    #   3. Loop for args.duration seconds; sample one frame every
    #      1.0/args.rate_hz seconds (use a monotonic clock; do NOT rely
    #      on the OAK queue's natural rate).
    #   4. Encode as JPEG quality 90; write img_NNNNNN_<unix_ms>.jpg.
    #   5. After the loop, print n_frames + estimated dataset size and a
    #      hint to the labeling tool.
    print(f"[scaffold] Would capture frames to {args.out_dir} for "
          f"{args.duration:.0f} s at {args.rate_hz:.1f} Hz.")
    print("TODO: see comments at top of file.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
