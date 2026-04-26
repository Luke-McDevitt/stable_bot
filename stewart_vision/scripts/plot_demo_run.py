#!/usr/bin/env python3
"""plot_demo_run.py — render per-demo PNG panels + an HTML index from a bag.

Usage:
  ros2 run stewart_vision plot_demo_run \\
      --bag <bag_dir> --demo {1|2|3} --out <out_dir>

Per-demo plot menus (spec §13.5):

  Demo 1 (orbit):
    - reference + measured trajectory in platform frame, time-colored
    - radial error vs time
    - tangential / phase error vs time
    - tilt commands + per-leg lengths vs time
    - latency histogram
    - V0 vs V1 detector agreement (scatter + Bland-Altman)
    - direction-reversal events as vertical bars

  Demo 2 (goto):
    - per-target trajectory traces, colored by leg
    - settling-time histogram
    - time-to-target vs Euclidean distance scatter
    - tilt commands segmented by goto leg
    - 2-D platform error heatmap

  Demo 3 (path):
    - reference path vs measured trajectory, colored by along-track
    - cross-track error vs time
    - along-track progress vs time
    - per-loop overlay (if Loop on) — repeatability is the wow plot
    - reverse events as vertical bars; trail-erase resets visible

  Common across all:
    - end-to-end latency
    - ArUco markers visible (n / 8) over time
    - KF NIS distribution
    - mono-vs-stereo ball-position discrepancy

Output: <out_dir>/index.html with embedded thumbnails + summary stats,
plus per-panel PNGs at full resolution.

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §13.5.
"""
from __future__ import annotations

import argparse
import os
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--bag', required=True,
                   help='Path to a ROS 2 bag directory.')
    p.add_argument('--demo', type=int, choices=[1, 2, 3], required=True)
    p.add_argument('--out', required=True,
                   help='Output directory; will be created.')
    args = p.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # TODO:
    #   1. Open the bag with rosbag2_py; read messages from the topics
    #      relevant to args.demo (see spec §13.5 for per-demo schemas).
    #   2. Build a pandas DataFrame keyed on message stamp.
    #   3. Demo-specific plot factory in stewart_vision._plot_demo_{1,2,3}.py
    #      (extract once it grows).
    #   4. Common panels in stewart_vision._common_panels.py.
    #   5. Render per-panel PNGs with matplotlib (or plotly for
    #      interactivity); embed in <out_dir>/index.html with summary
    #      stats (RMS errors, settling-time medians, lap repeatability,
    #      latency p50/p95).
    print(f"[scaffold] plot_demo_run --demo {args.demo} --bag {args.bag} "
          f"--out {args.out}")
    print("TODO: see comments at top of file.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
