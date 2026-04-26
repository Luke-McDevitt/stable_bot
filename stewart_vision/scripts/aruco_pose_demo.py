#!/usr/bin/env python3
"""aruco_pose_demo.py — standalone ArUco-board pose sanity check.

Opens the OAK's left mono stream, runs the same detection pipeline as
platform_pose_node, and overlays the recovered platform-frame axes on
each frame in a cv2.imshow window. Useful to verify Stage A
intrinsics + the marker layout before launching the full stack.

Usage:
  python3 aruco_pose_demo.py
  python3 aruco_pose_demo.py --image /path/to/frame.png   # offline mode

Exit with 'q'.

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §6.
"""
from __future__ import annotations

import argparse
import os
import sys

print(__doc__)

# TODO:
#   1. Load marker_layout.yaml + oak_intrinsics.yaml.
#   2. Build the cv2.aruco.Board.
#   3. If --image: read once, detect, draw axes, save annotated PNG.
#   4. Else: open OAK via DepthAI, loop on frames, draw axes,
#      cv2.imshow, exit on 'q'.
#   5. Print mean reprojection error in the corner of the frame.
#
# The implementation mirrors platform_pose_node.py — once that node is
# fleshed out, factor the detection logic into stewart_vision._aruco_helpers
# and import it from both places.
print("[scaffold] aruco_pose_demo.py — TODO: see comments at top of file.")
sys.exit(0)
