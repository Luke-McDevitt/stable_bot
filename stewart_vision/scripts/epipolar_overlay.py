#!/usr/bin/env python3
"""epipolar_overlay.py — rectification visual proof.

Loads a stereo pair (live or from disk), applies the rectification
maps from oak_intrinsics.yaml, and draws horizontal epipolar lines
across both images. After rectification, corresponding features
should lie on the same horizontal line in both views — that's the
visual proof of Lecture 6 slide 28.

Usage:
  python3 epipolar_overlay.py --live
  python3 epipolar_overlay.py --left left.png --right right.png --out overlay.png

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §7.
"""
from __future__ import annotations

import argparse
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--live', action='store_true')
    p.add_argument('--left')
    p.add_argument('--right')
    p.add_argument('--out', default='epipolar_overlay.png')
    p.add_argument('--n_lines', type=int, default=20)
    args = p.parse_args()

    # TODO:
    #   1. Load oak_intrinsics.yaml; build undistort+rectify maps for L
    #      and R via cv2.initUndistortRectifyMap.
    #   2. If --live: open OAK, grab one synced pair from MonoCamera L+R.
    #      Else: read the two PNGs.
    #   3. cv2.remap each into rectified frame.
    #   4. Concatenate side-by-side; draw args.n_lines evenly spaced
    #      horizontal lines across the full pair (red, AA).
    #   5. Save to args.out (or imshow if --live).
    #   6. Optional: also overlay 8-point + RANSAC F estimate from SIFT
    #      matches and confirm the recovered F agrees with the
    #      factory-cal F (Lecture 6 slides 34–36 self-check).
    print(f"[scaffold] Would write epipolar overlay to {args.out}.")
    print("TODO: see comments at top of file.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
