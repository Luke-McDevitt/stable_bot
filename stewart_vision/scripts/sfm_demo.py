#!/usr/bin/env python3
"""sfm_demo.py — Structure-from-Motion pipeline (Lecture 6 slides 41–42).

Pan the OAK by hand (or load N images of a static scene), and recover
the camera trajectory using:
  1. SIFT / ORB feature detection
  2. KNN feature matching with Lowe's ratio test
  3. F via 8-point + RANSAC
  4. E = K^T F K
  5. Camera pose (R, t) from E via SVD + cheirality test
  6. Triangulation of correspondences

Outputs: a 3-D plot of recovered camera centers + sparse point cloud.

Usage:
  python3 sfm_demo.py --frames frames/*.png
  python3 sfm_demo.py --live --capture 8

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §7
("standalone debug routine").

This is the demo that visibly proves to the professor that we
understand the full classical-SfM pipeline rather than relying on
the OAK's built-in stereo. Worth getting right.
"""
from __future__ import annotations

import argparse
import sys


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--frames', nargs='+',
                   help='Globbed image paths (≥ 2).')
    p.add_argument('--live', action='store_true')
    p.add_argument('--capture', type=int, default=8,
                   help='With --live: number of frames to capture.')
    p.add_argument('--features', choices=['sift', 'orb'], default='sift')
    p.add_argument('--out_dir', default='./sfm_out')
    args = p.parse_args()

    # TODO:
    #   1. Load oak_intrinsics.yaml left.K (the SfM target is the RGB
    #      cam; reuse left intrinsics for the scaffold).
    #   2. Load N frames (live capture or from --frames).
    #   3. Pairwise feature matching (SIFT or ORB), Lowe ratio < 0.75.
    #   4. For each consecutive pair: F via cv2.findFundamentalMat
    #      (FM_RANSAC, 1.0 px). Confirm rank-2.
    #   5. E = K^T F K; cv2.recoverPose for cheirality + (R, t).
    #   6. Triangulate matched points with cv2.triangulatePoints; chain
    #      poses to global frame.
    #   7. Render a matplotlib 3-D plot of camera centers + point cloud.
    #   8. Save plot + a JSON of the recovered trajectory to args.out_dir.
    print(f"[scaffold] sfm_demo.py — would run on {args.frames or 'live OAK'}.")
    print("TODO: see comments at top of file.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
