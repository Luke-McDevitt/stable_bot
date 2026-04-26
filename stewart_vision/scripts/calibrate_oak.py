#!/usr/bin/env python3
"""calibrate_oak.py — three-stage OAK-D Pro AF calibration.

Usage:
  python3 calibrate_oak.py --stage A          # intrinsics, both eyes
  python3 calibrate_oak.py --stage B          # stereo extrinsics + rectification
  python3 calibrate_oak.py --stage C          # camera-to-world via ArUco ring
  python3 calibrate_oak.py --stage all        # A then B then C
  python3 calibrate_oak.py --report           # regenerate calibration_report.html

Stage A and B are slow (~30 captures, 5–10 minutes); they almost never
need to be re-run. Stage C is fast (~30 seconds) and MUST be re-run
every time the camera arm is repositioned.

Outputs:
  config/oak_intrinsics.yaml    (A + B)
  config/oak_extrinsics.yaml    (C)
  calibration_report.html       (A + B; rectified pairs + epipolar overlay)

Acceptance gates (refuses to write below):
  per-eye reprojection error < 0.3 px
  epipolar error after rectification < 1 px
  ArUco ring reprojection error < 2 px (Stage C)

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §6.

This is a SCAFFOLD. The chessboard capture loop, OpenCV calibrateCamera /
stereoCalibrate / stereoRectify calls, and the HTML report rendering
are implemented in stub form below — add real OAK frame capture and
write the YAMLs once the camera is in hand.
"""
from __future__ import annotations

import argparse
import sys
import textwrap


def stage_a():
    """TODO: Capture ~30 chessboard pairs from left + right monos.
    Run cv2.calibrateCamera per eye. Write K, dist to oak_intrinsics.yaml.
    """
    print(textwrap.dedent("""
        [Stage A — intrinsics]
        TODO:
          1. Open OAK via DepthAI; queue MonoCamera left + right at 800p.
          2. Detect 9x6 chessboard (25 mm squares) in each pair; require
             ≥ 30 distinct poses for a stable solve.
          3. cv2.calibrateCamera per eye. Reject if reprojection > 0.3 px.
          4. Write left.K, left.dist, right.K, right.dist into
             config/oak_intrinsics.yaml.
    """).strip())


def stage_b():
    """TODO: cv2.stereoCalibrate (CALIB_FIX_INTRINSIC) + stereoRectify.
    Write R_LR, t_LR, P_left, P_right, Q. Render calibration_report.html.
    """
    print(textwrap.dedent("""
        [Stage B — stereo extrinsics + rectification]
        TODO:
          1. Re-use Stage A captures.
          2. cv2.stereoCalibrate(..., flags=CALIB_FIX_INTRINSIC).
          3. cv2.stereoRectify → projection matrices + Q.
          4. Verify epipolar error < 1 px after rectification.
          5. Write oak_intrinsics.yaml stereo block.
          6. Render calibration_report.html with rectified pairs and
             horizontal epipolar lines (Lecture 6 slide 28 visual proof).
    """).strip())


def stage_c():
    """TODO: with platform leveled, run aruco.estimatePoseBoard against
    the ring (marker_layout.yaml gives 3-D corner positions). The
    resulting transform IS T_cam_world. Write to oak_extrinsics.yaml.
    """
    print(textwrap.dedent("""
        [Stage C — camera-to-world via ArUco ring]
        TODO:
          1. Confirm platform is leveled (IMU 0/0 ± 0.1°).
          2. Capture one frame from rectified left mono.
          3. cv2.aruco.estimatePoseBoard with marker_layout.yaml as the
             board definition and Stage A's K_left / dist_left.
          4. Reproject all 32 ring corners; require mean error < 2 px.
          5. Write rvec, tvec to config/oak_extrinsics.yaml.
        This is the only stage that runs after a camera arm move.
    """).strip())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--stage', choices=['A', 'B', 'C', 'all'], default='all')
    p.add_argument('--report', action='store_true',
                   help='Regenerate calibration_report.html only.')
    args = p.parse_args()

    if args.report:
        print("[report] TODO: render calibration_report.html from "
              "existing oak_intrinsics.yaml.")
        return 0

    if args.stage in ('A', 'all'):
        stage_a()
    if args.stage in ('B', 'all'):
        stage_b()
    if args.stage in ('C', 'all'):
        stage_c()
    return 0


if __name__ == '__main__':
    sys.exit(main())
