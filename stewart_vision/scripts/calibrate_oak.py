#!/usr/bin/env python3
"""calibrate_oak.py — OAK-D Pro AF calibration.

READ-ONLY ON THE DEVICE: this script only ever calls
`device.readCalibration()` and `device.readFactoryCalibration()`. It
never calls `device.flashCalibration()` or `flashFactoryCalibration()`,
so the OAK's EEPROM is NOT modified. (The current Stage A / B paths
are still TODO stubs and would need an explicit flash call to
overwrite EEPROM — which they do not have.)

Usage:
  python3 calibrate_oak.py --stage factory    # DEFAULT (v6): read EEPROM
  python3 calibrate_oak.py --stage A          # custom intrinsics, both eyes
  python3 calibrate_oak.py --stage B          # custom stereo extrinsics
  python3 calibrate_oak.py --stage C          # camera-to-world via ArUco ring
  python3 calibrate_oak.py --stage all        # factory then C
  python3 calibrate_oak.py --backup-only      # dump EEPROM JSON, don't write YAML
  python3 calibrate_oak.py --report           # regenerate calibration_report.html

The OAK ships with factory-calibrated intrinsics and stereo extrinsics
on its EEPROM. `--stage factory` reads those out via DepthAI's
Device.readCalibration() and writes them to oak_intrinsics.yaml in
seconds — no chessboard captures required. It also drops timestamped
JSON backups of both the user EEPROM and the factory EEPROM into
~/oak_calibration_backups/ so the device can be re-flashed from
known-good data if anything ever goes wrong. This is the default flow
for v6.

Stages A/B are kept for users who want their own chessboard-based
calibration (tighter reprojection error, useful for the comparison
artifact). They are slow (~30 captures, 5–10 min). Skip them unless
you specifically want a tighter baseline.

Stage C (camera-to-world via the platform's ArUco ring) is always
done ourselves — the OAK has no idea where the platform is. It runs
through the GUI on the laptop in production; this CLI flag exists
for headless re-runs.

Outputs:
  config/oak_intrinsics.yaml    (factory or A+B)
  config/oak_extrinsics.yaml    (C — also written by calibration_node)
  calibration_report.html       (A + B; rectified pairs + epipolar overlay)

Acceptance gates (refuses to write below):
  per-eye reprojection error < 0.3 px       (Stage A)
  epipolar error after rectification < 1 px (Stage B)
  ArUco ring reprojection error < 2 px      (Stage C)

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md §6.
"""
from __future__ import annotations

import argparse
import datetime
import os
import sys
import textwrap

try:
    import depthai as dai
    HAVE_DEPTHAI = True
except ImportError:
    dai = None
    HAVE_DEPTHAI = False

try:
    import numpy as np
    import cv2
    import yaml
except ImportError as e:
    print(f"[calibrate_oak] Required dependency missing: {e}", file=sys.stderr)
    sys.exit(2)


# Mono streams from oak_driver_node use THE_800_P = 1280x800 native.
# StereoDepth.rectifiedLeft/Right publish at this resolution, and
# `/oak/left/image_compressed` (the stream calibration_node and
# platform_pose_node subscribe to) is the rectified left mono.
MONO_W, MONO_H = 1280, 800

# RGB pipeline in oak_driver runs 1080p source with setIspScale(1, 2)
# → effective 960x540 ISP. Match it here so the RGB intrinsics are
# valid for the V0 ball detector's pixel coordinates.
RGB_W, RGB_H = 960, 540


DEFAULT_BACKUP_DIR = os.path.expanduser('~/oak_calibration_backups')


def backup_eeproms(device, backup_dir: str) -> dict:
    """Dump full EEPROM JSON for both `readCalibration()` (current
    user cal — falls back to factory if no user cal flashed) and
    `readFactoryCalibration()` (immutable factory cal). Files land
    under `backup_dir` with an ISO-8601 timestamp + the device's MX ID
    so backups across multiple OAKs don't collide.

    NOTE: DepthAI's `eepromToJsonFile()` is purely a host-side write —
    nothing is sent back to the device.

    Returns {'user': path, 'factory': path or None, 'mx_id': str}.
    """
    os.makedirs(backup_dir, exist_ok=True)
    ts = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
    try:
        mx_id = device.getMxId() or 'unknown_mxid'
    except Exception:
        mx_id = 'unknown_mxid'
    safe_mx = ''.join(c for c in mx_id if c.isalnum() or c == '_')

    paths = {'user': None, 'factory': None, 'mx_id': mx_id}

    # User cal (== factory if device has never been re-flashed). This
    # is the calibration the runtime pipeline currently uses.
    try:
        cal_user = device.readCalibration()
        p_user = os.path.join(
            backup_dir, f'oak_eeprom_user_{safe_mx}_{ts}.json')
        cal_user.eepromToJsonFile(p_user)
        paths['user'] = p_user
    except Exception as e:
        print(f"[backup] user EEPROM dump failed: {e}", file=sys.stderr)

    # Factory cal — immutable, separately stored on EEPROM. Available
    # on most modern OAKs; on older firmware/devices the call may
    # raise, in which case we just note the absence.
    try:
        cal_factory = device.readFactoryCalibration()
        p_factory = os.path.join(
            backup_dir, f'oak_eeprom_factory_{safe_mx}_{ts}.json')
        cal_factory.eepromToJsonFile(p_factory)
        paths['factory'] = p_factory
    except Exception as e:
        print(f"[backup] factory-cal slot not separately accessible "
              f"({e}); user backup above already covers the active cal.",
              file=sys.stderr)

    return paths


def default_out_path() -> str:
    """Resolve the YAML output path.

    Prefer the ament share dir (so `colcon build --symlink-install`
    can re-sync it for runtime nodes). Fall back to the source-tree
    config dir if ament isn't available (dev machine without ROS).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('stewart_vision')
        return os.path.join(share, 'config', 'oak_intrinsics.yaml')
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(
            os.path.join(here, '..', 'config', 'oak_intrinsics.yaml'))


def stage_factory(out_path: str | None = None,
                  backup_dir: str | None = DEFAULT_BACKUP_DIR,
                  yaml_out: bool = True) -> int:
    """Read the OAK's factory calibration from EEPROM and write
    oak_intrinsics.yaml. v6 default — fastest path to a working
    pipeline.

    READ-ONLY ON THE DEVICE — only ever calls readCalibration() and
    readFactoryCalibration(). Never flashes EEPROM.

    Always dumps a timestamped JSON backup of both EEPROM slots to
    `backup_dir` first, so the device can be re-flashed from a
    known-good state if anything ever goes wrong downstream. Pass
    `backup_dir=None` to skip (only useful in CI / sandbox).
    Pass `yaml_out=False` to do the backup without writing the YAML
    (i.e. `--backup-only`).

    Schema matches what calibration_node._load_intrinsics and
    platform_pose_node._load_intrinsics expect: `left.K` is a
    9-element flat list (3x3 row-major) and `left.dist` is a
    5-element flat list (k1, k2, p1, p2, k3). Same for `right`.

    Because oak_driver_node streams the *rectified* left/right mono
    (StereoDepth.rectifiedLeft/Right), the K values written under
    `left`/`right` are the rectified intrinsics (computed via
    cv2.stereoRectify) and the dist coefficients are zero. The raw
    factory K + dist + stereo extrinsics are also retained under
    `*_raw` and `stereo` for audit / Lecture-6 comparison work.
    """
    if not HAVE_DEPTHAI:
        print("[stage factory] depthai not installed. On the Pi: "
              "`pip install --break-system-packages depthai` "
              "(or use the official venv). Confirm the OAK is on a "
              "blue USB-A 3.0 port (USB-C on Pi 5 is power-only).",
              file=sys.stderr)
        return 1

    if out_path is None:
        out_path = default_out_path()

    print(f"[stage factory] Opening OAK device for EEPROM read "
          f"(no pipeline)...")
    try:
        device = dai.Device()
    except Exception as e:
        print(f"[stage factory] Failed to open OAK: {e}", file=sys.stderr)
        print("  - Check `lsusb` for 03e7:f63b (Movidius MyriadX).",
              file=sys.stderr)
        print("  - Pi 5: cable must be in a BLUE USB-A 3.0 port.",
              file=sys.stderr)
        return 1

    try:
        device_name = device.getDeviceName() or 'OAK-D-Pro-AF'
        usb_speed = str(device.getUsbSpeed())
        cameras = [s.name for s in device.getConnectedCameras()]

        # Always back up EEPROM JSON before doing anything else.
        # Read-only on the device — eepromToJsonFile writes host-side.
        backup_paths = None
        if backup_dir is not None:
            print(f"[stage factory] Dumping EEPROM backup to {backup_dir}...")
            backup_paths = backup_eeproms(device, backup_dir)
            if backup_paths.get('user'):
                print(f"  user EEPROM    -> {backup_paths['user']}")
            if backup_paths.get('factory'):
                print(f"  factory EEPROM -> {backup_paths['factory']}")
        else:
            print("[stage factory] EEPROM backup SKIPPED "
                  "(--no-backup). The device EEPROM is still untouched.")

        if not yaml_out:
            # --backup-only path: caller wants the JSON dump and nothing
            # more. Skip the YAML composition entirely.
            print("[stage factory] --backup-only: skipping "
                  "oak_intrinsics.yaml write.")
            return 0

        cal = device.readCalibration()

        # CAM_B = mono left, CAM_C = mono right, CAM_A = RGB.
        K_l_raw = np.asarray(
            cal.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_B, MONO_W, MONO_H),
            dtype=np.float64).reshape(3, 3)
        K_r_raw = np.asarray(
            cal.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_C, MONO_W, MONO_H),
            dtype=np.float64).reshape(3, 3)
        d_l_raw = np.asarray(
            cal.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B),
            dtype=np.float64).ravel()
        d_r_raw = np.asarray(
            cal.getDistortionCoefficients(dai.CameraBoardSocket.CAM_C),
            dtype=np.float64).ravel()

        # OpenCV's standard 5-coefficient model: [k1, k2, p1, p2, k3].
        # DepthAI stores up to 14; the first 5 match this layout.
        d_l_5 = d_l_raw[:5].copy()
        d_r_5 = d_r_raw[:5].copy()

        # Stereo extrinsics: 4x4 transform from CAM_B (left) to CAM_C
        # (right). DepthAI returns translation in **centimetres**;
        # convert to metres so cv2.stereoRectify's Q matrix yields
        # depths in metres.
        ext = np.asarray(
            cal.getCameraExtrinsics(
                dai.CameraBoardSocket.CAM_B,
                dai.CameraBoardSocket.CAM_C),
            dtype=np.float64).reshape(4, 4)
        R_lr = ext[:3, :3]
        t_lr_cm = ext[:3, 3]
        t_lr_m = t_lr_cm / 100.0

        # Rectify. alpha=0 → no black borders; the rectified image
        # matches what DepthAI's StereoDepth produces.
        R_l_rect, R_r_rect, P_l, P_r, Q, roi_l, roi_r = cv2.stereoRectify(
            K_l_raw, d_l_5, K_r_raw, d_r_5,
            (MONO_W, MONO_H), R_lr, t_lr_m, alpha=0)

        # Rectified intrinsics = first 3x3 of P matrices; rectified
        # images have zero distortion by construction.
        K_l_rect = P_l[:3, :3]
        K_r_rect = P_r[:3, :3]

        # RGB camera (used for V0 ball detection on the RGB stream).
        K_rgb = np.asarray(
            cal.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, RGB_W, RGB_H),
            dtype=np.float64).reshape(3, 3)
        d_rgb = np.asarray(
            cal.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A),
            dtype=np.float64).ravel()

        baseline_mm = float(np.linalg.norm(t_lr_m) * 1000.0)
    finally:
        device.close()

    out = {
        'camera_model': device_name,
        'source': 'factory',
        'captured_at': datetime.datetime.utcnow().isoformat(timespec='seconds'),
        'usb_speed': usb_speed,
        'cameras_present': cameras,
        # Consumed by platform_pose_node + calibration_node. RAW K +
        # RAW dist, paired with the RAW (non-rectified) mono streams
        # oak_driver_node now publishes. cv2.aruco.estimatePoseBoard
        # un-distorts each detected marker corner internally during
        # solvePnP, so a pre-rectified image is not required and
        # actually hurts (the cv2.stereoRectify-derived rectified K
        # diverges from DepthAI's internal rectification — see Stage C
        # bring-up notes 2026-04-29). The rectified P_left / P_right
        # are still written under `rectification:` for future
        # stereo-triangulation work.
        'left': {
            'resolution': [MONO_W, MONO_H],
            'rectified': False,
            'K': K_l_raw.flatten().tolist(),
            'dist': d_l_raw[:5].tolist(),
            'dist_raw': d_l_raw.tolist(),
        },
        'right': {
            'resolution': [MONO_W, MONO_H],
            'rectified': False,
            'K': K_r_raw.flatten().tolist(),
            'dist': d_r_raw[:5].tolist(),
            'dist_raw': d_r_raw.tolist(),
        },
        # RGB stream is NOT rectified by oak_driver — it's raw ISP
        # output. So K/dist here are the raw factory values.
        'rgb': {
            'resolution': [RGB_W, RGB_H],
            'rectified': False,
            'K': K_rgb.flatten().tolist(),
            'dist': d_rgb[:5].tolist(),
            'dist_raw': d_rgb.tolist(),
        },
        'stereo': {
            'R': R_lr.flatten().tolist(),
            't_m': t_lr_m.tolist(),
            'baseline_mm': baseline_mm,
        },
        'rectification': {
            'R_left': R_l_rect.flatten().tolist(),
            'R_right': R_r_rect.flatten().tolist(),
            'P_left': P_l.flatten().tolist(),
            'P_right': P_r.flatten().tolist(),
            'Q': Q.flatten().tolist(),
            'roi_left': list(roi_l),
            'roi_right': list(roi_r),
        },
    }

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        yaml.safe_dump(out, f, sort_keys=False, default_flow_style=False)

    print(f"[stage factory] Wrote {out_path}")
    print(f"  device:               {device_name} (USB {usb_speed})")
    print(f"  cameras:              {cameras}")
    print(f"  left  K (raw):        fx={K_l_raw[0, 0]:.2f} "
          f"fy={K_l_raw[1, 1]:.2f} "
          f"cx={K_l_raw[0, 2]:.2f} "
          f"cy={K_l_raw[1, 2]:.2f}")
    print(f"  right K (raw):        fx={K_r_raw[0, 0]:.2f} "
          f"fy={K_r_raw[1, 1]:.2f} "
          f"cx={K_r_raw[0, 2]:.2f} "
          f"cy={K_r_raw[1, 2]:.2f}")
    print(f"  left dist[:5] (raw):  {[round(float(x), 4) for x in d_l_raw[:5]]}")
    print(f"  baseline:             {baseline_mm:.2f} mm")
    print(f"  rgb K:                fx={K_rgb[0, 0]:.2f} "
          f"fy={K_rgb[1, 1]:.2f} "
          f"cx={K_rgb[0, 2]:.2f} "
          f"cy={K_rgb[1, 2]:.2f}")
    if backup_paths is not None and (
            backup_paths.get('user') or backup_paths.get('factory')):
        print(f"  EEPROM backups:       {backup_dir}")
    print()
    print("  Restart the vision stack so platform_pose_node + "
          "calibration_node pick up the new intrinsics:")
    print("    sudo systemctl restart stable_bot.service")
    return 0


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
        Skip unless you specifically want a tighter custom baseline —
        the factory cal is well within demo tolerance.
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
    """Stage C is interactive in production — the GUI calls the
    /calibrate/{capture_frame,solve,reset} services on
    calibration_node. This CLI path exists for headless re-runs.
    """
    print(textwrap.dedent("""
        [Stage C — camera-to-world via ArUco ring]
        Stage C in production runs through the GUI:
          Vision Debug → Calibration → Capture / Solve.
        The calibration_node ROS service handlers do exactly the same
        cv2.aruco.estimatePoseBoard solve described here, gated by the
        same <2 px reprojection threshold. If you want to run it
        headlessly:
          ros2 service call /calibrate/capture_frame std_srvs/srv/Trigger
          ros2 service call /calibrate/solve         std_srvs/srv/Trigger
        See stewart_vision/stewart_vision/calibration_node.py.
    """).strip())


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--stage',
                   choices=['factory', 'A', 'B', 'C', 'all'],
                   default='factory',
                   help='factory (default): read EEPROM. A/B: custom '
                        'chessboard. C: ArUco ring. all: factory + C.')
    p.add_argument('--out',
                   default=None,
                   help='Output path for oak_intrinsics.yaml. Default: '
                        'package share dir, falling back to source tree.')
    p.add_argument('--backup-dir',
                   default=DEFAULT_BACKUP_DIR,
                   help=f'Directory for EEPROM JSON backups (read-only on '
                        f'the device). Default: {DEFAULT_BACKUP_DIR}')
    p.add_argument('--no-backup', action='store_true',
                   help='Skip the EEPROM JSON backup. Not recommended; '
                        'the device EEPROM is still untouched either way.')
    p.add_argument('--backup-only', action='store_true',
                   help='Dump EEPROM JSON backups and exit without '
                        'writing oak_intrinsics.yaml.')
    p.add_argument('--report', action='store_true',
                   help='Regenerate calibration_report.html only.')
    args = p.parse_args()

    if args.report:
        print("[report] TODO: render calibration_report.html from "
              "existing oak_intrinsics.yaml.")
        return 0

    backup_dir = None if args.no_backup else args.backup_dir

    if args.backup_only:
        return stage_factory(
            args.out, backup_dir=backup_dir, yaml_out=False)

    if args.stage == 'factory':
        return stage_factory(args.out, backup_dir=backup_dir)
    elif args.stage == 'A':
        stage_a()
    elif args.stage == 'B':
        stage_b()
    elif args.stage == 'C':
        stage_c()
    elif args.stage == 'all':
        # `all` flow: factory cal (instant) + Stage C (camera placement).
        # Stages A/B are NOT in 'all' — opt in explicitly.
        rc = stage_factory(args.out, backup_dir=backup_dir)
        if rc != 0:
            return rc
        stage_c()
    return 0


if __name__ == '__main__':
    sys.exit(main())
