#!/usr/bin/env python3
"""calibration_node — Stage C (camera-to-world) ArUco-ring calibration.

This node is the GUI-facing calibration entry point. It exposes ONLY
the three Stage C operations as ROS services:

    /calibrate/capture_frame  (Trigger) — accept one ArUco-ring frame
    /calibrate/solve          (Trigger) — average accepted frames,
                                          run estimatePoseBoard,
                                          write oak_extrinsics.yaml
    /calibrate/reset          (Trigger) — clear captured frame buffer

Stage C is the operation that matters between demo runs — every time
the camera arm is repositioned, the camera-to-world transform must be
re-solved. It is fast (~1 frame typically suffices when the platform
is leveled), and it does NOT touch the OAK's factory-tuned intrinsics
or stereo extrinsics.

Stages A and B (custom intrinsics / stereo recalibration) are NOT
exposed by this node — they live behind the `calibrate_oak.py` CLI
on the Pi. This is intentional: Stages A/B *override* the OAK's
factory tuning, and a misclick in the GUI could replace a working
factory cal with a half-baked one. See spec §6 for the rationale.

Subscribes:
  /oak/left/image_compressed (sensor_msgs/CompressedImage) — the
      mono left stream where the ArUco ring is detected.

Publishes:
  /calibrate/status (std_msgs/String JSON @ 5 Hz) — live status for
      the GUI: number of frames captured, last-frame markers seen,
      last-frame reprojection error.

Services:
  /calibrate/capture_frame, /calibrate/solve, /calibrate/reset
      (all std_srvs/Trigger). Result message returns a human-readable
      string that the GUI displays as a toast.

Spec: ../../stewart_bringup/docs/closed_loop_ball_demos.md §6, §11.1, §12.
"""
from __future__ import annotations

import datetime
import json
import os
import threading
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import cv2
except ImportError:
    cv2 = None

from stewart_vision._aruco_helpers import (
    load_marker_layout, get_aruco_dictionary, build_board, MarkerLayout)


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('stewart_vision')


@dataclass
class CapturedFrame:
    rvec: np.ndarray            # (3,) rotation vector, camera frame
    tvec: np.ndarray            # (3,) translation, camera frame, meters
    n_markers: int              # how many markers were detected
    reproj_err_px: float        # mean per-corner reprojection error
    timestamp_iso: str
    detected_ids: List[int] = field(default_factory=list)


# Acceptance thresholds (spec §6 Stage C)
MIN_MARKERS = 3                 # need ≥ 3 markers for a stable pose
MAX_REPROJ_ERR_PX = 2.0         # write-gate: <2 px mean reprojection
TARGET_REPROJ_ERR_PX = 1.0      # green-zone for the GUI status badge


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        if cv2 is None:
            self.get_logger().fatal(
                "opencv-contrib-python not installed (cv2.aruco missing).")
            raise SystemExit(2)

        share = _share_dir()
        layout_path = os.path.join(share, 'config', 'marker_layout.yaml')
        intr_path = os.path.join(share, 'config', 'oak_intrinsics.yaml')

        # Marker layout: which IDs are where in the platform frame.
        self.layout: MarkerLayout = load_marker_layout(layout_path)
        self.aruco_dict = get_aruco_dictionary(self.layout)
        self.board = build_board(self.layout)

        # ArUco detector params — same defaults as platform_pose_node.
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.detect_params = cv2.aruco.DetectorParameters()
        else:
            self.detect_params = cv2.aruco.DetectorParameters_create()

        # Intrinsics: must exist before Stage C is meaningful. Either
        # the user ran `calibrate_oak.py --stage factory` (default) or
        # `--stage A,B` (advanced). Without this file we refuse.
        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        if os.path.isfile(intr_path):
            self._load_intrinsics(intr_path)
        else:
            self.get_logger().warn(
                f"No intrinsics yet at {intr_path}. Run "
                f"`python3 calibrate_oak.py --stage factory` on the Pi "
                f"first — Stage C cannot proceed without K + dist.")

        # Output: where we write the camera-to-world transform.
        self.extr_path = os.path.join(share, 'config', 'oak_extrinsics.yaml')

        # State: latest decoded image + accepted frame buffer.
        self._lock = threading.Lock()
        self._latest_frame_gray: Optional[np.ndarray] = None
        self._latest_corners = None
        self._latest_ids = None
        self._captured: List[CapturedFrame] = []

        # ROS interface.
        self.create_subscription(
            CompressedImage, '/oak/left/image_compressed',
            self._on_image, qos_profile_sensor_data)

        self.create_service(
            Trigger, '/calibrate/capture_frame', self._srv_capture)
        self.create_service(
            Trigger, '/calibrate/solve', self._srv_solve)
        self.create_service(
            Trigger, '/calibrate/reset', self._srv_reset)

        self.pub_status = self.create_publisher(
            String, '/calibrate/status', 5)
        self.create_timer(0.2, self._publish_status)   # 5 Hz

        self.get_logger().info(
            f"calibration_node ready. {self.layout.num_markers} markers "
            f"in layout (dictionary {self.layout.dictionary_name}).")

    # ---- intrinsics ---------------------------------------------------------

    def _load_intrinsics(self, path: str):
        with open(path, 'r') as f:
            d = yaml.safe_load(f)
        try:
            self.K = np.asarray(
                d['left']['K'], dtype=np.float64).reshape(3, 3)
            self.dist = np.asarray(
                d['left']['dist'], dtype=np.float64).ravel()
            src = d.get('source', '?')
            self.get_logger().info(
                f"Loaded left intrinsics from {path} (source: {src}).")
        except Exception as e:
            self.get_logger().error(
                f"oak_intrinsics.yaml parse error: {e}")

    # ---- image stream -------------------------------------------------------

    def _on_image(self, msg: CompressedImage):
        """Decode + detect markers; cache latest detection so capture
        can access it without re-doing the work.
        """
        if self.K is None:
            return
        arr = np.frombuffer(bytearray(msg.data), dtype=np.uint8)
        gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            return

        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.detect_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.detect_params)

        with self._lock:
            self._latest_frame_gray = gray
            self._latest_corners = corners
            self._latest_ids = ids

    # ---- helpers ------------------------------------------------------------

    def _solve_pose(self, corners, ids):
        """Run estimatePoseBoard on the cached detections. Returns
        (success, rvec, tvec, mean_reproj_err_px).
        """
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.zeros(3, dtype=np.float64)
        try:
            ok, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners, ids, self.board, self.K, self.dist, rvec, tvec)
        except Exception as e:
            return False, None, None, float('inf'), str(e)
        if not ok:
            return False, None, None, float('inf'), "pose solver returned 0"

        # Reprojection error: project all known corners and compare.
        # The detected corners came from cv2.aruco.detectMarkers; we
        # need to align them to the layout's known 3-D positions by ID.
        id_to_obj = {int(self.layout.ids[i]): self.layout.corners_m[i]
                     for i in range(self.layout.num_markers)}
        all_obj = []
        all_img = []
        for i, mid in enumerate(ids.ravel().tolist()):
            if mid not in id_to_obj:
                continue
            all_obj.append(id_to_obj[mid])         # (4, 3)
            all_img.append(corners[i].reshape(4, 2))   # (4, 2)
        if not all_obj:
            return False, None, None, float('inf'), "no known IDs detected"

        obj_pts = np.concatenate(all_obj, axis=0).astype(np.float32)
        img_pts = np.concatenate(all_img, axis=0).astype(np.float32)
        proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, self.K, self.dist)
        proj = proj.reshape(-1, 2)
        err_px = float(np.mean(np.linalg.norm(proj - img_pts, axis=1)))
        return True, rvec, tvec, err_px, None

    def _publish_status(self):
        with self._lock:
            n_cap = len(self._captured)
            last_n_markers = (
                0 if self._latest_ids is None
                else int(len(self._latest_ids)))
            last_err = (self._captured[-1].reproj_err_px
                        if self._captured else None)

        status = {
            'frames_captured': n_cap,
            'last_frame_markers_visible': last_n_markers,
            'markers_required': MIN_MARKERS,
            'last_frame_reproj_err_px': last_err,
            'extrinsics_path': self.extr_path,
            'extrinsics_written': os.path.isfile(self.extr_path),
            'intrinsics_loaded': self.K is not None,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    # ---- services -----------------------------------------------------------

    def _srv_capture(self, req: Trigger.Request, resp: Trigger.Response):
        if self.K is None:
            resp.success = False
            resp.message = (
                "no intrinsics loaded — run "
                "`python3 calibrate_oak.py --stage factory` on the Pi.")
            return resp

        with self._lock:
            corners = self._latest_corners
            ids = self._latest_ids

        if ids is None or len(ids) < MIN_MARKERS:
            n = 0 if ids is None else len(ids)
            resp.success = False
            resp.message = (
                f"only {n}/{self.layout.num_markers} ArUco markers visible — "
                f"need >= {MIN_MARKERS}. Adjust angle or lighting.")
            return resp

        ok, rvec, tvec, err_px, why = self._solve_pose(corners, ids)
        if not ok:
            resp.success = False
            resp.message = f"pose solve failed: {why}"
            return resp

        if err_px > MAX_REPROJ_ERR_PX:
            resp.success = False
            resp.message = (
                f"reprojection error {err_px:.2f} px exceeds "
                f"{MAX_REPROJ_ERR_PX:.2f} px gate — frame rejected. "
                f"Try a different angle or improve lighting.")
            return resp

        cap = CapturedFrame(
            rvec=rvec.ravel(),
            tvec=tvec.ravel(),
            n_markers=int(len(ids)),
            reproj_err_px=err_px,
            timestamp_iso=datetime.datetime.utcnow().isoformat(timespec='seconds'),
            detected_ids=[int(x) for x in ids.ravel().tolist()],
        )
        with self._lock:
            self._captured.append(cap)

        resp.success = True
        resp.message = (
            f"frame {len(self._captured)} accepted — "
            f"{cap.n_markers}/{self.layout.num_markers} markers, "
            f"reproj {err_px:.2f} px"
            f"{' (excellent)' if err_px < TARGET_REPROJ_ERR_PX else ''}.")
        return resp

    def _srv_solve(self, req: Trigger.Request, resp: Trigger.Response):
        with self._lock:
            captured = list(self._captured)

        if not captured:
            resp.success = False
            resp.message = "no frames captured yet."
            return resp

        # Average rvec/tvec across captured frames. For a static board
        # under a static camera, frame-to-frame jitter is small, so
        # the mean is a fine estimator. For the rotation we average
        # in axis-angle space (acceptable for this jitter scale; for
        # tighter work, average quaternions or do a joint solve).
        rvecs = np.stack([c.rvec for c in captured])
        tvecs = np.stack([c.tvec for c in captured])
        rvec_mean = rvecs.mean(axis=0)
        tvec_mean = tvecs.mean(axis=0)
        err_mean = float(np.mean([c.reproj_err_px for c in captured]))

        if err_mean > MAX_REPROJ_ERR_PX:
            resp.success = False
            resp.message = (
                f"averaged reprojection error {err_mean:.2f} px exceeds "
                f"{MAX_REPROJ_ERR_PX:.2f} px gate — REFUSING to write "
                f"oak_extrinsics.yaml. Reset and recapture.")
            return resp

        # Compose the YAML.
        R, _ = cv2.Rodrigues(rvec_mean.reshape(3, 1))
        out = {
            'source': 'stage_c_aruco_ring',
            'captured_at': datetime.datetime.utcnow().isoformat(timespec='seconds'),
            'frames_used': len(captured),
            'mean_reproj_err_px': err_mean,
            # Camera-to-platform pose: position + rotation of the
            # platform-frame origin expressed in the camera frame.
            # Because the platform is leveled and centered on world
            # origin, this IS T_cam_world.
            'camera_to_world': {
                'rvec': rvec_mean.tolist(),
                'tvec_m': tvec_mean.tolist(),
                'R': R.tolist(),
            },
            'detected_ids_per_frame': [c.detected_ids for c in captured],
        }
        try:
            os.makedirs(os.path.dirname(self.extr_path), exist_ok=True)
            with open(self.extr_path, 'w') as f:
                yaml.safe_dump(out, f, sort_keys=False)
        except Exception as e:
            resp.success = False
            resp.message = f"failed to write {self.extr_path}: {e}"
            return resp

        # Clear the buffer after a successful solve so a subsequent
        # round of captures doesn't accidentally include stale frames.
        with self._lock:
            self._captured.clear()

        resp.success = True
        resp.message = (
            f"oak_extrinsics.yaml written — {len(captured)} frame(s), "
            f"mean reproj {err_mean:.2f} px. Buffer cleared.")
        return resp

    def _srv_reset(self, req: Trigger.Request, resp: Trigger.Response):
        with self._lock:
            n = len(self._captured)
            self._captured.clear()
        resp.success = True
        resp.message = f"cleared {n} captured frame(s)."
        return resp


def main():
    rclpy.init()
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
