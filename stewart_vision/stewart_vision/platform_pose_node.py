#!/usr/bin/env python3
"""platform_pose_node — recover the platform's pose from the ArUco ring.

Subscribes to a rectified mono stream (left, by default) and detects
the marker ring board defined in `marker_layout.yaml`. Publishes:

    /platform_pose (geometry_msgs/PoseStamped) @ ~30 Hz, frame: oak_left
    /platform_pose/markers_visible (std_msgs/Int32) — count, for debug

The pose is the camera-to-platform transform: position + orientation
of the platform-frame origin expressed in the camera frame.

When intrinsics aren't on disk yet (oak_intrinsics.yaml missing),
the node logs a warning every few seconds and skips publication —
without K, ArUco pose is meaningless.

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md (§4, §5, §7).
"""
from __future__ import annotations

import os
from typing import Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

try:
    import cv2
except ImportError:
    cv2 = None

from stewart_vision._aruco_helpers import (
    load_marker_layout, get_aruco_dictionary, build_board)


def _share_dir() -> str:
    """Resolve the package share dir at runtime."""
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('stewart_vision')


class PlatformPoseNode(Node):
    def __init__(self):
        super().__init__('platform_pose')
        if cv2 is None:
            self.get_logger().fatal("opencv-contrib-python required.")
            raise SystemExit(2)

        share = _share_dir()
        layout_path = os.path.join(share, 'config', 'marker_layout.yaml')
        intr_path = os.path.join(share, 'config', 'oak_intrinsics.yaml')

        self.layout = load_marker_layout(layout_path)
        self.aruco_dict = get_aruco_dictionary(self.layout)
        self.board = build_board(self.layout)

        # Detector parameters tuned for matte vinyl prints under IR
        # illumination (markers are darker; default thresholds clip).
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.detect_params = cv2.aruco.DetectorParameters()
        else:
            self.detect_params = cv2.aruco.DetectorParameters_create()

        self.K: Optional[np.ndarray] = None
        self.dist: Optional[np.ndarray] = None
        if os.path.isfile(intr_path):
            self._load_intrinsics(intr_path)
        else:
            self.get_logger().warn(
                f"No intrinsics yet at {intr_path}. Run "
                f"`python3 scripts/calibrate_oak.py --stage A` first.")

        self.create_subscription(
            CompressedImage, '/oak/left/image_compressed',
            self._on_image, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/platform_pose', 10)
        self.pub_n = self.create_publisher(
            Int32, '/platform_pose/markers_visible', 10)

        self._last_warn = 0.0

    def _load_intrinsics(self, path: str):
        with open(path, 'r') as f:
            d = yaml.safe_load(f)
        try:
            self.K = np.asarray(d['left']['K'], dtype=np.float64).reshape(3, 3)
            self.dist = np.asarray(d['left']['dist'], dtype=np.float64).ravel()
            self.get_logger().info(
                f"Loaded left-cam intrinsics from {path}")
        except Exception as e:
            self.get_logger().error(f"Bad intrinsics yaml: {e}")
            self.K = None
            self.dist = None

    def _on_image(self, msg: CompressedImage):
        if self.K is None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_warn > 5.0:
                self.get_logger().warn("Skipping pose: no intrinsics loaded.")
                self._last_warn = now
            return

        arr = np.frombuffer(bytearray(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
        if frame is None:
            return

        # Detect markers
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.detect_params)
            corners, ids, _ = detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame, self.aruco_dict, parameters=self.detect_params)

        n_msg = Int32()
        n_msg.data = int(0 if ids is None else len(ids))
        self.pub_n.publish(n_msg)

        if ids is None or len(ids) < 3:
            return  # need at least 3 markers for a stable pose

        # Pose-from-board: returns rvec, tvec for the board origin in the
        # camera frame. The board origin = platform-frame origin by
        # construction (markers were placed symmetrically around it).
        rvec = np.zeros(3, dtype=np.float64)
        tvec = np.zeros(3, dtype=np.float64)
        try:
            ok, rvec, tvec = cv2.aruco.estimatePoseBoard(
                corners, ids, self.board, self.K, self.dist, rvec, tvec)
        except Exception as e:
            self.get_logger().warn(f"estimatePoseBoard failed: {e}")
            return
        if not ok:
            return

        R, _ = cv2.Rodrigues(rvec)
        # Convert rotation matrix to quaternion (xyzw).
        qw, qx, qy, qz = _rot_to_quat(R)

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'oak_left'
        out.pose.position.x = float(tvec[0])
        out.pose.position.y = float(tvec[1])
        out.pose.position.z = float(tvec[2])
        out.pose.orientation.x = float(qx)
        out.pose.orientation.y = float(qy)
        out.pose.orientation.z = float(qz)
        out.pose.orientation.w = float(qw)
        self.pub_pose.publish(out)


def _rot_to_quat(R: np.ndarray):
    """Rotation matrix → (w, x, y, z). Branchless Shoemake form."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(w), float(x), float(y), float(z)


def main():
    rclpy.init()
    node = PlatformPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
