#!/usr/bin/env python3
"""ball_localizer_node — combine per-detector ball pixels with platform
pose to produce ball position in the platform frame.

Two parallel paths (spec §7):
  /ball_xy_mono   = ray-plane intersection from RGB pixel + platform pose
  /ball_xy_stereo = DLT triangulation of (uL, uR) — TODO, needs V1 detections

Detector selection rule (spec §8):
  1. Both V0 & V1 within 5 px AND conf >= 0.8 each → average.
  2. Exactly one with conf >= 0.8                  → use it.
  3. Both with low conf but agree within 10 px      → average (consensus).
  4. Otherwise                                       → no publication.

This scaffold implements the mono path with V0 only. V1 hooks are
present but no-op until the YOLO model lands.
"""
from __future__ import annotations

import os
from typing import Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage  # noqa: F401  (future use)
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('stewart_vision')


class BallLocalizerNode(Node):
    def __init__(self):
        super().__init__('ball_localizer')

        share = _share_dir()
        intr_path = os.path.join(share, 'config', 'oak_intrinsics.yaml')
        self.K_rgb: Optional[np.ndarray] = None
        self.dist_rgb: Optional[np.ndarray] = None
        if os.path.isfile(intr_path):
            self._load_intrinsics(intr_path)
        else:
            self.get_logger().warn(
                "No intrinsics yet; mono projection disabled until "
                "calibrate_oak.py runs.")

        self.last_v0: Optional[PointStamped] = None
        self.last_v1: Optional[PointStamped] = None
        self.last_pose: Optional[PoseStamped] = None

        self.create_subscription(
            PointStamped, '/oak/ball/v0/rgb_pixel', self._on_v0, 10)
        self.create_subscription(
            PointStamped, '/oak/ball/v1/rgb_pixel', self._on_v1, 10)
        self.create_subscription(
            PoseStamped, '/platform_pose', self._on_pose, 10)

        self.pub_xy_mono = self.create_publisher(
            PointStamped, '/ball_xy_mono', 10)
        self.pub_xy_stereo = self.create_publisher(
            PointStamped, '/ball_xy_stereo', 10)
        self.pub_diag = self.create_publisher(
            Float32MultiArray, '/oak/ball/diagnostic', 10)

        # Tick at 60 Hz; frames may not arrive at that rate but it's the
        # ceiling. Latest-wins.
        self.create_timer(1.0 / 60.0, self._tick)

    def _load_intrinsics(self, path: str):
        with open(path, 'r') as f:
            d = yaml.safe_load(f)
        # Reuse "left" intrinsics as a placeholder for the RGB cam until
        # the calibrate script supports a separate RGB block. RGB ↔
        # rectified-left mapping is one of the open TODOs.
        try:
            self.K_rgb = np.asarray(d['left']['K'], dtype=np.float64).reshape(3, 3)
            self.dist_rgb = np.asarray(d['left']['dist'], dtype=np.float64).ravel()
        except Exception as e:
            self.get_logger().error(f"Bad intrinsics: {e}")

    def _on_v0(self, msg: PointStamped):
        self.last_v0 = msg

    def _on_v1(self, msg: PointStamped):
        self.last_v1 = msg

    def _on_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def _select_pixel(self):
        """Pick a single (cx, cy) according to the spec selection rule.
        Returns (cx, cy, source_string) or (None, None, None).

        Confidence isn't on the PointStamped messages directly — the
        per-detector diagnostic topics carry it. For scaffolding we
        treat both detectors as confident if they recently published.
        TODO: subscribe to /oak/ball/v0/diagnostic and v1/diagnostic
        for real confidence values.
        """
        v0 = self.last_v0
        v1 = self.last_v1
        if v0 is None and v1 is None:
            return None, None, None
        if v0 is None:
            return float(v1.point.x), float(v1.point.y), 'v1'
        if v1 is None:
            return float(v0.point.x), float(v0.point.y), 'v0'
        dx = v1.point.x - v0.point.x
        dy = v1.point.y - v0.point.y
        if (dx * dx + dy * dy) ** 0.5 < 5.0:
            return ((v0.point.x + v1.point.x) * 0.5,
                    (v0.point.y + v1.point.y) * 0.5,
                    'avg')
        # Disagreement; for the scaffold prefer V0 (color is robust on a
        # B/W platform). The full rule needs confidences.
        return float(v0.point.x), float(v0.point.y), 'v0'

    def _project_to_platform(self, cx: float, cy: float) -> Optional[tuple]:
        """Mono ray-plane intersection: pixel → 3-D ball center on the
        platform plane. Returns (px, py, pz) in the platform frame, or
        None if the inputs are missing.

        Method:
          1. Undistort + back-project the pixel to a unit ray in the
             camera frame: r_cam = K^-1 [u, v, 1].
          2. Transform the ray and the platform-plane normal/origin from
             the camera frame using the platform pose.
          3. Solve t * r_cam = origin + s*ex + t*ey. Equivalently:
             s = -(n · cam_origin) / (n · r_cam)  for the plane through
             platform origin with normal n.

        The platform plane in the camera frame is given by the
        platform_pose's rotation: plane normal = R_cam_plat[:, 2] (z-axis
        of the platform expressed in the camera frame), plane point =
        translation column.

        TODO: account for ball radius — the visible ball center in image
        is the projection of the ball's surface, not its 3-D center. For
        a 40 mm ball at ~700 mm range the offset is ~1 mm and ignorable
        at v0.1; revisit for v1.
        """
        if self.K_rgb is None or self.last_pose is None:
            return None
        K_inv = np.linalg.inv(self.K_rgb)
        ray = K_inv @ np.array([cx, cy, 1.0])
        ray = ray / np.linalg.norm(ray)

        p = self.last_pose.pose
        # Camera->platform translation (camera frame, meters)
        t = np.array([p.position.x, p.position.y, p.position.z])
        qx, qy, qz, qw = (p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        R = _quat_to_rot(qw, qx, qy, qz)
        n = R[:, 2]  # platform z-axis in camera frame

        denom = float(n @ ray)
        if abs(denom) < 1e-6:
            return None
        s = float(n @ t) / denom  # ray scale to plane
        hit_cam = s * ray
        # Express hit in platform frame:
        delta_cam = hit_cam - t
        px_plat = R.T @ delta_cam
        return float(px_plat[0]), float(px_plat[1]), float(px_plat[2])

    def _tick(self):
        cx, cy, src = self._select_pixel()
        if cx is None:
            return
        result = self._project_to_platform(cx, cy)
        if result is None:
            return
        x_m, y_m, z_m = result

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'platform'
        out.point.x = x_m * 1000.0   # report in mm
        out.point.y = y_m * 1000.0
        out.point.z = z_m * 1000.0   # near zero by construction
        self.pub_xy_mono.publish(out)

        # Diagnostic: which detector won.
        d = Float32MultiArray()
        d.data = [
            float(self.last_v0.point.x) if self.last_v0 else float('nan'),
            float(self.last_v0.point.y) if self.last_v0 else float('nan'),
            float(self.last_v1.point.x) if self.last_v1 else float('nan'),
            float(self.last_v1.point.y) if self.last_v1 else float('nan'),
            float(cx), float(cy),
            {'v0': 0, 'v1': 1, 'avg': 2}.get(src or '', -1),
        ]
        self.pub_diag.publish(d)

        # TODO stereo: triangulate (uL, uR) with P_left, P_right from
        # rectification. Needs V1 (or grayscale) detections in BOTH mono
        # streams; currently we only have a single RGB-cam V0 detection.


def _quat_to_rot(w, x, y, z):
    """Quaternion (w, x, y, z) → rotation matrix."""
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1.0 - (yy + zz),       xy - wz,        xz + wy],
        [xy + wz,               1.0 - (xx + zz), yz - wx],
        [xz - wy,               yz + wx,        1.0 - (xx + yy)],
    ])


def main():
    rclpy.init()
    node = BallLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
