#!/usr/bin/env python3
"""ball_localizer_node — combine per-detector ball pixels with platform
pose to produce ball position in the platform frame.

Two parallel paths (spec §7):
  /ball_xy_mono   = ray-plane intersection from RGB pixel + platform pose
  /ball_xy_oak    = SpatialLocationCalculator depth at RGB ROI, then
                    transform to platform frame. Truth signal that the
                    controller closes on. (oak_driver_node publishes the
                    raw 3D point on /oak/ball/spatial; we transform.)
  /ball_xy_stereo = DLT triangulation of (uL, uR) — TODO, needs V1
                    detections in both monos.

Detector selection rule (spec §8):
  1. Both V0 & V1 within 5 px AND conf >= 0.8 each → average.
  2. Exactly one with conf >= 0.8                  → use it.
  3. Both with low conf but agree within 10 px     → average (consensus).
  4. Otherwise                                     → no publication.

V1 hooks are present but no-op until the YOLO model lands.

Frame conventions:
  RGB camera frame (CAM_A): OpenCV +x right, +y down, +z forward.
  Mono-left frame (CAM_B):  same axes, different origin.
  Platform frame: from the ArUco-board pose that platform_pose_node
                  publishes. /platform_pose lives in the **RGB**
                  camera frame as of 2026-04-29 — ArUco was moved
                  off the mono-left stream to dodge the IR-projector
                  speckle that was breaking corner detection.

Bridging:
  V0 detects in RGB-frame pixels.
  /platform_pose is also in RGB frame.
  → No cross-camera transform; the projection collapses to
    "undistort RGB pixel → ray in RGB → intersect platform plane
    (in RGB frame) → express in platform frame".
  T_rgb_to_mono is kept in the code as identity for now so the math
  paths read uniformly; if Stage C ever moves back to mono (e.g.,
  IR-strobing solution), this single matrix is the toggle point.
"""
from __future__ import annotations

import os
import time
from typing import Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage  # noqa: F401  (future use)
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray

try:
    import cv2
except ImportError:
    cv2 = None


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('stewart_vision')


class BallLocalizerNode(Node):
    def __init__(self):
        super().__init__('ball_localizer')

        if cv2 is None:
            self.get_logger().fatal(
                "opencv-contrib-python required (cv2.undistortPoints).")
            raise SystemExit(2)

        share = _share_dir()
        intr_path = os.path.join(share, 'config', 'oak_intrinsics.yaml')
        # RGB intrinsics for the V0/V1 ball-pixel detector inputs.
        self.K_rgb: Optional[np.ndarray] = None
        self.dist_rgb: Optional[np.ndarray] = None
        # 4x4 transform that takes a point in RGB camera frame and
        # returns it in mono-left camera frame. /platform_pose is in
        # mono-left frame, so we have to bridge.
        self.T_rgb_to_mono: Optional[np.ndarray] = None
        if os.path.isfile(intr_path):
            self._load_intrinsics(intr_path)
        else:
            self.get_logger().warn(
                "No intrinsics yet; mono projection disabled until "
                "calibrate_oak.py runs.")

        # ArUco→IMU 2x2 rotation from the IVA Procrustes fit. When
        # present, applied to ball xy in platform frame so the
        # BALL_TRACK loop sees IMU-aligned x/y instead of
        # ArUco-board-frame x/y. Solves the sign-convention chase
        # that plagued Demo 1/2 tuning. None = identity (no cal yet).
        self.M_aruco_to_imu: Optional[np.ndarray] = None
        self._load_aruco_imu_alignment()

        self.last_v0: Optional[PointStamped] = None
        # Receipt-time of the last v0/v1 detection so _select_pixel
        # can drop stale cache. When the ball falls off the platform,
        # the V0 detector stops firing — but without this freshness
        # check, _select_pixel returns the cached last_v0 forever and
        # the loop reacts to a ball that's long gone.
        self._last_v0_t = 0.0
        self._last_v1_t = 0.0
        # Ball-loss timeout. Older than this, we treat the cache as
        # invalid. Slightly longer than the typical V0 publish gap
        # (~70 ms at 14 fps) but well under the controller's 0.5 s
        # state-stale threshold.
        self._pixel_stale_s = 0.4
        self.last_v1: Optional[PointStamped] = None
        self.last_depth_pixel: Optional[PointStamped] = None
        self.last_pose: Optional[PoseStamped] = None

        # Diagnostic counters — emitted every 5 s so we can tell why
        # /ball_xy_depth (or /ball_xy_mono) is silent without ros2
        # topic echo. Increments wherever a detection arrives, where
        # we publish, and where the on-platform gate kicks in.
        self._n_depth_in = 0
        self._n_depth_out = 0
        self._n_depth_gate_rejects = 0
        self._n_v0_in = 0
        self._n_v0_out = 0
        self._n_v0_gate_rejects = 0
        self.create_timer(5.0, self._log_localizer_health)

        self.create_subscription(
            PointStamped, '/oak/ball/v0/rgb_pixel', self._on_v0, 10)
        self.create_subscription(
            PointStamped, '/oak/ball/v1/rgb_pixel', self._on_v1, 10)
        self.create_subscription(
            PoseStamped, '/platform_pose', self._on_pose, 10)
        # Depth-based 3D ball position in RGB-camera frame, from
        # oak_driver_node's SpatialLocationCalculator. Spec §7 truth.
        self.create_subscription(
            PointStamped, '/oak/ball/spatial', self._on_spatial, 10)
        # Depth-blob detector pixel — independent of V0. Projected
        # to platform frame the same way V0 is, then published to
        # /ball_xy_depth so the GUI can compare both detectors with
        # a centered ball.
        self.create_subscription(
            PointStamped, '/oak/ball/depth/rgb_pixel',
            self._on_depth_pixel, 10)

        self.pub_xy_mono = self.create_publisher(
            PointStamped, '/ball_xy_mono', 10)
        self.pub_xy_oak = self.create_publisher(
            PointStamped, '/ball_xy_oak', 10)
        self.pub_xy_stereo = self.create_publisher(
            PointStamped, '/ball_xy_stereo', 10)
        self.pub_xy_depth = self.create_publisher(
            PointStamped, '/ball_xy_depth', 10)
        self.pub_diag = self.create_publisher(
            Float32MultiArray, '/oak/ball/diagnostic', 10)
        # Per-stage VISION latency probe: [capture→localizer-in ms,
        # capture→mono-out ms], published with each /ball_xy_mono. Lets the
        # digest split the detect→state gap into transport (capture→in − the
        # detector's own v0_lat) vs localizer tick+reproject (out − in) vs
        # KF (state − out). Additive; no existing consumer.
        self.pub_xy_mono_lat = self.create_publisher(
            Float32MultiArray, '/ball_xy_mono/lat', 10)
        self._last_v0_recv_s = 0.0   # ROS-clock receive time of last v0
        self._last_v1_recv_s = 0.0
        self._last_pub_t = 0.0       # monotonic time of last mono publish
        self._last_lat_cap_ns = -1   # capture stamp of last latency probe

        # Tick at 60 Hz as the steady republish cadence (unchanged). On top
        # of it we ALSO publish the instant a new detection arrives
        # (_on_v0/_on_v1 → _tick, debounced) so a fresh sighting doesn't
        # wait up to a tick period — cutting the localizer's contribution to
        # the detect→state latency without removing the proven 60 Hz path.
        self.create_timer(1.0 / 60.0, self._tick)

    def _load_aruco_imu_alignment(self):
        """Load the 2x2 rotation that maps ArUco-derived (cam_roll,
        cam_pitch) into IMU body frame. Computed by digest_iva_bag.py
        from an IVA sweep and copied here by gui_server's "Apply"
        button. Loaded once at startup; null when no calibration
        has been applied yet (loop runs in the ArUco frame, with
        whatever sign-misalignment that implies)."""
        share = _share_dir()
        path = os.path.join(share, 'config', 'aruco_imu_alignment.yaml')
        if not os.path.isfile(path):
            self.get_logger().info(
                "no aruco_imu_alignment.yaml — running ArUco-frame; "
                "Demo gains will need sign correction by hand. Run "
                "an IVA sweep + Apply to remove sign assumptions.")
            return
        try:
            with open(path) as f:
                d = yaml.safe_load(f) or {}
            m = d.get('matrix')
            if (m and len(m) == 2 and len(m[0]) == 2):
                self.M_aruco_to_imu = np.asarray(m, dtype=np.float64)
                self.get_logger().info(
                    f"loaded ArUco→IMU alignment: "
                    f"rot={d.get('rotation_deg', 0):+.1f}° "
                    f"det={d.get('det', 0):+.2f} "
                    f"rms={d.get('post_alignment_rms_deg', 0):.2f}° "
                    f"from {d.get('source_bag', '?')}")
            else:
                self.get_logger().warn(
                    f"{path} present but no usable 'matrix' field")
        except Exception as e:
            self.get_logger().warn(f"failed to load alignment: {e}")

    def _apply_aruco_imu_rotation(self, p_plat):
        """Apply the 2x2 alignment to (x, y); leave z untouched.
        Pass-through if no alignment is loaded."""
        if self.M_aruco_to_imu is None:
            return p_plat
        xy = self.M_aruco_to_imu @ np.array(
            [float(p_plat[0]), float(p_plat[1])])
        return np.array([float(xy[0]), float(xy[1]), float(p_plat[2])])

    def _load_intrinsics(self, path: str):
        with open(path, 'r') as f:
            d = yaml.safe_load(f)
        try:
            # Use the RGB block — V0 detects in RGB-frame pixels.
            rgb = d.get('rgb')
            if rgb is None:
                # Older YAML schema fallback (Phase A bring-up era):
                # treat 'left' as RGB. Not strictly correct (different
                # sensor + lens) but better than nothing.
                self.get_logger().warn(
                    "oak_intrinsics.yaml has no 'rgb' block; falling "
                    "back to 'left' intrinsics. Re-run "
                    "`calibrate_oak.py --stage factory` to fix.")
                rgb = d['left']
            self.K_rgb = np.asarray(
                rgb['K'], dtype=np.float64).reshape(3, 3)
            self.dist_rgb = np.asarray(
                rgb['dist'], dtype=np.float64).ravel()

            # /platform_pose now lives in the RGB camera frame, same
            # frame as V0 detections and SLC output (since 2026-04-29:
            # ArUco moved off mono left to dodge IR-projector
            # contamination). The transform between "ball-detection
            # frame" and "platform_pose frame" is therefore identity
            # — the cam_a_to_cam_b extrinsics in the YAML are still
            # the correct factory RGB→mono transform but no longer
            # part of the projection chain.
            self.T_rgb_to_mono = np.eye(4, dtype=np.float64)
        except Exception as e:
            self.get_logger().error(f"Bad intrinsics: {e}")

    def _on_v0(self, msg: PointStamped):
        self.last_v0 = msg
        self._last_v0_t = time.monotonic()
        self._last_v0_recv_s = self.get_clock().now().nanoseconds * 1e-9
        self._maybe_publish_event()

    def _on_v1(self, msg: PointStamped):
        self.last_v1 = msg
        self._last_v1_t = time.monotonic()
        self._last_v1_recv_s = self.get_clock().now().nanoseconds * 1e-9
        self._maybe_publish_event()

    def _maybe_publish_event(self):
        """Event-driven freshness: republish /ball_xy_mono the instant a new
        detection lands, instead of waiting up to one 60 Hz tick. Debounced
        (≥5 ms) so the event and the periodic tick can't double-fire back to
        back. The single-threaded executor serialises this with the timer,
        so there's no re-entrancy with _tick."""
        if time.monotonic() - self._last_pub_t >= 0.005:
            self._tick()

    def _on_pose(self, msg: PoseStamped):
        self.last_pose = msg

    def _on_depth_pixel(self, msg: PointStamped):
        """Depth-blob detector pixel → ray-plane intersection →
        /ball_xy_depth in platform-frame mm. Same projection path as
        V0 so the two outputs are directly comparable; on-platform
        gate inside _project_to_platform applies."""
        self.last_depth_pixel = msg
        self._n_depth_in += 1
        cx = float(msg.point.x)
        cy = float(msg.point.y)
        result = self._project_to_platform(cx, cy)
        if result is None:
            self._n_depth_gate_rejects += 1
            return
        x_m, y_m, z_m = result
        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'platform'
        out.point.x = x_m * 1000.0
        out.point.y = y_m * 1000.0
        out.point.z = z_m * 1000.0
        self.pub_xy_depth.publish(out)
        self._n_depth_out += 1

    def _log_localizer_health(self):
        """Periodic snapshot of accept/reject counts so we can tell
        whether /ball_xy_depth silence is "pixel never arrived",
        "projection rejected (gate / no pose / no intrinsics)", or
        "publish succeeded but downstream not subscribed"."""
        self.get_logger().info(
            f"[health] v0: in={self._n_v0_in} out={self._n_v0_out} "
            f"reject={self._n_v0_gate_rejects} | "
            f"depth: in={self._n_depth_in} out={self._n_depth_out} "
            f"reject={self._n_depth_gate_rejects} | "
            f"K_rgb={'YES' if self.K_rgb is not None else 'NO'} "
            f"pose={'YES' if self.last_pose is not None else 'NO'}")

    def _select_pixel(self):
        """Pick a single (cx, cy) according to the spec selection rule.
        Returns (cx, cy, source_string) or (None, None, None).

        Confidence isn't on the PointStamped messages directly — the
        per-detector diagnostic topics carry it. For scaffolding we
        treat both detectors as confident if they recently published.
        TODO: subscribe to /oak/ball/v0/diagnostic and v1/diagnostic
        for real confidence values.
        """
        # Drop stale cache: if the detector hasn't fired recently,
        # treat the slot as None so we don't keep "seeing" a ball
        # that fell off the platform minutes ago.
        now = time.monotonic()
        v0 = self.last_v0
        v1 = self.last_v1
        if v0 is not None and (now - self._last_v0_t) > self._pixel_stale_s:
            v0 = None
        if v1 is not None and (now - self._last_v1_t) > self._pixel_stale_s:
            v1 = None
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
        """Mono ray-plane intersection: RGB pixel → 3-D ball position
        on the platform plane. Returns (px_m, py_m, pz_m) in the
        platform frame (METRES), or None if inputs are missing.

        Method:
          1. Undistort the RGB pixel via cv2.undistortPoints (handles
             the 14-coeff rational model DepthAI calibrates with).
             Result is a 2-D point in normalized camera coordinates
             (z=1 plane, no distortion).
          2. Build the corresponding unit ray in RGB camera frame.
          3. Transform the ray (direction + origin) into mono-left
             camera frame via the cam_a_to_cam_b extrinsics.
          4. Intersect that ray with the platform plane, which is
             defined in mono-left frame by /platform_pose:
                plane normal n = R_pose[:, 2]
                plane point  p = pose.position
          5. Express the hit point in platform frame.

        TODO: account for ball radius — the visible ball center in
        image is the projection of the ball's surface centroid, not
        its 3-D centre. For a 40 mm ball at ~700 mm range the offset
        is ~1 mm and ignorable; revisit if it shows up in residuals.
        """
        if self.K_rgb is None or self.last_pose is None:
            return None
        if self.T_rgb_to_mono is None:
            return None

        # Undistort the pixel to normalized camera coordinates. The
        # output is in z=1 plane of the RGB camera frame, distortion
        # already removed. Pass the full 14-coeff dist vector so
        # cv2 uses the rational model (matches DepthAI calibration).
        pts_in = np.array([[[cx, cy]]], dtype=np.float64)
        pts_norm = cv2.undistortPoints(
            pts_in, self.K_rgb, self.dist_rgb).reshape(2)
        # Unit ray in RGB camera frame.
        ray_rgb = np.array([pts_norm[0], pts_norm[1], 1.0])
        ray_rgb = ray_rgb / np.linalg.norm(ray_rgb)

        # Transform ray direction into mono-left frame. Translation
        # affects the ORIGIN, not the direction, so apply only the
        # rotation block. Origin in mono-left = the RGB camera's
        # origin expressed in mono-left = T_rgb_to_mono.translation.
        R_a_to_b = self.T_rgb_to_mono[:3, :3]
        t_a_to_b = self.T_rgb_to_mono[:3, 3]
        ray_mono = R_a_to_b @ ray_rgb
        origin_mono = t_a_to_b.copy()

        # Platform plane in mono-left frame from /platform_pose.
        p = self.last_pose.pose
        t_pose = np.array(
            [p.position.x, p.position.y, p.position.z])
        qx, qy, qz, qw = (p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        R_pose = _quat_to_rot(qw, qx, qy, qz)
        n = R_pose[:, 2]

        # Plane: n^T (P - t_pose) = 0
        # Ray:   P = origin_mono + s * ray_mono
        # Solve: s = n^T (t_pose - origin_mono) / (n^T ray_mono)
        denom = float(n @ ray_mono)
        if abs(denom) < 1e-6:
            return None
        s = float(n @ (t_pose - origin_mono)) / denom
        if s <= 0:
            # Plane is behind the camera — bad pose or pixel.
            return None
        hit_mono = origin_mono + s * ray_mono

        # Express hit in platform frame.
        delta = hit_mono - t_pose
        p_plat = R_pose.T @ delta
        # Apply the IVA-derived ArUco→IMU 2x2 rotation BEFORE the gate
        # check so the on-platform gate operates on IMU-frame radius
        # (which is the same magnitude as ArUco-frame radius for any
        # rotation, but doing it after keeps everything self-consistent
        # for any future affine corrections).
        p_plat = self._apply_aruco_imu_rotation(p_plat)
        # On-platform gate. The ray-plane intersection extends infinitely;
        # an orange object on a desk next to the platform projects to a
        # platform-frame point well outside the disk. Reject anything
        # beyond 220 mm radius (platform radius 200 mm + 20 mm margin
        # for ball-rim slop and pose noise) — V0 false-positives off the
        # side stop driving /ball_xy_mono and the GUI overlay (which
        # gates on /ball_xy_mono freshness).
        ON_PLATFORM_R_M = 0.220
        r2 = float(p_plat[0]) ** 2 + float(p_plat[1]) ** 2
        if r2 > ON_PLATFORM_R_M ** 2:
            return None
        return float(p_plat[0]), float(p_plat[1]), float(p_plat[2])

    def _project_spatial_to_platform(
            self, P_rgb_m: np.ndarray) -> Optional[tuple]:
        """Transform a 3-D point from RGB-camera frame (metres) to
        platform frame (metres). Used for /ball_xy_oak (the SLC
        truth signal) — no ray-plane intersection needed because we
        already have a 3-D point.
        """
        if self.last_pose is None or self.T_rgb_to_mono is None:
            return None

        # RGB → mono-left (4x4 homogeneous transform).
        P_h = np.append(P_rgb_m, 1.0)
        P_mono = (self.T_rgb_to_mono @ P_h)[:3]

        # mono-left → platform.
        p = self.last_pose.pose
        t_pose = np.array(
            [p.position.x, p.position.y, p.position.z])
        qx, qy, qz, qw = (p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        R_pose = _quat_to_rot(qw, qx, qy, qz)
        delta = P_mono - t_pose
        p_plat = R_pose.T @ delta
        p_plat = self._apply_aruco_imu_rotation(p_plat)
        # Same on-platform gate as the mono path — see
        # _project_to_platform docstring.
        ON_PLATFORM_R_M = 0.220
        r2 = float(p_plat[0]) ** 2 + float(p_plat[1]) ** 2
        if r2 > ON_PLATFORM_R_M ** 2:
            return None
        return float(p_plat[0]), float(p_plat[1]), float(p_plat[2])

    def _on_spatial(self, msg: PointStamped):
        """SLC point in RGB-camera frame (metres) → publish
        /ball_xy_oak in platform-frame mm.
        """
        P_rgb = np.array([msg.point.x, msg.point.y, msg.point.z])
        # Discard zero-depth (SLC reports 0 when no valid disparity).
        if abs(P_rgb[2]) < 1e-3:
            return
        result = self._project_spatial_to_platform(P_rgb)
        if result is None:
            return
        x_m, y_m, z_m = result
        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'platform'
        out.point.x = x_m * 1000.0
        out.point.y = y_m * 1000.0
        out.point.z = z_m * 1000.0
        self.pub_xy_oak.publish(out)

    def _tick(self):
        cx, cy, src = self._select_pixel()
        if cx is None:
            return
        self._n_v0_in += 1
        result = self._project_to_platform(cx, cy)
        if result is None:
            self._n_v0_gate_rejects += 1
            return
        x_m, y_m, z_m = result

        out = PointStamped()
        # Carry the detector's CAPTURE stamp through (was now()) so the
        # downstream KF can report true photon→state latency. The KF
        # consumes /ball_xy_mono by value only — it ignores this stamp and
        # times itself off its own clock — so this is behaviour-neutral for
        # control; it is purely latency instrumentation. src tells us which
        # cached detection won the pixel selection.
        src_msg = self.last_v1 if src == 'v1' else self.last_v0
        out.header.stamp = (src_msg.header.stamp if src_msg is not None
                            else self.get_clock().now().to_msg())
        out.header.frame_id = 'platform'
        out.point.x = x_m * 1000.0   # report in mm
        out.point.y = y_m * 1000.0
        out.point.z = z_m * 1000.0   # near zero by construction
        self.pub_xy_mono.publish(out)
        self._n_v0_out += 1
        self._last_pub_t = time.monotonic()
        # Per-stage latency probe (additive): capture→localizer-in (when we
        # received the winning detection) and capture→mono-out (now), both vs
        # the photon capture stamp. The digest differences these against the
        # detector's v0_lat and the KF's capture→state to localize the gap.
        if src_msg is not None:
            cap = src_msg.header.stamp
            cap_ns = int(cap.sec) * 1_000_000_000 + int(cap.nanosec)
            # Emit the probe ONLY for a fresh detection (new capture stamp),
            # never for the 60 Hz republishes of a cached v0 — otherwise the
            # median measures republish STALENESS, not the localizer's fresh
            # receive→publish latency.
            if cap_ns != self._last_lat_cap_ns:
                self._last_lat_cap_ns = cap_ns
                cap_s = cap_ns * 1e-9
                now_s = self.get_clock().now().nanoseconds * 1e-9
                recv_s = (self._last_v1_recv_s if src == 'v1'
                          else self._last_v0_recv_s)
                lat = Float32MultiArray()
                lat.data = [float((recv_s - cap_s) * 1e3),
                            float((now_s - cap_s) * 1e3)]
                self.pub_xy_mono_lat.publish(lat)

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
