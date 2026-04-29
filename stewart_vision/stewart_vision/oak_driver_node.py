#!/usr/bin/env python3
"""oak_driver_node — DepthAI driver for the Luxonis OAK-D Pro AF.

Hardware (OAK-D Pro AF):
  - 12 MP IMX378 RGB sensor (auto-focus)
  - Two OV9282 monochrome global-shutter stereo sensors
  - IR projector + IR illumination LED
  - Movidius Myriad X VPU (on-chip inference)

Builds a DepthAI pipeline that streams:
  - RGB (downscaled to 720p, JPEG-compressed) → /oak/rgb/image_compressed
  - Left mono (rectified by StereoDepth)      → /oak/left/image_compressed
  - Right mono (rectified by StereoDepth)     → /oak/right/image_compressed
  - Disparity (color-mapped)                  → /oak/disparity_compressed
  - V0 ball detection (host-side HSV thresh)  → /oak/ball/v0/rgb_pixel

V1 (YOLO) detection is a TODO — once the trained model is on disk it
plugs into the pipeline as a NeuralNetwork node and publishes
/oak/ball/v1/{left,right}_pixel. The stereo-triangulation localizer
(ball_localizer_node) is the consumer of the per-eye paths.

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md (§5, §8, §11.1).
"""
from __future__ import annotations

import os
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

# DepthAI is hardware-side; allow the module to import on machines
# without the OAK plugged in (the node will refuse to spin without it).
try:
    import depthai as dai
    HAVE_DEPTHAI = True
except ImportError:
    dai = None
    HAVE_DEPTHAI = False

try:
    import cv2
except ImportError:
    cv2 = None


# --- V0 color-threshold detector ---------------------------------------------

# Saturated orange foam ball on a black-and-white platform — wide HSV
# tolerance so paint variation and shadows don't drop detections. Tune
# in the GUI's vision-debug panel.
HSV_LO = np.array([5,   140, 90], dtype=np.uint8)   # H,S,V lower
HSV_HI = np.array([22,  255, 255], dtype=np.uint8)  # H,S,V upper
MIN_CONTOUR_AREA_PX = 60


def detect_ball_v0(rgb_bgr: np.ndarray) -> Optional[tuple]:
    """Return (cx, cy, radius_px, confidence) or None.

    Confidence is a heuristic in [0, 1] derived from the contour's
    circularity and area relative to expected ball size. Real tuning
    happens in the running system.
    """
    if cv2 is None:
        return None
    hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LO, HSV_HI)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area < MIN_CONTOUR_AREA_PX:
        return None
    (cx, cy), r = cv2.minEnclosingCircle(c)
    # Circularity: 1.0 = perfect circle, < ~0.7 = misshapen (probably noise).
    perim = cv2.arcLength(c, True)
    circ = 4.0 * np.pi * area / max(perim * perim, 1.0)
    confidence = float(np.clip(circ, 0.0, 1.0))
    return float(cx), float(cy), float(r), confidence


# --- DepthAI pipeline builder ------------------------------------------------

def _build_pipeline(rgb_fps: int = 15, mono_fps: int = 30):
    """Build the OAK-D Pro AF pipeline. Returns the dai.Pipeline.

    Phase-A pipeline (lean — RGB + raw mono left only):
      RGB:       1080p ISP scaled to 540p, MJPEG-encoded
      Mono left: 800p, RAW (not rectified)

    Why raw mono, not rectified:
      ArUco's cv2.aruco.estimatePoseBoard takes (K, dist) and handles
      distortion internally during solvePnP. Pre-rectifying the image
      and then handing the solver a "rectified K" only works if the
      rectified K we hand it matches the K of the actual rectified
      pixels. DepthAI's StereoDepth uses its own internal
      rectification with parameters that differ from a naive
      cv2.stereoRectify(alpha=0) call — empirically observed during
      Stage C bring-up where the rectified-K approach gave a 13 px
      reprojection error on a clean leveled-platform image.

      Solving directly against the raw mono image with the raw factory
      K + factory dist sidesteps the entire rectification-mismatch
      class of bugs.

      StereoDepth + the right mono are dropped from the Phase-A
      pipeline entirely — also frees Myriad X memory and lowers USB
      power draw, which compounded the earlier mid-stream crashes.
      Both come back when wiring `/ball_xy_stereo` (spec §7).
    """
    pipeline = dai.Pipeline()

    # Color (autofocus controlled by IMX378 lens)
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setFps(rgb_fps)
    cam_rgb.setIspScale(1, 2)              # 540p ISP for cheaper compression
    cam_rgb.setInterleaved(False)
    cam_rgb.initialControl.setAutoFocusMode(
        dai.RawCameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

    # Mono left — raw output for ArUco pose recovery
    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_l.setFps(mono_fps)

    # JPEG encoder for RGB stream
    enc_rgb = pipeline.create(dai.node.VideoEncoder)
    enc_rgb.setDefaultProfilePreset(
        rgb_fps, dai.VideoEncoderProperties.Profile.MJPEG)
    cam_rgb.video.link(enc_rgb.input)

    # Outputs
    def add_out(name, src):
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(name)
        src.link(out.input)
        return out

    add_out('rgb_jpeg', enc_rgb.bitstream)
    add_out('rgb_raw',  cam_rgb.video)         # uncompressed for V0 detector
    add_out('left',     mono_l.out)            # raw mono left for ArUco

    return pipeline


# --- ROS node ----------------------------------------------------------------

class OakDriverNode(Node):
    def __init__(self):
        super().__init__('oak_driver')

        if not HAVE_DEPTHAI:
            self.get_logger().fatal(
                "depthai not installed. `pip install depthai` and ensure the "
                "OAK is plugged in via USB3.")
            raise SystemExit(2)
        if cv2 is None:
            self.get_logger().fatal(
                "opencv-contrib-python not installed.")
            raise SystemExit(2)

        # Publishers — sensor QoS for the streams, default for events.
        sq = qos_profile_sensor_data
        self.pub_rgb = self.create_publisher(
            CompressedImage, '/oak/rgb/image_compressed', sq)
        self.pub_left = self.create_publisher(
            CompressedImage, '/oak/left/image_compressed', sq)
        self.pub_right = self.create_publisher(
            CompressedImage, '/oak/right/image_compressed', sq)
        self.pub_disp = self.create_publisher(
            CompressedImage, '/oak/disparity_compressed', sq)
        self.pub_v0 = self.create_publisher(
            PointStamped, '/oak/ball/v0/rgb_pixel', 10)
        self.pub_v0_diag = self.create_publisher(
            Float32MultiArray, '/oak/ball/v0/diagnostic', 10)

        self.get_logger().info("Building OAK pipeline...")
        pipeline = _build_pipeline()

        # USB speed cap. The Pi 5 caps total USB current at 600 mA
        # by default (raise to 1.2 A with `usb_max_current_enable=1`
        # in /boot/firmware/config.txt + reboot). Even with that
        # raised, OAK-D Pro at USB SUPER on Pi 5 has been observed to
        # crash mid-stream with a Myriad X firmware fault and no
        # crash-dump (X_LINK_ERROR on the next tryGet). Luxonis'
        # documented workaround is to force USB 2.0 — bandwidth is
        # plenty for our pipeline (540p RGB MJPEG + 800p mono left),
        # and current draw drops below the cap.
        # Override with `OAK_USB_SPEED=super` once power is solved
        # (powered hub or full Pi 5 5 V/5 A supply).
        usb_speed_env = os.environ.get('OAK_USB_SPEED', 'high').lower()
        usb_speed_map = {
            'high': dai.UsbSpeed.HIGH,        # USB 2.0 high-speed (480 Mbps)
            'super': dai.UsbSpeed.SUPER,      # USB 3.0 (5 Gbps)
            'super_plus': dai.UsbSpeed.SUPER_PLUS,  # USB 3.1 gen 2 (10 Gbps)
        }
        max_usb = usb_speed_map.get(usb_speed_env, dai.UsbSpeed.HIGH)
        self.get_logger().info(
            f"OAK device cap: maxUsbSpeed={max_usb.name} "
            f"(override via OAK_USB_SPEED=high|super|super_plus)")

        try:
            self.device = dai.Device(pipeline, maxUsbSpeed=max_usb)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open OAK device: {e}")
            raise SystemExit(2)

        self.get_logger().info(
            f"OAK opened: {self.device.getDeviceName()} | "
            f"USB speed: {self.device.getUsbSpeed()} | "
            f"cameras: {[s.name for s in self.device.getConnectedCameras()]}")

        self.q_rgb_jpeg  = self.device.getOutputQueue('rgb_jpeg', 4, False)
        self.q_rgb_raw   = self.device.getOutputQueue('rgb_raw', 2, False)
        self.q_left      = self.device.getOutputQueue('left', 2, False)
        # right mono / disparity / stereo rectification all dropped
        # from the Phase-A pipeline. See _build_pipeline docstring.

        # Drive everything from a single timer; DepthAI queues are
        # already producer-buffered, so we just drain them.
        self.create_timer(1.0 / 60.0, self._tick)

    def _publish_compressed(self, pub, payload: bytes, fmt: str):
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = fmt
        msg.data = list(payload) if isinstance(payload, (bytes, bytearray)) \
            else payload
        pub.publish(msg)

    def _encode_and_publish_mono(self, pub, frame_u8: np.ndarray):
        ok, buf = cv2.imencode('.jpg', frame_u8,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        self._publish_compressed(pub, buf.tobytes(), 'jpeg')

    def _tick(self):
        # RGB JPEG stream
        rgb_jpeg = self.q_rgb_jpeg.tryGet()
        if rgb_jpeg is not None:
            self._publish_compressed(
                self.pub_rgb, rgb_jpeg.getData().tobytes(), 'jpeg')

        # RGB raw → V0 detector → /oak/ball/v0/rgb_pixel
        rgb_raw = self.q_rgb_raw.tryGet()
        if rgb_raw is not None:
            frame = rgb_raw.getCvFrame()  # BGR uint8
            det = detect_ball_v0(frame)
            if det is not None:
                cx, cy, r, conf = det
                p = PointStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = 'oak_rgb'
                p.point.x = cx
                p.point.y = cy
                p.point.z = r           # radius in pixels piggy-backed in z
                self.pub_v0.publish(p)
                # diagnostic with confidence + radius for debug strip
                d = Float32MultiArray()
                d.data = [float(cx), float(cy), float(r), float(conf)]
                self.pub_v0_diag.publish(d)

        # Raw mono left — Stage C ArUco solve consumes this.
        # estimatePoseBoard takes (K, dist) and handles distortion
        # internally; no pre-rectification needed.
        left = self.q_left.tryGet()
        if left is not None:
            self._encode_and_publish_mono(self.pub_left, left.getCvFrame())

        # right + disparity outputs not exposed in Phase A. The
        # pub_right / pub_disp publishers stay declared so the topic
        # names exist for any subscriber that probes for them.

    def destroy_node(self):
        try:
            if hasattr(self, 'device'):
                self.device.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = OakDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
