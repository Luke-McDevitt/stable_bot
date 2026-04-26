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

    Resolution choices:
      RGB: 1080p ISP, downscaled to 540p for streaming (the controller
           doesn't need 1080p; the operator only sees compressed frames).
      Mono: 800p (1280x800 OV9282 native).

    Stereo depth runs at the mono sensors' native rate; disparity is
    streamed at low fps (5) for the GUI debug panel.
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

    # Mono left
    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_l.setFps(mono_fps)

    # Mono right
    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_r.setFps(mono_fps)

    # Stereo depth — outputs rectified left, rectified right, disparity.
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(True)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

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

    add_out('rgb_jpeg',  enc_rgb.bitstream)
    add_out('rgb_raw',   cam_rgb.video)        # uncompressed for V0 detector
    add_out('left_rect', stereo.rectifiedLeft)
    add_out('right_rect', stereo.rectifiedRight)
    add_out('disparity', stereo.disparity)

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

        try:
            self.device = dai.Device(pipeline)
        except Exception as e:
            self.get_logger().fatal(f"Failed to open OAK device: {e}")
            raise SystemExit(2)

        self.get_logger().info(
            f"OAK opened: {self.device.getDeviceName()} | "
            f"USB speed: {self.device.getUsbSpeed()} | "
            f"cameras: {[s.name for s in self.device.getConnectedCameras()]}")

        self.q_rgb_jpeg  = self.device.getOutputQueue('rgb_jpeg', 4, False)
        self.q_rgb_raw   = self.device.getOutputQueue('rgb_raw', 2, False)
        self.q_left      = self.device.getOutputQueue('left_rect', 2, False)
        self.q_right     = self.device.getOutputQueue('right_rect', 2, False)
        self.q_disparity = self.device.getOutputQueue('disparity', 2, False)

        # Drive everything from a single timer; DepthAI queues are
        # already producer-buffered, so we just drain them.
        self.create_timer(1.0 / 60.0, self._tick)
        self._frame_count = 0

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

        # Mono streams
        left = self.q_left.tryGet()
        if left is not None:
            self._encode_and_publish_mono(self.pub_left, left.getCvFrame())
        right = self.q_right.tryGet()
        if right is not None:
            self._encode_and_publish_mono(self.pub_right, right.getCvFrame())

        # Disparity (color-mapped, low rate — 5 Hz target)
        self._frame_count += 1
        if self._frame_count % 12 == 0:
            disp = self.q_disparity.tryGet()
            if disp is not None:
                d = disp.getFrame()
                # Normalize to 8-bit for visualization.
                d8 = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX)
                d8 = d8.astype(np.uint8)
                color = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
                ok, buf = cv2.imencode('.jpg', color,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ok:
                    self._publish_compressed(
                        self.pub_disp, buf.tobytes(), 'jpeg')

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
