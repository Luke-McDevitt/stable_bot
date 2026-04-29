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
from std_msgs.msg import Float32, Float32MultiArray

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


# --- Resolution / output sizes (must match the pipeline build below) --------

# RGB ISP at 1080p / 2 = 540p effective. V0 detection coordinates and
# the SLC ROI normalization both reference this.
RGB_W, RGB_H = 960, 540

# Mono cameras at THE_800_P native (1280 × 800).
MONO_W, MONO_H = 1280, 800


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

    Phase-B pipeline (depth-based detection):
      RGB:       1080p ISP scaled to 540p, MJPEG-encoded + raw for V0
      Mono left: 800p, RAW (not rectified) — for ArUco pose recovery
      Mono right: 800p (StereoDepth input only — no XLinkOut yet)
      StereoDepth: depth aligned to RGB via setDepthAlign(CAM_A);
                   LR-check + subpixel + disparity output all OFF
                   (memory budget + we don't need them yet).
      SpatialLocationCalculator: takes a runtime ROI in normalized
                   RGB coordinates and emits the average 3-D point
                   in RGB-camera frame at that ROI (metres).

    Why raw mono left for ArUco:
      cv2.aruco.estimatePoseBoard takes (K, dist) and handles
      distortion internally during solvePnP. The cv2.stereoRectify-
      derived K diverges from DepthAI's internal rectification, which
      gave 13 px reprojection error during bring-up. Solving against
      the raw image with raw factory K + raw factory dist sidesteps
      the whole rectification-mismatch class of bugs.

    Why depth aligned to RGB:
      V0 detects in RGB-frame pixels. The SLC needs an ROI in the
      depth image. With setDepthAlign(CAM_A), depth pixels share the
      RGB image's coordinate space, so the V0 ROI maps 1:1 — no
      separate RGB↔mono pixel conversion needed at the OAK side.
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

    # Mono right — needed only as StereoDepth's right input. No XLinkOut.
    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_r.setFps(mono_fps)

    # StereoDepth — depth output aligned to RGB. Conservative settings
    # to keep Myriad X working memory in budget. NOTE: LR-check MUST
    # be ON when setDepthAlign points at a non-input socket (CAM_A here);
    # the OAK firmware rejects the config otherwise (error code 180:
    # "Disparity/depth CENTER alignment requires left-right check mode
    # enabled"). Subpixel and disparity-output stay off — those were the
    # actual memory hogs in the earlier mid-stream crash.
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    # SpatialLocationCalculator — takes a config (with ROIs) and emits
    # the 3-D point per ROI sampled from depth. Driven dynamically
    # from the host: when V0 detects a ball, _tick pushes a config
    # with a small ROI centered on (cx, cy).
    slc = pipeline.create(dai.node.SpatialLocationCalculator)
    # Don't block on missing config — let depth flow even when no
    # detection has been pushed yet (avoids head-of-line blocking).
    slc.inputConfig.setWaitForMessage(False)
    slc.inputDepth.setBlocking(False)

    # Initial config so the SLC has something to emit before the host
    # sends the first runtime ROI. Centered ROI, depth thresholds wide.
    init_roi = dai.SpatialLocationCalculatorConfigData()
    init_roi.depthThresholds.lowerThreshold = 100    # mm
    init_roi.depthThresholds.upperThreshold = 5000   # mm
    init_roi.roi = dai.Rect(
        dai.Point2f(0.45, 0.45), dai.Point2f(0.55, 0.55))
    slc.initialConfig.addROI(init_roi)

    stereo.depth.link(slc.inputDepth)

    # Runtime config input (host → SLC) for V0-driven ROI updates.
    slc_cfg_in = pipeline.create(dai.node.XLinkIn)
    slc_cfg_in.setStreamName('slc_config')
    slc_cfg_in.out.link(slc.inputConfig)

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
    add_out('rgb_raw',   cam_rgb.video)            # uncompressed for V0
    add_out('left',      mono_l.out)               # raw mono left for ArUco
    add_out('spatial',   slc.out)                  # 3-D point in RGB frame

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
        # OAK-capture-to-Pi latency, computed Pi-side from the device
        # timestamp metadata so it doesn't depend on cross-host clock
        # sync. Published every RGB frame.
        self.pub_latency = self.create_publisher(
            Float32, '/oak/latency_ms', 10)
        # SpatialLocationCalculator output — 3-D ball position in
        # RGB-camera frame (metres). ball_localizer_node transforms
        # this into platform-frame mm and republishes as
        # /ball_xy_oak (spec §7 truth signal).
        self.pub_spatial = self.create_publisher(
            PointStamped, '/oak/ball/spatial', 10)

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

        # Enable the IR dot projector to add texture for stereo
        # matching on featureless surfaces (the platform top is
        # mostly smooth black, which gives StereoDepth nothing to
        # match without projected texture). 800 mA is moderate;
        # the OAK-D Pro accepts up to 1200 mA. 0 = OFF.
        # Override via OAK_IR_PROJECTOR_MA env var.
        ir_proj_ma = float(os.environ.get('OAK_IR_PROJECTOR_MA', '800'))
        try:
            self.device.setIrLaserDotProjectorBrightness(ir_proj_ma)
            self.get_logger().info(
                f"IR dot projector: {ir_proj_ma:.0f} mA")
        except Exception as e:
            self.get_logger().warn(
                f"IR projector control unavailable: {e}")

        # Cap queues at 1 (latest-only, non-blocking). With size > 1,
        # any tick the host is late picking up backlogs frames, and
        # the OAK-side latency we publish on /oak/latency_ms grows by
        # one frame period per queued frame. With size 1 the queue
        # always holds the most recent frame; older ones are dropped.
        # We'd rather skip frames than serve stale ones to the
        # controller — see oscillating-latency observation 2026-04-29.
        self.q_rgb_jpeg = self.device.getOutputQueue('rgb_jpeg', 1, False)
        self.q_rgb_raw  = self.device.getOutputQueue('rgb_raw',  1, False)
        self.q_left     = self.device.getOutputQueue('left',     1, False)
        # SpatialLocationCalculator output. Latest-only — we only
        # need the freshest 3-D point per V0 detection.
        self.q_spatial  = self.device.getOutputQueue('spatial',  1, False)
        # Runtime SLC config in (host → device). Used by _tick to push
        # a small ROI around each V0 detection so the SLC samples
        # depth at the ball pixel.
        self.q_slc_cfg  = self.device.getInputQueue('slc_config')

        # Tightness of the depth ROI around the V0 pixel. 8 px on each
        # side at 540p RGB is roughly 1.5° of FOV — covers the ball
        # well at typical platform distances without spilling onto the
        # platform background and pulling depth toward the plate.
        self._slc_roi_half_norm = 8.0 / RGB_W

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
            # OAK-capture-to-Pi-receipt latency. Both timestamps come
            # from dai's steady_clock so they share the same epoch with
            # no host-clock-skew dependency. This is the only honest
            # latency the Pi can compute by itself; transport over
            # rosbridge to the laptop adds more on top, but that piece
            # is not part of the closed-loop control budget (spec §13).
            try:
                capture_ts = rgb_jpeg.getTimestamp().total_seconds()
                now_ts = dai.Clock.now().total_seconds()
                lat_msg = Float32()
                lat_msg.data = float((now_ts - capture_ts) * 1000.0)
                self.pub_latency.publish(lat_msg)
            except Exception:
                pass

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

                # Push a fresh ROI to the SpatialLocationCalculator so
                # depth is sampled exactly where the ball was seen.
                # ROI in NORMALIZED depth-image coords ([0, 1]).
                # Depth is aligned to RGB (setDepthAlign(CAM_A)), so
                # we can use RGB pixel coords directly.
                cx_n = float(np.clip(cx / RGB_W, 0.0, 1.0))
                cy_n = float(np.clip(cy / RGB_H, 0.0, 1.0))
                hr = self._slc_roi_half_norm
                tl = dai.Point2f(
                    max(0.0, cx_n - hr), max(0.0, cy_n - hr))
                br = dai.Point2f(
                    min(1.0, cx_n + hr), min(1.0, cy_n + hr))
                roi_cfg = dai.SpatialLocationCalculatorConfigData()
                roi_cfg.depthThresholds.lowerThreshold = 100   # mm
                roi_cfg.depthThresholds.upperThreshold = 5000  # mm
                roi_cfg.roi = dai.Rect(tl, br)
                cfg = dai.SpatialLocationCalculatorConfig()
                cfg.addROI(roi_cfg)
                try:
                    self.q_slc_cfg.send(cfg)
                except Exception:
                    pass

        # Spatial output → /oak/ball/spatial. Coordinates in metres
        # in the RGB-camera frame (because depth was aligned to CAM_A).
        spatial = self.q_spatial.tryGet()
        if spatial is not None:
            data = spatial.getSpatialLocations()
            if data:
                d0 = data[0]
                # depthAverage is in mm; spatialCoordinates {x,y,z} also mm.
                P_mm = d0.spatialCoordinates
                if abs(P_mm.z) > 1.0:   # ignore zero-depth (no valid disparity)
                    sp = PointStamped()
                    sp.header.stamp = self.get_clock().now().to_msg()
                    sp.header.frame_id = 'oak_rgb'
                    # ball_localizer expects metres on /oak/ball/spatial.
                    sp.point.x = float(P_mm.x) * 1e-3
                    sp.point.y = float(P_mm.y) * 1e-3
                    sp.point.z = float(P_mm.z) * 1e-3
                    self.pub_spatial.publish(sp)

        # Raw mono left — Stage C ArUco solve consumes this.
        # estimatePoseBoard takes (K, dist) and handles distortion
        # internally; no pre-rectification needed.
        left = self.q_left.tryGet()
        if left is not None:
            self._encode_and_publish_mono(self.pub_left, left.getCvFrame())

        # right + disparity outputs not exposed yet. The
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
