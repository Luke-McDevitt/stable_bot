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
from typing import Optional, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
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


def detect_ball_v0(rgb_bgr: np.ndarray,
                   restrict_mask: Optional[np.ndarray] = None) -> Optional[tuple]:
    """Return (cx, cy, radius_px, confidence) or None.

    Confidence is a heuristic in [0, 1] derived from the contour's
    circularity and area relative to expected ball size. Real tuning
    happens in the running system.

    If ``restrict_mask`` is given (a uint8 mask of the same H×W as
    ``rgb_bgr`` with 255 inside the region of interest), the HSV
    threshold mask is ANDed with it before contour finding so V0 only
    considers pixels inside the projected platform disk. Without it,
    V0 picks the most-orange thing anywhere in the frame — including
    foam pads, table edges, or bench clutter — which we observed
    repeatedly on 2026-04-29 (V0 locking onto pixel (138, 177), well
    outside the platform).
    """
    if cv2 is None:
        return None
    hsv = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LO, HSV_HI)
    if restrict_mask is not None:
        mask = cv2.bitwise_and(mask, restrict_mask)
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


# --- Platform-mask projection (RGB image space) -----------------------------

# Platform disk radius, in metres. Slightly larger than the physical 200 mm
# so a ball just inside the rim isn't trimmed by sub-pixel polygon
# rasterization or pose noise. Matches the on-platform gate in
# ball_localizer_node._project_to_platform.
PLATFORM_MASK_RADIUS_M = 0.220
# The depth-blob detector needs a TIGHTER mask than V0 because the
# ArUco marker ring is mounted on a frame that physically protrudes
# above the deck (the markers sit at ring_radius_mm=120 with size 50
# so they cover r∈[~95, ~145] mm, and the mounting hardware is 5 to
# tens of mm above the deck). Inside the 220-mm V0 mask, the
# marker-ring frame dominates the connected components — bag
# 20260429T230819Z showed depth-blob locked at 150 mm from V0 ball
# with std=0.9 mm, on a 10000-pixel blob. V0 doesn't care because
# the markers aren't orange. Keep depth-blob inside the marker ring.
PLATFORM_DEPTH_MASK_RADIUS_M = 0.080


def _quat_to_rot(w, x, y, z):
    """Quaternion (w, x, y, z) → 3x3 rotation matrix. Same convention
    as ball_localizer_node so the masks line up with the gate."""
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


def _project_platform_polygon(K: np.ndarray, dist: np.ndarray,
                              R_pose: np.ndarray, t_pose: np.ndarray,
                              radius_m: float = PLATFORM_MASK_RADIUS_M,
                              n_samples: int = 72) -> Optional[np.ndarray]:
    """Project the platform disk's outline into RGB image pixels.

    Returns an Nx2 int32 array suitable for cv2.fillConvexPoly, or None
    if the polygon ends up empty/behind the camera.
    """
    if cv2 is None:
        return None
    theta = np.linspace(0.0, 2.0 * np.pi, n_samples, endpoint=False)
    pts_plat = np.stack(
        [radius_m * np.cos(theta),
         radius_m * np.sin(theta),
         np.zeros_like(theta)], axis=1)
    pts_cam = (R_pose @ pts_plat.T).T + t_pose
    in_front = pts_cam[:, 2] > 1e-3
    if not np.all(in_front):
        return None
    obj = pts_cam.reshape(-1, 1, 3).astype(np.float64)
    rvec = np.zeros(3, dtype=np.float64)
    tvec = np.zeros(3, dtype=np.float64)
    pix, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    pix = pix.reshape(-1, 2)
    return pix.astype(np.int32)


def _platform_image_mask(shape: Tuple[int, int],
                         polygon: np.ndarray) -> np.ndarray:
    """Rasterize the projected polygon to a uint8 mask at ``shape``."""
    mask = np.zeros(shape, dtype=np.uint8)
    if polygon is None or len(polygon) < 3:
        return mask
    cv2.fillConvexPoly(mask, polygon, 255)
    return mask


# --- Depth-blob detector (option 3) -----------------------------------------

# Acceptable height-above-platform window for a foam ball. The visible
# top hemisphere of a 40 mm-diameter ball ranges 0..40 mm above the
# platform plane; we leave a few mm of slack on each side to absorb
# stereo noise + plane-fit error.
BALL_HEIGHT_MIN_MM = 5.0
BALL_HEIGHT_MAX_MM = 80.0
# Lowered from 30 → 10 px after observing detection rate drop from
# ~90 % to ~17 % on the foam ball. Foam absorbs IR poorly, so the
# OAK's stereo depth map is sparse on the ball and the connected-
# component areas are smaller than the geometric ball-cap area would
# predict. A 10-px minimum keeps the detector firing as long as the
# blob is unambiguously above the plane; the morphological close
# below stitches the sparse hits back together.
DEPTH_BLOB_MIN_AREA_PX = 10


def _expected_plane_depth_map(shape: Tuple[int, int],
                              K: np.ndarray,
                              R_pose: np.ndarray,
                              t_pose: np.ndarray) -> np.ndarray:
    """Per-pixel expected z-depth (mm) of the platform plane.

    For pixel (u, v) the back-projected ray in camera frame is
    d(u,v) = K^{-1}·(u, v, 1)^T. The platform plane is
    n^T (P - t_pose) = 0 with n = R_pose[:, 2]. Substituting
    P = s·d gives s = n^T·t_pose / n^T·d, and the z-depth at that
    pixel is just s (because d.z = 1 by construction).

    Returns a float32 array (H, W). Pixels behind the camera or where
    the ray is parallel to the plane are filled with +inf so they
    naturally fail the height-above-plane threshold.
    """
    H, W = shape
    fx = float(K[0, 0]); fy = float(K[1, 1])
    cx = float(K[0, 2]); cy = float(K[1, 2])
    u = (np.arange(W, dtype=np.float32) - cx) / fx
    v = (np.arange(H, dtype=np.float32) - cy) / fy
    uu, vv = np.meshgrid(u, v)               # both (H, W) float32
    # n in camera frame
    n = R_pose[:, 2].astype(np.float32)
    # n^T·d = n.x·u + n.y·v + n.z
    denom = n[0] * uu + n[1] * vv + n[2]
    num = float(np.dot(n, t_pose))
    out = np.full_like(uu, np.inf, dtype=np.float32)
    valid = np.abs(denom) > 1e-6
    out[valid] = (num / denom[valid]).astype(np.float32) * 1000.0  # m → mm
    out[out <= 0] = np.inf
    return out


def detect_ball_depth_blob(depth_mm: np.ndarray,
                           platform_mask: np.ndarray,
                           expected_depth_mm: np.ndarray,
                           h_min_mm: float = BALL_HEIGHT_MIN_MM,
                           h_max_mm: float = BALL_HEIGHT_MAX_MM,
                           min_area_px: int = DEPTH_BLOB_MIN_AREA_PX,
                           edge_erode_px: int = 12) \
        -> Optional[tuple]:
    """Find an "above-the-platform" blob in the depth image.

    Inputs:
      depth_mm           uint16 (H, W) depth from StereoDepth (mm,
                         0 = invalid)
      platform_mask      uint8 (H, W) 255 inside the platform disk
      expected_depth_mm  float32 (H, W) per-pixel expected platform-
                         plane depth in mm (see
                         _expected_plane_depth_map)
      edge_erode_px      pixels to erode the mask by before height
                         thresholding. Stereo matching produces
                         systematically wrong depths within a few
                         pixels of any depth discontinuity (the
                         platform rim is one); eroding suppresses
                         that fringe.

    Method:
      1. Erode the platform mask to dump the depth-discontinuity
         fringe.
      2. Compute height = expected_depth - measured_depth.
      3. Take the median height across the eroded mask's valid
         pixels. This is the empirical platform plane offset — it
         absorbs the cm-level bias in the ArUco-derived expected
         plane and re-zeroes the threshold without needing a
         tighter pose. The ball's pixels are a small fraction of
         the platform area, so they don't move the median. (This
         is also why we use median over mean — robust to a small
         high-outlier population, i.e., the ball.)
      4. Threshold (height - offset) ∈ [h_min, h_max] inside the
         eroded mask. Take the largest connected component.

    Returns (cx, cy, area_px, confidence) or None.
    """
    if cv2 is None:
        return None
    if depth_mm.shape != platform_mask.shape:
        # StereoDepth depth-aligned to CAM_A should match RGB raw
        # resolution; if it doesn't, resize depth to the mask shape.
        depth_mm = cv2.resize(
            depth_mm, (platform_mask.shape[1], platform_mask.shape[0]),
            interpolation=cv2.INTER_NEAREST)

    # Erode the mask. The kernel is 2*r+1 square so an erode of
    # `edge_erode_px` shaves that many pixels off every direction.
    if edge_erode_px > 0:
        k = 2 * int(edge_erode_px) + 1
        eroded = cv2.erode(platform_mask,
                           np.ones((k, k), np.uint8), iterations=1)
    else:
        eroded = platform_mask

    measured = depth_mm.astype(np.float32)
    # Treat missing depth (0) as "no observation" — fill with +inf so
    # the height test fails for those pixels.
    invalid = measured <= 1.0
    measured_safe = measured.copy()
    measured_safe[invalid] = np.inf
    raw_height = expected_depth_mm - measured_safe   # +ve = closer than plane

    # Empirical plane offset: median height across valid platform
    # pixels. The ArUco marker ring is mounted significantly above
    # the actual platform deck — bag 20260429T224802Z showed every
    # frame hitting a −100 mm clamp with std=0, which means the true
    # offset is well past −100 mm and the clamp was breaking
    # detection. Clip is now ±500 mm; large enough that any sane
    # geometry passes through, small enough that a truly broken pose
    # (e.g., quaternion went NaN) still produces something
    # bounded. Detector remains robust to ball+small-object outliers
    # because median is the aggregator.
    valid_for_plane = (~invalid) & (eroded > 0) & np.isfinite(raw_height)
    if int(np.sum(valid_for_plane)) >= 100:
        plane_offset = float(np.median(raw_height[valid_for_plane]))
        plane_offset = max(-500.0, min(500.0, plane_offset))
    else:
        plane_offset = 0.0

    height = raw_height - plane_offset
    above = (height > h_min_mm) & (height < h_max_mm)
    above &= eroded > 0
    if not np.any(above):
        return None
    bin_img = above.astype(np.uint8) * 255
    # Close BEFORE open: foam-ball depth has scattered dropouts (foam
    # absorbs IR), so close (dilate-then-erode) stitches a sparse
    # blob back into a single component. Open kills isolated 1–2 px
    # noise speckles that survived the close.
    kernel = np.ones((3, 3), np.uint8)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE,
                               kernel, iterations=2)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN,
                               kernel, iterations=1)
    n_lab, labels, stats, cents = cv2.connectedComponentsWithStats(
        bin_img, connectivity=8)
    # label 0 is background — skip it
    if n_lab <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    j = int(np.argmax(areas)) + 1
    area = float(stats[j, cv2.CC_STAT_AREA])
    if area < min_area_px:
        return None
    cx, cy = cents[j]
    # Confidence = clipped fraction of the largest blob's bounding-box
    # area filled with above-plane pixels. A perfect ball cap fills
    # ~π/4 of its bbox; we normalize against that.
    bbox_w = float(stats[j, cv2.CC_STAT_WIDTH])
    bbox_h = float(stats[j, cv2.CC_STAT_HEIGHT])
    bbox_area = max(bbox_w * bbox_h, 1.0)
    fill = area / bbox_area
    confidence = float(np.clip(fill / (np.pi / 4.0), 0.0, 1.0))
    return (float(cx), float(cy), float(area), confidence,
            plane_offset, bin_img, eroded)


def render_depth_debug_overlay(rgb_bgr: np.ndarray,
                               platform_mask: Optional[np.ndarray],
                               eroded_mask: Optional[np.ndarray],
                               above_mask: Optional[np.ndarray],
                               blob: Optional[tuple],
                               plane_offset_mm: float,
                               n_above: int = 0) -> np.ndarray:
    """Annotate the RGB frame with everything the depth-blob detector
    saw. Returns a BGR uint8 frame the caller can JPEG-encode and
    publish. This is the canonical artifact for "why didn't depth-blob
    detect the ball" debugging.

    Layers (bottom to top):
      - RGB
      - Above-plane pixels tinted lime green (where the threshold
        actually fired)
      - Eroded platform mask outline (yellow, thick)
      - Full platform mask outline (cyan, thin)
      - Detected blob: red minEnclosingCircle + centroid dot
      - Text strip: plane_offset_mm, area_px, valid-pixel count
    """
    if cv2 is None:
        return rgb_bgr
    out = rgb_bgr.copy()
    # Above-plane tint first so outlines paint on top.
    if above_mask is not None:
        # `above_mask` from cv2.connectedComponents lives at 0/255 already
        m = above_mask > 0
        if np.any(m):
            tint = out.copy()
            tint[m] = (0, 255, 0)            # lime green in BGR
            out = cv2.addWeighted(out, 0.6, tint, 0.4, 0)
    # Outlines
    if platform_mask is not None:
        cnts, _ = cv2.findContours(
            platform_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, cnts, -1, (255, 200, 0), 1)   # cyan
    if eroded_mask is not None:
        cnts, _ = cv2.findContours(
            eroded_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, cnts, -1, (0, 255, 255), 2)   # yellow
    # Detected blob
    if blob is not None:
        cx, cy, area_px = blob[0], blob[1], blob[2]
        r_px = int(np.sqrt(max(area_px, 1.0) / np.pi))
        cv2.circle(out, (int(cx), int(cy)), max(r_px, 6),
                   (0, 0, 255), 2)
        cv2.circle(out, (int(cx), int(cy)), 3, (0, 0, 255), -1)
    # HUD strip
    txt = f"plane_off={plane_offset_mm:+.1f}mm  above_px={n_above}"
    if blob is not None:
        txt += f"  blob_area={int(blob[2])}"
    else:
        txt += "  NO BLOB"
    cv2.rectangle(out, (4, 4), (4 + 11 * len(txt) + 8, 30),
                  (0, 0, 0), -1)
    cv2.putText(out, txt, (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
    return out


def _load_rgb_intrinsics(yaml_path: str) -> Tuple[Optional[np.ndarray],
                                                  Optional[np.ndarray]]:
    """Read K and dist from the 'rgb' block of oak_intrinsics.yaml.

    Returns (None, None) if the file or the block is missing.
    """
    if not os.path.isfile(yaml_path):
        return None, None
    try:
        with open(yaml_path, 'r') as f:
            d = yaml.safe_load(f)
        rgb = d.get('rgb') or d.get('left')
        if rgb is None:
            return None, None
        K = np.asarray(rgb['K'], dtype=np.float64).reshape(3, 3)
        dist = np.asarray(rgb['dist'], dtype=np.float64).ravel()
        return K, dist
    except Exception:
        return None, None


# --- DepthAI pipeline builder ------------------------------------------------

def _build_pipeline(rgb_fps: int = 15, mono_fps: int = 15,
                    enable_depth: bool = False):
    """Build the OAK-D Pro AF pipeline. Returns the dai.Pipeline.

    Two configurations, selected by `enable_depth`:

    enable_depth=False (default — Phase A, lean):
      RGB:       1080p ISP scaled to 540p, MJPEG-encoded + raw for V0
      Mono left: 800p, RAW — ArUco pose recovery
      No mono right, no StereoDepth, no SLC, no IR projector.
      Lowest memory + USB current draw. /ball_xy_mono works (ray-
      plane projection from V0); /ball_xy_oak does not publish.

    enable_depth=True (full — depth-based detection):
      Adds mono right, StereoDepth (with LR-check + depth-align to
      RGB), and SpatialLocationCalculator. The IR dot projector
      should be enabled (OAK_IR_PROJECTOR_MA env var) to texture
      featureless surfaces for stereo matching. Heavier on USB
      current — observed to brown out the Pi 5 without a powered
      hub on 2026-04-29. Opt in once power is sized for it.

    StereoDepth in depth-mode:
      LR-check + depth-align(CAM_A) — depth aligned to RGB so V0
      pixel coordinates map 1:1 to depth pixels for the SLC ROI.
      LR-check is REQUIRED whenever depth is aligned to a non-input
      socket; the OAK firmware halts the pipeline otherwise (error
      code 180).

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
    add_out('rgb_raw',  cam_rgb.video)             # uncompressed for V0
    add_out('left',     mono_l.out)                # raw mono left for ArUco

    if not enable_depth:
        return pipeline

    # ---- Depth subsystem (opt-in via OAK_ENABLE_DEPTH=1) ----

    # Mono right — needed as StereoDepth's right input.
    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_r.setFps(mono_fps)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(
        dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)        # required when depth-align
                                          # targets a non-input socket
    stereo.setExtendedDisparity(False)
    stereo.setSubpixel(False)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    slc = pipeline.create(dai.node.SpatialLocationCalculator)
    slc.inputConfig.setWaitForMessage(False)
    slc.inputDepth.setBlocking(False)

    init_roi = dai.SpatialLocationCalculatorConfigData()
    init_roi.depthThresholds.lowerThreshold = 100    # mm
    init_roi.depthThresholds.upperThreshold = 5000   # mm
    init_roi.roi = dai.Rect(
        dai.Point2f(0.45, 0.45), dai.Point2f(0.55, 0.55))
    slc.initialConfig.addROI(init_roi)

    stereo.depth.link(slc.inputDepth)

    slc_cfg_in = pipeline.create(dai.node.XLinkIn)
    slc_cfg_in.setStreamName('slc_config')
    slc_cfg_in.out.link(slc.inputConfig)

    add_out('spatial', slc.out)            # 3-D point in RGB frame
    # Full depth image (uint16 mm, aligned to CAM_A so resolution
    # matches the RGB ISP output). Consumed by the depth-blob
    # detector. Bandwidth: 540×960×2 bytes × 15 fps ≈ 15 MB/s — fine
    # over USB3 SUPER. Skip it on USB2 if you ever fall back there:
    # see the OAK_USB_SPEED env var.
    add_out('depth', stereo.depth)

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
        # Independent depth-blob detection — orange-agnostic. Picks
        # the largest connected region of pixels lying above the
        # platform plane (5..80 mm closer to the camera than the
        # plane's expected depth at that pixel). Lives alongside V0
        # so we can compare the two with a centered ball.
        self.pub_depth_pixel = self.create_publisher(
            PointStamped, '/oak/ball/depth/rgb_pixel', 10)
        self.pub_depth_diag = self.create_publisher(
            Float32MultiArray, '/oak/ball/depth/diagnostic', 10)
        # Annotated RGB frame: platform mask outlines + above-plane
        # pixels tinted + detected blob + HUD. Watch this in
        # foxglove or rosbag2 to see why depth-blob is or isn't
        # detecting the ball. Toggle with OAK_DEPTH_DEBUG=0|1
        # (default on; cost is one JPEG encode per depth frame
        # ≈ 6 ms on Pi 5).
        self.pub_depth_dbg = self.create_publisher(
            CompressedImage, '/oak/depth_blob/debug_image', sq)
        self.depth_debug_enabled = (
            os.environ.get('OAK_DEPTH_DEBUG', '1') == '1')

        # Depth subsystem (stereo + SLC + IR projector) is opt-in. The
        # full pipeline pulls extra USB current (the IR projector alone
        # is hundreds of mA) and was observed to brown out the Pi 5 on
        # 2026-04-29 even with usb_max_current_enable=1. Set this to
        # '1' once a powered USB hub or 5 A PSU is in place.
        self.enable_depth = os.environ.get('OAK_ENABLE_DEPTH', '0') == '1'
        self.get_logger().info(
            f"Depth subsystem: {'ENABLED' if self.enable_depth else 'DISABLED'} "
            f"(set OAK_ENABLE_DEPTH=1 to enable)")

        self.get_logger().info("Building OAK pipeline...")
        pipeline = _build_pipeline(enable_depth=self.enable_depth)

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

        # IR dot projector — only meaningful when stereo is in the
        # pipeline (it adds texture for stereo matching on featureless
        # surfaces). Default 0 = off; opt-in via OAK_IR_PROJECTOR_MA.
        # Range is 0..1200 mA. 400 mA is a reasonable starting point
        # once the depth subsystem is enabled.
        if self.enable_depth:
            ir_proj_ma = float(os.environ.get('OAK_IR_PROJECTOR_MA', '0'))
            try:
                self.device.setIrLaserDotProjectorBrightness(ir_proj_ma)
                self.get_logger().info(
                    f"IR dot projector: {ir_proj_ma:.0f} mA "
                    f"(override via OAK_IR_PROJECTOR_MA, range 0..1200)")
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
        # Spatial output / SLC config queues — only present when depth
        # is enabled. _tick guards on these being non-None.
        if self.enable_depth:
            self.q_spatial = self.device.getOutputQueue('spatial', 1, False)
            self.q_slc_cfg = self.device.getInputQueue('slc_config')
            self.q_depth = self.device.getOutputQueue('depth', 1, False)
        else:
            self.q_spatial = None
            self.q_slc_cfg = None
            self.q_depth = None

        # Platform-mask + depth-blob state ---------------------------------
        # We need RGB intrinsics + the latest /platform_pose to (a)
        # mask V0 to the platform disk and (b) compute per-pixel
        # expected platform-plane depth for the depth-blob detector.
        # Both are recomputed on pose update; everything else is
        # read-only at runtime.
        self.K_rgb: Optional[np.ndarray] = None
        self.dist_rgb: Optional[np.ndarray] = None
        try:
            from ament_index_python.packages \
                import get_package_share_directory
            share = get_package_share_directory('stewart_vision')
            intr_path = os.path.join(share, 'config', 'oak_intrinsics.yaml')
        except Exception:
            intr_path = ''
        if intr_path:
            self.K_rgb, self.dist_rgb = _load_rgb_intrinsics(intr_path)
            if self.K_rgb is None:
                self.get_logger().warn(
                    f"oak_intrinsics.yaml at {intr_path} not loaded; "
                    f"V0 platform-mask + depth-blob disabled until "
                    f"calibrate_oak.py --stage factory has run.")
            else:
                self.get_logger().info(
                    f"loaded RGB intrinsics from {intr_path}")

        self._last_pose: Optional[PoseStamped] = None
        self._mask_pose_stamp: Optional[float] = None
        self._platform_mask: Optional[np.ndarray] = None
        # Tighter mask for the depth-blob detector that lives inside
        # the ArUco marker ring (radius ~120 mm), so the marker
        # mounting frame can't be the largest above-plane component.
        self._platform_depth_mask: Optional[np.ndarray] = None
        self._expected_depth: Optional[np.ndarray] = None
        # Latest RGB raw frame, cached for the depth-debug overlay.
        # _tick already gets RGB raw for V0; we just stash it.
        self._last_rgb_bgr: Optional[np.ndarray] = None
        # Diagnostic counters — published every 5 s so we can tell at
        # a glance where the V0 / depth-blob pipeline is dying when
        # it does. All increment in their respective handlers.
        self._n_pose = 0
        self._n_depth_frame = 0
        self._n_v0 = 0
        self._n_depth_blob = 0
        self._diag_t0 = time.monotonic()
        self.create_timer(5.0, self._log_pipeline_health)

        # Toggle for V0 platform masking. Default on. Set
        # OAK_MASK_V0_TO_PLATFORM=0 to fall back to the old greedy V0
        # if you want to compare or if the mask is suppressing a real
        # ball detection while debugging.
        self.mask_v0_to_platform = (
            os.environ.get('OAK_MASK_V0_TO_PLATFORM', '1') == '1')
        self.get_logger().info(
            f"V0 platform-masking: "
            f"{'ON' if self.mask_v0_to_platform else 'OFF'} "
            f"(toggle via OAK_MASK_V0_TO_PLATFORM=0|1)")

        self.create_subscription(
            PoseStamped, '/platform_pose', self._on_pose, 10)

        # Half-width of the depth ROI around the V0 pixel, in
        # normalized RGB coordinates. 24 px (48 px wide ROI) covers
        # most of the foam ball at typical 600-800 mm range, which
        # gives the SLC enough valid-depth pixels to average over —
        # an 8-px ROI sat *inside* the ball where the foam surface
        # is smoothest, hitting too many zero-depth pixels and
        # leaving /ball_xy_oak at <1 Hz. SLC's depthThresholds (set
        # below per ROI) drop both invalid depth (= 0) and out-of-
        # range pixels, so a slightly wider ROI doesn't drag the
        # measurement toward the platform plane behind the ball.
        self._slc_roi_half_norm = 24.0 / RGB_W

        # Drive everything from a single timer; DepthAI queues are
        # already producer-buffered, so we just drain them.
        self.create_timer(1.0 / 60.0, self._tick)

    def _on_pose(self, msg: PoseStamped):
        """Cache the latest platform pose. Mask + expected-depth-map
        are recomputed lazily on the next _tick that needs them — see
        _refresh_mask_if_stale."""
        self._last_pose = msg
        self._n_pose += 1

    def _log_pipeline_health(self):
        """Periodic snapshot so we can tell which stage of the V0 /
        depth-blob pipeline is silent without ros2 topic hz."""
        mask_state = ('computed' if self._platform_mask is not None
                      else 'NONE')
        intrinsics_state = ('loaded' if self.K_rgb is not None
                            else 'NONE')
        depth_q = (self.q_depth is not None)
        self.get_logger().info(
            f"[health] poses={self._n_pose} v0_pub={self._n_v0} "
            f"depth_frames={self._n_depth_frame} "
            f"depth_blob_pub={self._n_depth_blob} "
            f"mask={mask_state} intrinsics={intrinsics_state} "
            f"depth_queue={'YES' if depth_q else 'NO'} "
            f"mask_v0={self.mask_v0_to_platform}")
        # Sanity-probe the depth math: at the principal point, what's
        # the ArUco-derived expected depth vs. what stereo measured?
        # On a flat platform with markers coplanar with the deck,
        # these should agree within a few mm. A 100+ mm gap means
        # there's a units or sign bug, not a geometry feature.
        if (self._last_pose is not None
                and self._expected_depth is not None
                and self._last_rgb_bgr is not None
                and self.q_depth is not None):
            try:
                p = self._last_pose.pose
                pose_z_mm = float(p.position.z) * 1000.0
                cy_idx = self._expected_depth.shape[0] // 2
                cx_idx = self._expected_depth.shape[1] // 2
                exp_center = float(self._expected_depth[cy_idx, cx_idx])
                # Last measured depth — pulled fresh on a tryGet
                msg = self.q_depth.tryGet()
                if msg is not None:
                    df = msg.getFrame()
                    h, w = df.shape[:2]
                    sub = df[h // 2 - 5:h // 2 + 5,
                             w // 2 - 5:w // 2 + 5]
                    valid_center = sub[sub > 0]
                    meas = float(np.median(valid_center)) if valid_center.size else 0.0
                    # Frame-wide stats so we can see whether the
                    # whole image reads at the wrong scale or just
                    # the center patch.
                    valid_all = df[df > 0]
                    if valid_all.size:
                        d_min = float(np.min(valid_all))
                        d_p25 = float(np.percentile(valid_all, 25))
                        d_med = float(np.median(valid_all))
                        d_p75 = float(np.percentile(valid_all, 75))
                        d_max = float(np.max(valid_all))
                        invalid_frac = float(np.sum(df == 0)) / float(df.size)
                    else:
                        d_min = d_p25 = d_med = d_p75 = d_max = 0.0
                        invalid_frac = 1.0
                    self.get_logger().info(
                        f"[probe] pose.z={pose_z_mm:.0f}mm "
                        f"exp_center={exp_center:.0f}mm "
                        f"meas_center={meas:.0f}mm "
                        f"depth_shape={df.shape} "
                        f"mask_shape={self._platform_mask.shape} "
                        f"dtype={df.dtype}")
                    self.get_logger().info(
                        f"[probe] depth_full: "
                        f"min={d_min:.0f} p25={d_p25:.0f} "
                        f"median={d_med:.0f} p75={d_p75:.0f} "
                        f"max={d_max:.0f} "
                        f"invalid={invalid_frac*100:.1f}%")
                    # Sample row at image vertical center, 5 columns
                    # spread evenly. Tells us if the wrong-scale
                    # reading is image-wide or pocketed.
                    row = df[h // 2, :]
                    pts = []
                    for col in (0, w // 4, w // 2, 3 * w // 4, w - 1):
                        v = int(row[col])
                        pts.append(f"x={col}:{v}")
                    self.get_logger().info(
                        f"[probe] depth_row(y={h//2}): " + "  ".join(pts))
            except Exception as e:
                self.get_logger().info(f"[probe] failed: {e}")

    def _refresh_mask_if_stale(self):
        """Project the platform disk into image pixels and refresh the
        per-pixel expected-plane depth map whenever the cached pose
        timestamp is older than what we received last. Both are
        invariant in the pose so a quick stamp comparison gates the
        recompute."""
        if self.K_rgb is None or self._last_pose is None:
            return
        stamp = self._last_pose.header.stamp
        stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if (self._mask_pose_stamp is not None
                and abs(stamp_s - self._mask_pose_stamp) < 1e-6):
            return
        p = self._last_pose.pose
        t_pose = np.array(
            [p.position.x, p.position.y, p.position.z], dtype=np.float64)
        R_pose = _quat_to_rot(
            p.orientation.w, p.orientation.x,
            p.orientation.y, p.orientation.z).astype(np.float64)
        poly = _project_platform_polygon(
            self.K_rgb, self.dist_rgb, R_pose, t_pose)
        if poly is None:
            self._platform_mask = None
            self._platform_depth_mask = None
            self._expected_depth = None
            self._mask_pose_stamp = stamp_s
            return
        # cv2 image axes are (row=H, col=W).
        self._platform_mask = _platform_image_mask((RGB_H, RGB_W), poly)
        # Tight mask for depth-blob: inside the marker ring, so the
        # raised marker hardware doesn't win the largest-blob contest.
        depth_poly = _project_platform_polygon(
            self.K_rgb, self.dist_rgb, R_pose, t_pose,
            radius_m=PLATFORM_DEPTH_MASK_RADIUS_M)
        self._platform_depth_mask = (
            _platform_image_mask((RGB_H, RGB_W), depth_poly)
            if depth_poly is not None else None)
        self._expected_depth = _expected_plane_depth_map(
            (RGB_H, RGB_W), self.K_rgb, R_pose, t_pose)
        self._mask_pose_stamp = stamp_s

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
        # Refresh the projected platform mask + expected-plane depth
        # map whenever the latest pose is newer than what we baked
        # into the cache. Cheap when stamps match; ~3 ms when they
        # don't.
        self._refresh_mask_if_stale()

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
            self._last_rgb_bgr = frame
            v0_mask = self._platform_mask if self.mask_v0_to_platform else None
            det = detect_ball_v0(frame, restrict_mask=v0_mask)
            if det is not None:
                cx, cy, r, conf = det
                p = PointStamped()
                p.header.stamp = self.get_clock().now().to_msg()
                p.header.frame_id = 'oak_rgb'
                p.point.x = cx
                p.point.y = cy
                p.point.z = r           # radius in pixels piggy-backed in z
                self.pub_v0.publish(p)
                self._n_v0 += 1
                # diagnostic with confidence + radius for debug strip
                d = Float32MultiArray()
                d.data = [float(cx), float(cy), float(r), float(conf)]
                self.pub_v0_diag.publish(d)

                # Push a fresh ROI to the SpatialLocationCalculator so
                # depth is sampled exactly where the ball was seen.
                # Skip when depth subsystem is disabled.
                if self.q_slc_cfg is not None:
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
        # Only present when the depth subsystem is enabled.
        if self.q_spatial is not None:
            spatial = self.q_spatial.tryGet()
            if spatial is not None:
                data = spatial.getSpatialLocations()
                if data:
                    d0 = data[0]
                    P_mm = d0.spatialCoordinates
                    if abs(P_mm.z) > 1.0:
                        sp = PointStamped()
                        sp.header.stamp = self.get_clock().now().to_msg()
                        sp.header.frame_id = 'oak_rgb'
                        sp.point.x = float(P_mm.x) * 1e-3
                        sp.point.y = float(P_mm.y) * 1e-3
                        sp.point.z = float(P_mm.z) * 1e-3
                        self.pub_spatial.publish(sp)

        # Depth-image → depth-blob detector → /oak/ball/depth/rgb_pixel.
        # Independent of V0; finds the largest blob whose measured
        # depth is 5..80 mm closer to the camera than the platform
        # plane's expected depth at that pixel. Requires the same
        # platform mask V0 uses.
        if self.q_depth is not None:
            depth_msg = self.q_depth.tryGet()
            if depth_msg is not None:
                self._n_depth_frame += 1
                if (self._platform_depth_mask is not None
                        and self._expected_depth is not None):
                    depth_frame = depth_msg.getFrame()  # uint16 mm, (H, W)
                    det = detect_ball_depth_blob(
                        depth_frame, self._platform_depth_mask,
                        self._expected_depth)
                    above_mask = None
                    eroded_mask = None
                    plane_offset_mm = 0.0
                    blob_for_dbg = None
                    if det is not None:
                        (cx, cy, area_px, conf, plane_offset_mm,
                         above_mask, eroded_mask) = det
                        blob_for_dbg = (cx, cy, area_px)
                        p = PointStamped()
                        p.header.stamp = self.get_clock().now().to_msg()
                        p.header.frame_id = 'oak_rgb'
                        p.point.x = cx
                        p.point.y = cy
                        # piggy-back blob-area-derived radius proxy in
                        # z (sqrt(area / pi)) so the GUI overlay can
                        # size the cyan circle from a single
                        # PointStamped, like V0's radius piggy-back.
                        p.point.z = float(np.sqrt(max(area_px, 1.0) / np.pi))
                        self.pub_depth_pixel.publish(p)
                        self._n_depth_blob += 1
                        d = Float32MultiArray()
                        # [cx, cy, area, confidence, plane_offset_mm]
                        # The last field is the empirical median
                        # height across the platform mask — large
                        # absolute values mean the ArUco-derived
                        # plane is biased and the self-cal is
                        # absorbing it. Watch this in the GUI.
                        d.data = [float(cx), float(cy),
                                  float(area_px), float(conf),
                                  float(plane_offset_mm)]
                        self.pub_depth_diag.publish(d)

                    # Debug overlay — render even on detection failure
                    # so we can see which platform pixels were eligible
                    # and where the spurious blobs are landing. We pass
                    # the *depth* mask (tight, inside-marker-ring) as
                    # the "platform mask" so the cyan outline shows
                    # exactly the region the detector considered.
                    if (self.depth_debug_enabled
                            and self._last_rgb_bgr is not None):
                        n_above = (int(np.sum(above_mask > 0))
                                   if above_mask is not None else 0)
                        dbg = render_depth_debug_overlay(
                            self._last_rgb_bgr,
                            self._platform_depth_mask, eroded_mask,
                            above_mask, blob_for_dbg,
                            plane_offset_mm, n_above=n_above)
                        ok, buf = cv2.imencode(
                            '.jpg', dbg,
                            [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                        if ok:
                            self._publish_compressed(
                                self.pub_depth_dbg, buf.tobytes(),
                                'jpeg')

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
