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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32, Float32MultiArray, String

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
# the SLC ROI normalization both reference this — they are the canonical
# image-space dimensions used by ball_localizer's K_rgb intrinsics.
RGB_W, RGB_H = 960, 540

# Mono cameras at THE_800_P native (1280 × 800).
MONO_W, MONO_H = 1280, 800

# Phase 2B reference target — when on-device V0 (NN blob) lands, the
# detector will run on a 320×180 ImageManip output. Kept here as a
# spec'd dimension for the blob's input shape; not consumed by the
# current pipeline (Phase 2A's downscale was reverted because the cv2
# detector struggled with a 5-px-radius ball). See
# docs/oak_phase2b_on_device_v0.md.
V0_W, V0_H = 320, 180


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
# With self-cal removed (probe data confirmed pose math is accurate
# within 20 mm), the depth-blob detector no longer needs a tight
# mask to keep the median honest — invalid stereo on the smooth
# deck just produces height=-inf, which fails the threshold
# naturally. Keep the depth mask just slightly tighter than the
# physical 200 mm deck so the rim's stereo discontinuity doesn't
# leak in: 0.190 m + 12 px erosion is well inside the rim.
PLATFORM_DEPTH_MASK_RADIUS_M = 0.190


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

# Acceptable height-above-platform window for a ball. The visible top
# hemisphere of a 40 mm ball ranges 0..40 mm above the platform plane;
# the OAK's stereo + ArUco pose math has been measured agreeing within
# ~20 mm, so a [3, 80] mm window captures the ball cap reliably while
# rejecting platform pixels (which read 0±5 mm above plane in valid
# stereo regions) and off-platform background (deeply negative).
BALL_HEIGHT_MIN_MM = 3.0
BALL_HEIGHT_MAX_MM = 80.0
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
        -> Tuple[Optional[tuple], dict]:
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

    Returns (blob_or_None, stats_dict). Stats are filled at every
    intermediate gate so the periodic [probe] log can pinpoint why
    a frame produced no detection.
    """
    stats: dict = {
        'mask_pixels': 0, 'eroded_pixels': 0,
        'valid_in_eroded': 0, 'n_above': 0,
        'max_height_mm': 0.0, 'min_height_mm_in_mask': 0.0,
        'median_height_in_mask_mm': 0.0,
        'largest_blob_area': 0, 'fail_reason': 'cv2_missing',
    }
    if cv2 is None:
        return None, stats
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
    # the height test fails for those pixels. Stereo on the user's
    # carbon-fiber + vinyl deck is sparse (probe data 2026-04-29
    # showed ~18 % of the frame invalid, with most platform-interior
    # pixels at 0); marking them +inf just means the height test
    # cleanly excludes them instead of producing a misleading
    # negative.
    invalid = measured <= 1.0
    measured_safe = measured.copy()
    measured_safe[invalid] = np.inf
    height = expected_depth_mm - measured_safe   # +ve = closer than plane

    # Earlier versions ran a per-frame "median plane offset" self-cal
    # to absorb assumed ArUco bias. Probe data showed pose.z and
    # expected_depth_at_principal_point agree within ~20 mm, so the
    # self-cal was over-correcting. Worse: with invalid stereo
    # dominating the platform interior, the median was being driven
    # by sparse stereo edge artifacts at ~1.5 m, dragging the offset
    # to −500 and shifting the threshold away from the ball every
    # frame. Use the ArUco plane directly. plane_offset stays in the
    # diagnostic tuple for backwards-compat / overlay HUD.
    plane_offset = 0.0

    # Capture stats now that we have everything: how many mask
    # pixels, how many had valid stereo, height distribution.
    stats['mask_pixels'] = int(np.sum(platform_mask > 0))
    stats['eroded_pixels'] = int(np.sum(eroded > 0))
    valid_mask_pixels = (~invalid) & (eroded > 0)
    stats['valid_in_eroded'] = int(np.sum(valid_mask_pixels))
    if stats['valid_in_eroded']:
        valid_heights = height[valid_mask_pixels]
        stats['max_height_mm'] = float(np.max(valid_heights))
        stats['min_height_mm_in_mask'] = float(np.min(valid_heights))
        stats['median_height_in_mask_mm'] = float(np.median(valid_heights))

    above = (height > h_min_mm) & (height < h_max_mm)
    above &= eroded > 0
    stats['n_above'] = int(np.sum(above))
    if not np.any(above):
        stats['fail_reason'] = 'no_pixels_above'
        return None, stats
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
    n_lab, labels, cc_stats, cents = cv2.connectedComponentsWithStats(
        bin_img, connectivity=8)
    # label 0 is background — skip it
    if n_lab <= 1:
        stats['fail_reason'] = 'no_components'
        return None, stats
    areas = cc_stats[1:, cv2.CC_STAT_AREA]
    j = int(np.argmax(areas)) + 1
    area = float(cc_stats[j, cv2.CC_STAT_AREA])
    stats['largest_blob_area'] = int(area)
    if area < min_area_px:
        stats['fail_reason'] = 'blob_below_min_area'
        return None, stats
    cx, cy = cents[j]
    # Confidence = clipped fraction of the largest blob's bounding-box
    # area filled with above-plane pixels. A perfect ball cap fills
    # ~π/4 of its bbox; we normalize against that.
    bbox_w = float(cc_stats[j, cv2.CC_STAT_WIDTH])
    bbox_h = float(cc_stats[j, cv2.CC_STAT_HEIGHT])
    bbox_area = max(bbox_w * bbox_h, 1.0)
    fill = area / bbox_area
    confidence = float(np.clip(fill / (np.pi / 4.0), 0.0, 1.0))
    stats['fail_reason'] = 'ok'
    return ((float(cx), float(cy), float(area), confidence,
             plane_offset, bin_img, eroded), stats)


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

def _build_pipeline(rgb_fps: int = 60, mono_fps: int = 15,
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
    # ISP downscale mode — env-toggleable. Investigating a 20 Hz cap
    # that survived USB-3 verification (5000M lsusb), depthai 2.32,
    # manual focus + manual exposure. Suspect the ISP scaler block
    # is rate-limiting at 1080p→540p. OAK_ISP_SCALE values:
    #   half (default): setIspScale(1, 2) → 540p out, half-area
    #   full          : skip ISP scale entirely → native 1080p out
    #   2x3           : setIspScale(2, 3) → 720p out (less work for
    #                   the scaler, larger frames downstream)
    isp_mode = os.environ.get('OAK_ISP_SCALE', 'half').lower()
    if isp_mode == 'full':
        # No ISP downscale — RGB output at native 1080p. ~6 MB raw
        # frame; at 60 fps that's 360 MB/s, fine on USB-3 SuperSpeed
        # (~400 MB/s sustained). cv2 V0 work scales with bbox area,
        # so HSV+contour at 1080p with bbox crop is ~12 ms vs ~3 ms
        # at 540p — still well under the 16.7 ms tick budget.
        pass
    elif isp_mode == '2x3':
        cam_rgb.setIspScale(2, 3)          # → 720p
    else:
        cam_rgb.setIspScale(1, 2)          # → 540p (legacy default)
    cam_rgb.setInterleaved(False)
    # Bump the camera's internal frame pool so the sensor doesn't stall
    # waiting on downstream consumers when one of them (encoder, host
    # queue) is briefly behind. Default is small; with multiple
    # consumers (rgb_jpeg + rgb_raw + the V0 path) at 60 Hz this is
    # the kind of back-pressure that silently caps frame rate.
    try:
        cam_rgb.setNumFramesPool(2, 3, 4, 4, 4)  # raw, isp, preview, video, still
    except Exception:
        pass
    # Manual focus + manual exposure to deterministically hit 60 fps.
    # CONTINUOUS_VIDEO autofocus pauses the sensor while it refocuses
    # (well-documented FPS killer on OAK-D Pro AF); auto-exposure
    # ignored our setAutoExposureLimit and stayed at ~67 ms exposure
    # in the 11:44 UTC test, capping us to ~15 Hz. Both modes are
    # replaced with fixed values, env-overridable for tuning:
    #   OAK_FOCUS_POS    — 0=infinity, 255=closest macro. Default 145
    #                      ≈ 50-60 cm; the platform sits at ~600 mm.
    #   OAK_EXP_US       — exposure time per frame in µs. Default
    #                      8000 (= 1/125 s). Cap is 1/exposure, so
    #                      8 ms allows up to 125 fps; 16 ms allows
    #                      62 fps; 33 ms caps at 30 fps.
    #   OAK_ISO          — analog gain. Default 800. Raise to brighten
    #                      (1600 / 3200) at the cost of more noise;
    #                      HSV detection holds up to ~3200.
    try:
        focus_pos = max(0, min(255,
            int(os.environ.get('OAK_FOCUS_POS', '145'))))
    except Exception:
        focus_pos = 145
    try:
        exp_us = max(500, min(33000,
            int(os.environ.get('OAK_EXP_US', '8000'))))
    except Exception:
        exp_us = 8000
    try:
        iso = max(100, min(6400,
            int(os.environ.get('OAK_ISO', '800'))))
    except Exception:
        iso = 800
    cam_rgb.initialControl.setManualFocus(focus_pos)
    cam_rgb.initialControl.setManualExposure(exp_us, iso)

    # Mono left — raw output for ArUco pose recovery
    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono_l.setFps(mono_fps)

    # JPEG encoder for RGB stream
    enc_rgb = pipeline.create(dai.node.VideoEncoder)
    enc_rgb.setDefaultProfilePreset(
        rgb_fps, dai.VideoEncoderProperties.Profile.MJPEG)
    # Quality 50 (down from 70) — at 540p MJPEG that's ~35-45 KB/frame
    # vs ~70-90 KB at q70. Smaller per-frame payload reduces the
    # transfer + queue time inside the OAK pipeline, which directly
    # cuts the 115 ms see→Pi median latency we measured. V0 reads the
    # uncompressed cam_rgb.video stream (not the JPEG output), so this
    # only affects the GUI live feed.  Override via
    # OAK_RGB_JPEG_QUALITY=NN if you need crisper screenshots.
    try:
        rgb_quality = int(os.environ.get('OAK_RGB_JPEG_QUALITY', '50'))
        rgb_quality = max(20, min(100, rgb_quality))
        enc_rgb.setQuality(rgb_quality)
    except Exception:
        pass
    # Drop-newest-on-overflow at the encoder input: prevents frame
    # backlog inside the OAK if encoding lags. A 7-frame queue at
    # 60 Hz capture is exactly what the 115 ms latency profile was
    # showing — non-blocking + queue-of-1 forces "always encode the
    # newest, drop the older". Same idea for the encoder's frame pool.
    enc_rgb.input.setQueueSize(1)
    enc_rgb.input.setBlocking(False)
    enc_rgb.setNumFramesPool(2)
    cam_rgb.video.link(enc_rgb.input)

    # Outputs
    def add_out(name, src):
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(name)
        src.link(out.input)
        return out

    add_out('rgb_jpeg', enc_rgb.bitstream)
    # Full-resolution raw RGB for V0 detection. We tried an on-device
    # ImageManip downscale to 320×180 (Phase 2A, 2026-04-30) to save
    # USB bandwidth; on USB-3 that wasn't binding, but the small frame
    # made the cv2 HSV+contour detector flaky (ball ~5 px radius is
    # right on the morph-open cliff edge). Bag data showed V0 success
    # rate dropping from 8 Hz to <2 Hz. Reverted — Phase 2B's on-device
    # NN path (see docs/oak_phase2b_on_device_v0.md) remains the right
    # move once a custom .blob is compiled.
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
    # Extended disparity DOUBLES the disparity range (95 → 190),
    # which halves the minimum measurable distance from ~632 mm to
    # ~316 mm. The user's platform sits at ~600 mm at center and
    # ~508 mm at the closest point — without ExtendedDisparity, the
    # entire platform + ball is BELOW the OAK's stereo minimum,
    # which is why bag 20260430T000925Z showed every frame failing
    # with "no_pixels_above" (depth_min=632 always, meas_center=4587
    # — wrong-match to background past the platform). With it, the
    # ball at 560 mm sits comfortably inside the measurable range.
    stereo.setExtendedDisparity(True)
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
        # Periodic health snapshot (per-stream Hz + per-path latency)
        # so demo bags carry the same info the [health] log lines do.
        # Field order matches the [health] log so post-hoc parsing is
        # consistent: [v0_arr_hz, v0_pub_hz, jpeg_hz, depth_hz,
        #              depth_pub_hz, pose_hz, jpeg_lat_ms, v0_lat_ms].
        self.pub_health = self.create_publisher(
            Float32MultiArray, '/oak/health', 10)
        # Configuration snapshot (rgb_fps, mono_fps, jpeg_quality,
        # enable_depth, etc.) so the digest can correlate behavior
        # changes with config changes — the user's "what tweak made
        # this work?" question deserves a record.
        self.pub_config = self.create_publisher(
            String, '/oak/config', 10)
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
        # Boot banner — prints the git sha of the running code so
        # we can verify a `git pull` actually took effect after a
        # restart. If the SHA in the journal doesn't match `git
        # rev-parse HEAD` on disk, the service didn't restart.
        try:
            import subprocess
            sha = subprocess.run(
                ['git', '-C', os.path.expanduser('~/stable_bot_repo'),
                 'rev-parse', '--short', 'HEAD'],
                capture_output=True, text=True, timeout=2.0)
            sha_str = sha.stdout.strip() or '?'
        except Exception:
            sha_str = '?'
        self.get_logger().info(
            f"[boot] oak_driver code sha={sha_str} "
            f"(check matches `git -C ~/stable_bot_repo rev-parse "
            f"--short HEAD` to confirm restart picked up new code)")

        # FPS — env-overridable. Default RGB 60 Hz to push V0 latency
        # as low as the IMX378 sensor allows. Earlier 60 Hz attempts
        # collapsed V0 because depth subsystem was simultaneously
        # consuming mono+stereo bandwidth on USB; with depth now
        # explicitly OFF (Environment=OAK_ENABLE_DEPTH=0 in the
        # systemd unit), RGB has the bus to itself and 60 Hz at
        # 540p ISP is well within USB-3 SuperSpeed budget (~93 MB/s
        # vs ~400 MB/s sustained ceiling).
        #
        # Mono stays at 15 Hz: it's used only for vision-debug bag
        # recording and (rarely) calibration sessions now that
        # ArUco runs on RGB.
        #
        # Drop OAK_RGB_FPS to 30 first if you see the [health] log's
        # v0_arr falling below the configured rate — that's the
        # signal the OAK is throttling under whatever the next
        # bottleneck is.
        try:
            rgb_fps = max(5, min(120, int(os.environ.get('OAK_RGB_FPS', '60'))))
        except Exception:
            rgb_fps = 60
        try:
            mono_fps = max(5, min(60, int(os.environ.get('OAK_MONO_FPS', '15'))))
        except Exception:
            mono_fps = 15
        try:
            jpeg_q = max(20, min(100,
                int(os.environ.get('OAK_RGB_JPEG_QUALITY', '50'))))
        except Exception:
            jpeg_q = 50
        # Stash for /oak/config snapshot (read again inside
        # _build_pipeline from env so the values stay in sync).
        self._rgb_fps_used = rgb_fps
        self._mono_fps_used = mono_fps
        self._jpeg_quality_used = jpeg_q
        self.get_logger().info(
            f"Building OAK pipeline (rgb={rgb_fps} Hz, mono={mono_fps} Hz)…")
        pipeline = _build_pipeline(rgb_fps=rgb_fps, mono_fps=mono_fps,
                                   enable_depth=self.enable_depth)

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
        # Bounding box of the projected platform polygon, cached
        # alongside _platform_mask so V0's HSV+contour pipeline can
        # crop to just the platform region instead of running on the
        # full 540p frame. Tuple (x0, y0, x1, y1) in image coords, or
        # None if no pose / off-screen polygon.
        self._platform_bbox: Optional[Tuple[int, int, int, int]] = None
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
        self._n_v0 = 0          # V0 detector publish (success only)
        self._n_v0_attempts = 0 # V0 detector ATTEMPTS (every RGB raw frame)
        self._n_rgb_jpeg = 0    # frames pulled off the JPEG queue
        self._n_depth_blob = 0
        # Last-log timestamp so health prints true Hz, not cumulative.
        self._health_last_t = time.time()
        # Last-frame age (seconds) for the V0 raw path — populated on
        # every RGB raw frame, surfaced by /oak/health.
        self._v0_last_age_s = float('nan')
        self._jpeg_last_age_s = float('nan')
        # 30s timer cadence for /oak/config snapshots.
        self._config_last_pub_t = 0.0
        self._diag_t0 = time.monotonic()
        # Per-frame depth detector stats from the last detection
        # attempt. Surfaced in the [probe] log so we can see which
        # gate (no above-plane pixels, blob too small, etc.) is
        # filtering out the ball without re-running the algorithm.
        self._last_depth_stats: Optional[dict] = None
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

        # Publish initial /oak/config so any subscriber that joins
        # before the first 30 s health tick has the snapshot in hand.
        self._publish_config_snapshot()

    def _publish_config_snapshot(self):
        """Emit a JSON snapshot of every OAK-side tunable so the
        digest can correlate demo outcomes with config state."""
        try:
            import json as _json
            cfg = {
                'rgb_fps': int(self._rgb_fps_used),
                'mono_fps': int(self._mono_fps_used),
                'jpeg_quality': int(self._jpeg_quality_used),
                'focus_pos': int(os.environ.get('OAK_FOCUS_POS', '145')),
                'exp_us':    int(os.environ.get('OAK_EXP_US', '8000')),
                'iso':       int(os.environ.get('OAK_ISO', '800')),
                'enable_depth': bool(self.enable_depth),
                'mask_v0_to_platform': bool(self.mask_v0_to_platform),
                'usb_speed': os.environ.get('OAK_USB_SPEED', '?'),
                'rgb_w': RGB_W, 'rgb_h': RGB_H,
            }
            m = String()
            m.data = _json.dumps(cfg, separators=(',', ':'))
            self.pub_config.publish(m)
        except Exception as e:
            self.get_logger().warn(f"config snapshot failed: {e}")

    def _on_pose(self, msg: PoseStamped):
        """Cache the latest platform pose. Mask + expected-depth-map
        are recomputed lazily on the next _tick that needs them — see
        _refresh_mask_if_stale."""
        self._last_pose = msg
        self._n_pose += 1

    def _log_pipeline_health(self):
        """Periodic snapshot so we can tell which stage of the V0 /
        depth-blob pipeline is silent without ros2 topic hz. Reports
        per-stream Hz over the last interval so we can see whether the
        OAK is actually delivering frames at the requested FPS.

        v0_arr  = RGB raw frames the host pulled off the queue
        v0_pub  = V0 detector successes (subset of v0_arr)
        jpeg    = MJPEG frames the host pulled off the queue
        depth   = depth frames the host pulled off the queue
        depth_pub = depth-blob detector successes (subset of depth)

        If v0_arr is far below the configured RGB FPS, the OAK isn't
        producing frames at the rate we asked for (USB / pipeline
        scheduling problem). If v0_arr ≈ FPS but v0_pub is much lower,
        detection itself is failing (HSV / mask / lighting)."""
        now = time.time()
        dt = max(now - self._health_last_t, 1e-3)
        self._health_last_t = now
        v0_arr_hz = self._n_v0_attempts / dt
        v0_pub_hz = self._n_v0 / dt
        jpeg_hz   = self._n_rgb_jpeg / dt
        depth_hz  = self._n_depth_frame / dt
        depth_pub_hz = self._n_depth_blob / dt
        pose_hz   = self._n_pose / dt
        # Reset counters for the next interval.
        self._n_v0_attempts = 0
        self._n_v0 = 0
        self._n_rgb_jpeg = 0
        self._n_depth_frame = 0
        self._n_depth_blob = 0
        self._n_pose = 0
        mask_state = ('computed' if self._platform_mask is not None
                      else 'NONE')
        intrinsics_state = ('loaded' if self.K_rgb is not None
                            else 'NONE')
        depth_q = (self.q_depth is not None)
        jpeg_lat_ms = (self._jpeg_last_age_s * 1000.0
                       if self._jpeg_last_age_s == self._jpeg_last_age_s
                       else float('nan'))
        v0_lat_ms = (self._v0_last_age_s * 1000.0
                     if self._v0_last_age_s == self._v0_last_age_s
                     else float('nan'))
        self.get_logger().info(
            f"[health] v0_arr={v0_arr_hz:.1f}Hz "
            f"v0_pub={v0_pub_hz:.1f}Hz "
            f"jpeg={jpeg_hz:.1f}Hz "
            f"depth={depth_hz:.1f}Hz "
            f"depth_pub={depth_pub_hz:.1f}Hz "
            f"pose={pose_hz:.1f}Hz "
            f"jpeg_lat={jpeg_lat_ms:.0f}ms "
            f"v0_lat={v0_lat_ms:.0f}ms "
            f"mask={mask_state} intr={intrinsics_state} "
            f"depthQ={'Y' if depth_q else 'N'} "
            f"mask_v0={self.mask_v0_to_platform}")
        # Same data into the bag via /oak/health.
        h = Float32MultiArray()
        h.data = [
            float(v0_arr_hz), float(v0_pub_hz),
            float(jpeg_hz), float(depth_hz),
            float(depth_pub_hz), float(pose_hz),
            float(jpeg_lat_ms), float(v0_lat_ms),
        ]
        self.pub_health.publish(h)
        # /oak/config — publish every health tick (5 s) so demo bags
        # always capture at least one snapshot regardless of run
        # duration. The String payload is tiny (~80 B); cost is
        # negligible.
        self._publish_config_snapshot()
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
                    # Last detector-algo internal stats. Tells us
                    # which gate killed the detection.
                    s = self._last_depth_stats
                    if s is not None:
                        self.get_logger().info(
                            f"[probe] depth_alg: "
                            f"mask={s.get('mask_pixels', 0)} "
                            f"eroded={s.get('eroded_pixels', 0)} "
                            f"valid_in_mask={s.get('valid_in_eroded', 0)} "
                            f"n_above={s.get('n_above', 0)} "
                            f"max_h={s.get('max_height_mm', 0):.0f}mm "
                            f"med_h={s.get('median_height_in_mask_mm', 0):.0f}mm "
                            f"largest_blob={s.get('largest_blob_area', 0)}px "
                            f"fail={s.get('fail_reason', '?')}")
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
            self._platform_bbox = None
            self._platform_depth_mask = None
            self._expected_depth = None
            self._mask_pose_stamp = stamp_s
            return
        # cv2 image axes are (row=H, col=W).
        self._platform_mask = _platform_image_mask((RGB_H, RGB_W), poly)
        # Cache the polygon's image-space bbox with a small margin so
        # V0 can crop to just the platform region (~1/4 of the frame
        # area) and skip cv2 work on the rest. ~75 % HSV/morph/contour
        # speedup, which is what gets us comfortably under the 16.7 ms
        # tick budget at 60 Hz.
        margin = 8
        x_min = int(np.clip(poly[:, 0].min() - margin, 0, RGB_W))
        y_min = int(np.clip(poly[:, 1].min() - margin, 0, RGB_H))
        x_max = int(np.clip(poly[:, 0].max() + margin, 0, RGB_W))
        y_max = int(np.clip(poly[:, 1].max() + margin, 0, RGB_H))
        if x_max > x_min and y_max > y_min:
            self._platform_bbox = (x_min, y_min, x_max, y_max)
        else:
            self._platform_bbox = None
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
            self._n_rgb_jpeg += 1
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
                age_s = max(0.0, now_ts - capture_ts)
                lat_msg = Float32()
                lat_msg.data = float(age_s * 1000.0)
                self.pub_latency.publish(lat_msg)
                self._jpeg_last_age_s = age_s
            except Exception:
                pass

        # RGB raw → V0 detector → /oak/ball/v0/rgb_pixel.
        # TODO(phase2b): replace this Pi-side cv2 detector with an
        # on-device NeuralNetwork node so V0 doesn't depend on the host
        # CPU at all. See docs/oak_phase2b_on_device_v0.md.
        rgb_raw = self.q_rgb_raw.tryGet()
        if rgb_raw is not None:
            self._n_v0_attempts += 1
            frame = rgb_raw.getCvFrame()  # BGR uint8
            self._last_rgb_bgr = frame
            # Translate the OAK frame timestamp into ROS-clock space
            # so /ball_xy_mono and /ball_state carry the actual capture
            # time, not "now". The controller's lookahead becomes
            # self-correcting: it can subtract the real age instead of
            # assuming a fixed control_latency_s. dai.Clock and the ROS
            # clock have different epochs (steady vs system), so we
            # compute the live offset and apply it.
            try:
                capture_dai = rgb_raw.getTimestamp().total_seconds()
                now_dai = dai.Clock.now().total_seconds()
                age_s = max(0.0, now_dai - capture_dai)
                ros_capture = self.get_clock().now() - Duration(
                    seconds=age_s)
                v0_stamp = ros_capture.to_msg()
                v0_age_s = age_s   # for /oak/health
            except Exception:
                v0_stamp = self.get_clock().now().to_msg()
                v0_age_s = float('nan')
            self._v0_last_age_s = v0_age_s
            # Pre-crop to the platform-mask bbox so HSV+morph+contour
            # only run on ~25 % of the frame area. Falls back to full-
            # frame if bbox isn't ready yet (no pose received).
            bbox = (self._platform_bbox
                    if (self.mask_v0_to_platform
                        and self._platform_bbox is not None)
                    else None)
            if bbox is not None:
                x0, y0, x1, y1 = bbox
                v0_frame = frame[y0:y1, x0:x1]
                v0_mask = (self._platform_mask[y0:y1, x0:x1]
                           if self._platform_mask is not None else None)
                det = detect_ball_v0(v0_frame, restrict_mask=v0_mask)
                if det is not None:
                    cx, cy, r, conf = det
                    cx += x0
                    cy += y0
                    det = (cx, cy, r, conf)
            else:
                v0_mask = (self._platform_mask
                           if self.mask_v0_to_platform else None)
                det = detect_ball_v0(frame, restrict_mask=v0_mask)
            if det is not None:
                cx, cy, r, conf = det
                p = PointStamped()
                p.header.stamp = v0_stamp
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
                depth_frame = depth_msg.getFrame()  # uint16 mm, (H, W)
                # Capture-time stamp for the depth pixel publish (see V0
                # block above for rationale).
                try:
                    capture_dai = depth_msg.getTimestamp().total_seconds()
                    now_dai = dai.Clock.now().total_seconds()
                    age_s = max(0.0, now_dai - capture_dai)
                    depth_stamp = (self.get_clock().now()
                                   - Duration(seconds=age_s)).to_msg()
                except Exception:
                    depth_stamp = self.get_clock().now().to_msg()

                # Frame-wide depth distribution (for the rich
                # diagnostic). Computed on every frame so the digest
                # can plot stereo health over time. ~3 ms on Pi 5
                # for a 540×960 frame.
                valid_all = depth_frame[depth_frame > 0]
                if valid_all.size:
                    d_min = float(np.min(valid_all))
                    d_p25 = float(np.percentile(valid_all, 25))
                    d_med = float(np.median(valid_all))
                    d_p75 = float(np.percentile(valid_all, 75))
                    d_max = float(np.max(valid_all))
                    invalid_pct = (
                        float(np.sum(depth_frame == 0))
                        / float(depth_frame.size) * 100.0)
                else:
                    d_min = d_p25 = d_med = d_p75 = d_max = 0.0
                    invalid_pct = 100.0

                # Probe-style center-pixel measurement.
                h_d, w_d = depth_frame.shape[:2]
                ctr_patch = depth_frame[h_d // 2 - 5:h_d // 2 + 5,
                                        w_d // 2 - 5:w_d // 2 + 5]
                ctr_valid = ctr_patch[ctr_patch > 0]
                meas_center_mm = (float(np.median(ctr_valid))
                                  if ctr_valid.size else 0.0)
                # Pose & expected — None when ArUco hasn't fired yet
                pose_z_mm = exp_center_mm = 0.0
                if (self._last_pose is not None
                        and self._expected_depth is not None):
                    pose_z_mm = float(self._last_pose.pose.position.z) * 1000.0
                    cy_idx = self._expected_depth.shape[0] // 2
                    cx_idx = self._expected_depth.shape[1] // 2
                    exp_center_mm = float(
                        self._expected_depth[cy_idx, cx_idx])

                if (self._platform_depth_mask is not None
                        and self._expected_depth is not None):
                    det, depth_stats = detect_ball_depth_blob(
                        depth_frame, self._platform_depth_mask,
                        self._expected_depth)
                    self._last_depth_stats = depth_stats
                    above_mask = None
                    eroded_mask = None
                    plane_offset_mm = 0.0
                    blob_for_dbg = None
                    cx_det = cy_det = float('nan')
                    area_det = conf_det = 0.0
                    if det is not None:
                        (cx_det, cy_det, area_det, conf_det,
                         plane_offset_mm,
                         above_mask, eroded_mask) = det
                        blob_for_dbg = (cx_det, cy_det, area_det)
                        p = PointStamped()
                        p.header.stamp = depth_stamp
                        p.header.frame_id = 'oak_rgb'
                        p.point.x = float(cx_det)
                        p.point.y = float(cy_det)
                        # piggy-back blob-area-derived radius proxy in
                        # z (sqrt(area / pi)) so the GUI overlay can
                        # size the cyan circle from a single
                        # PointStamped, like V0's radius piggy-back.
                        p.point.z = float(np.sqrt(max(area_det, 1.0) / np.pi))
                        self.pub_depth_pixel.publish(p)
                        self._n_depth_blob += 1

                    # Map fail_reason → numeric code for Float32MultiArray.
                    fail_code_map = {
                        'ok': 0.0, 'no_pixels_above': 1.0,
                        'no_components': 2.0, 'blob_below_min_area': 3.0,
                        'cv2_missing': 4.0,
                    }
                    fail_code = fail_code_map.get(
                        depth_stats.get('fail_reason', '?'), 9.0)
                    # Rich 20-field diagnostic published on EVERY
                    # depth frame so the digest can plot algorithm
                    # internals as time series. Backwards-compat:
                    # the first 5 fields preserve the old schema
                    # ([cx, cy, area, conf, plane_offset_mm]).
                    d = Float32MultiArray()
                    d.data = [
                        # 0-4: detection (NaN/0 when no detection)
                        float(cx_det), float(cy_det),
                        float(area_det), float(conf_det),
                        float(plane_offset_mm),
                        # 5-10: algo internals (every frame)
                        float(depth_stats.get('mask_pixels', 0)),
                        float(depth_stats.get('eroded_pixels', 0)),
                        float(depth_stats.get('valid_in_eroded', 0)),
                        float(depth_stats.get('n_above', 0)),
                        float(depth_stats.get('max_height_mm', 0.0)),
                        float(depth_stats.get('median_height_in_mask_mm', 0.0)),
                        # 11-12: blob + fail code
                        float(depth_stats.get('largest_blob_area', 0)),
                        fail_code,
                        # 13-15: probe-style pose vs measured
                        pose_z_mm, exp_center_mm, meas_center_mm,
                        # 16-19: frame-wide depth distribution
                        d_min, d_p25, d_med, d_p75,
                        # NB: d_max and invalid_pct go in fields 20-21
                        # but we cap at 22 fields total for the
                        # bag's wire format.
                        d_max, invalid_pct,
                    ]
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
