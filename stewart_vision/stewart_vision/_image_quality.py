"""Pure image-quality metrics for focus + exposure auto-calibration.

Single source of truth for the sharpness / exposure numbers used by the
focus-sweep, exposure-sweep, and baseline-capture tools (see
../../stewart_bringup/docs/oak_focus_exposure_autocal.md).

Design goals:
  - **No cv2 / scipy dependency** — pure NumPy, so it runs and unit-tests
    anywhere (the sweep *scripts* use cv2 to decode JPEG, but the metric
    math lives here and is testable without a camera).
  - **Size-invariant** focus measures (means, not sums) so a metric
    computed on a small ball ROI is comparable to one on the full frame.
  - Operate on grayscale for sharpness; accept BGR for exposure /
    saturation (cv2.imdecode returns BGR).

Focus operators (higher = sharper), per the Pertuz et al. (2013) survey
and the OpenCV comparative study: variance-of-Laplacian, Tenengrad
(gradient energy), sum-of-modified-Laplacian, normalized variance.
Tenengrad is the most noise-robust — relevant at our high ISO.
"""
from __future__ import annotations

from typing import Dict

import numpy as np

# 3x3 operators.
_LAP_KERNEL = np.array([[0.0, 1.0, 0.0],
                        [1.0, -4.0, 1.0],
                        [0.0, 1.0, 0.0]])
_SOBEL_X = np.array([[-1.0, 0.0, 1.0],
                     [-2.0, 0.0, 2.0],
                     [-1.0, 0.0, 1.0]])
_SOBEL_Y = _SOBEL_X.T


def _as_gray_f64(img: np.ndarray) -> np.ndarray:
    """Return a float64 grayscale view. BGR (cv2 default) → luma; an
    already-2D array is used as-is."""
    a = np.asarray(img)
    if a.ndim == 3:
        c = a[..., :3].astype(np.float64)
        # BGR weights (cv2.imdecode order): 0.114 B + 0.587 G + 0.299 R.
        return c[..., 0] * 0.114 + c[..., 1] * 0.587 + c[..., 2] * 0.299
    return a.astype(np.float64)


def _correlate3(a: np.ndarray, k: np.ndarray) -> np.ndarray:
    """'valid' 3x3 correlation of 2D `a` with kernel `k`, no padding.
    Returns shape (H-2, W-2). Pure-slicing, no scipy."""
    h, w = a.shape[0] - 2, a.shape[1] - 2
    out = np.zeros((h, w), dtype=np.float64)
    for i in range(3):
        for j in range(3):
            kij = k[i, j]
            if kij != 0.0:
                out += kij * a[i:i + h, j:j + w]
    return out


def _too_small(g: np.ndarray) -> bool:
    return g.shape[0] < 3 or g.shape[1] < 3


# --- Focus / sharpness measures (higher = sharper) --------------------------

def variance_of_laplacian(img: np.ndarray) -> float:
    """Variance of the Laplacian — the classic, simple focus measure."""
    g = _as_gray_f64(img)
    if _too_small(g):
        return 0.0
    return float(_correlate3(g, _LAP_KERNEL).var())


def tenengrad(img: np.ndarray, threshold: float = 0.0) -> float:
    """Mean gradient energy (Gx^2 + Gy^2). Most noise-robust operator.
    `threshold` (on gradient magnitude) optionally zeros weak gradients."""
    g = _as_gray_f64(img)
    if _too_small(g):
        return 0.0
    gx = _correlate3(g, _SOBEL_X)
    gy = _correlate3(g, _SOBEL_Y)
    fm = gx * gx + gy * gy
    if threshold > 0.0:
        fm = np.where(fm >= threshold * threshold, fm, 0.0)
    return float(fm.mean())


def sum_modified_laplacian(img: np.ndarray) -> float:
    """Mean modified Laplacian: |2c - l - r| + |2c - u - d|. Avoids the
    sign-cancellation of the plain Laplacian."""
    g = _as_gray_f64(img)
    if _too_small(g):
        return 0.0
    c = g[1:-1, 1:-1]
    ml = (np.abs(2.0 * c - g[1:-1, :-2] - g[1:-1, 2:])
          + np.abs(2.0 * c - g[:-2, 1:-1] - g[2:, 1:-1]))
    return float(ml.mean())


def normalized_variance(img: np.ndarray) -> float:
    """Illumination-normalized variance: var / mean. More comparable
    across brightness changes than raw variance."""
    g = _as_gray_f64(img)
    m = g.mean()
    if m <= 1e-9:
        return 0.0
    return float(((g - m) ** 2).mean() / m)


# --- Exposure / brightness measures -----------------------------------------

def exposure_stats(img: np.ndarray, lo: float = 2.0, hi: float = 253.0) -> Dict[str, float]:
    """Brightness + clipping summary on the luma channel.

    `over_clip_frac` / `under_clip_frac` are the fraction of pixels at/above
    `hi` / at-or-below `lo` — both should be small for a usable exposure.
    """
    g = _as_gray_f64(img)
    p01, p50, p99 = (float(x) for x in np.percentile(g, [1, 50, 99]))
    return {
        'over_clip_frac': float((g >= hi).mean()),
        'under_clip_frac': float((g <= lo).mean()),
        'mean': float(g.mean()),
        'p01': p01,
        'p50': p50,
        'p99': p99,
        'dynamic_range': p99 - p01,
    }


def saturation_stats(bgr: np.ndarray) -> Dict[str, float]:
    """HSV-style saturation/value on a BGR image (cv2-free).
    S = (max-min)/max scaled to 0-255; V = max channel. The cv2 HSV ball
    detector needs the ball's S/V high, so this is the exposure metric
    that matters for detection."""
    a = np.asarray(bgr)
    if a.ndim != 3:
        return {'sat_mean': 0.0, 'sat_p50': 0.0, 'val_mean': 0.0}
    c = a[..., :3].astype(np.float64)
    mx = c.max(axis=2)
    mn = c.min(axis=2)
    s = np.where(mx > 1e-9, (mx - mn) / np.maximum(mx, 1e-9), 0.0) * 255.0
    return {
        'sat_mean': float(s.mean()),
        'sat_p50': float(np.percentile(s, 50)),
        'val_mean': float(mx.mean()),
    }


def image_quality_report(img: np.ndarray) -> Dict[str, float]:
    """All metrics in one dict. Pass BGR (preferred) or grayscale."""
    rep = {
        'variance_of_laplacian': variance_of_laplacian(img),
        'tenengrad': tenengrad(img),
        'sum_modified_laplacian': sum_modified_laplacian(img),
        'normalized_variance': normalized_variance(img),
    }
    rep.update(exposure_stats(img))
    if np.asarray(img).ndim == 3:
        rep.update(saturation_stats(img))
    return rep


# Canonical ordering for CSV columns / cross-tool consistency.
METRIC_KEYS = (
    'variance_of_laplacian', 'tenengrad', 'sum_modified_laplacian',
    'normalized_variance', 'over_clip_frac', 'under_clip_frac', 'mean',
    'p01', 'p50', 'p99', 'dynamic_range', 'sat_mean', 'sat_p50', 'val_mean',
)
