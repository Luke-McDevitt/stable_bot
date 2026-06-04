"""Unit tests for stewart_vision._image_quality — the focus/exposure
metric math used by the auto-calibration tools. No camera required.

If these pass, the sharpness ordering (sharp > blurred) and the exposure
clipping detection are trustworthy, which is what the focus/exposure
sweeps rely on to pick an optimum.
"""
import numpy as np
import pytest

from stewart_vision._image_quality import (
    METRIC_KEYS,
    exposure_stats,
    image_quality_report,
    normalized_variance,
    saturation_stats,
    sum_modified_laplacian,
    tenengrad,
    variance_of_laplacian,
)


def _checkerboard(n=64, square=4):
    """High-frequency pattern → maximally 'sharp'."""
    yy, xx = np.mgrid[0:n, 0:n]
    return (((yy // square) + (xx // square)) % 2 * 255).astype(np.uint8)


def _blur(g, iters=4):
    """Simple 5-point box blur, repeated. Reduces high-frequency content."""
    out = g.astype(np.float64)
    for _ in range(iters):
        nxt = out.copy()
        nxt[1:-1, 1:-1] = (
            out[:-2, 1:-1] + out[2:, 1:-1] + out[1:-1, :-2]
            + out[1:-1, 2:] + out[1:-1, 1:-1]) / 5.0
        out = nxt
    return out


# --- sharpness ordering: sharp > blurred -----------------------------------

@pytest.mark.parametrize("fm", [
    variance_of_laplacian, tenengrad, sum_modified_laplacian,
    normalized_variance,
])
def test_sharp_beats_blurred(fm):
    sharp = _checkerboard()
    blurred = _blur(sharp)
    assert fm(sharp) > fm(blurred), f"{fm.__name__} did not rank sharp>blurred"


@pytest.mark.parametrize("fm", [
    variance_of_laplacian, tenengrad, sum_modified_laplacian,
])
def test_constant_image_is_zero_sharpness(fm):
    flat = np.full((32, 32), 128, dtype=np.uint8)
    assert fm(flat) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("fm", [
    variance_of_laplacian, tenengrad, sum_modified_laplacian,
    normalized_variance,
])
def test_tiny_roi_does_not_crash(fm):
    assert fm(np.zeros((2, 2), dtype=np.uint8)) == 0.0


def test_progressive_blur_is_monotonic_ish():
    sharp = _checkerboard()
    a = variance_of_laplacian(_blur(sharp, 2))
    b = variance_of_laplacian(_blur(sharp, 6))
    assert a > b  # more blur → less sharpness


# --- exposure / clipping ----------------------------------------------------

def test_over_and_under_clip_detection():
    assert exposure_stats(np.full((16, 16), 255, np.uint8))['over_clip_frac'] == 1.0
    assert exposure_stats(np.full((16, 16), 0, np.uint8))['under_clip_frac'] == 1.0
    ramp = np.tile(np.linspace(10, 245, 64).astype(np.uint8), (16, 1))
    s = exposure_stats(ramp)
    assert s['over_clip_frac'] == 0.0 and s['under_clip_frac'] == 0.0
    assert s['dynamic_range'] > 0.0


# --- saturation -------------------------------------------------------------

def test_saturated_color_beats_gray():
    orange = np.zeros((16, 16, 3), np.uint8)
    orange[..., 2] = 230   # R high in BGR-order array's index 2
    orange[..., 1] = 90
    gray = np.full((16, 16, 3), 128, np.uint8)
    assert saturation_stats(orange)['sat_mean'] > saturation_stats(gray)['sat_mean']
    assert saturation_stats(gray)['sat_mean'] == pytest.approx(0.0, abs=1e-6)


# --- report contract --------------------------------------------------------

def test_report_has_all_keys():
    rep = image_quality_report(np.zeros((16, 16, 3), np.uint8))
    for k in METRIC_KEYS:
        assert k in rep, f"missing metric {k}"
    # grayscale input → no saturation keys, but focus/exposure present.
    rep_g = image_quality_report(np.zeros((16, 16), np.uint8))
    assert 'variance_of_laplacian' in rep_g and 'over_clip_frac' in rep_g
