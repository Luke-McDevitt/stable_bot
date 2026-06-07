"""_kf_noise.py — estimate the ball KF's measurement noise R and the
false-positive outlier rate from a recorded bag, to tune ball_kf_node
(BALL_KF_R_MM / BALL_KF_GATE_CHI2 / BALL_KF_SIGMA_A).

The trick: R is the RAW-measurement scatter on a STATIONARY ball, so we isolate
"still" windows (the whole run on a meas-noise bag; the settled dwells on a demo
bag) and measure the spread there — robustly (MAD), so a false positive doesn't
inflate the R estimate (rejecting those is the gate's job, not R's). The outlier
rate (physically-impossible jumps) is what motivates the gate threshold.

Pure math (no ROS / numpy) → unit-tested in test/test_kf_noise.py; bag reading
lives in scripts/estimate_kf_noise.py.
"""
from __future__ import annotations

import math


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if not n:
        return 0.0
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def _robust_std(rs):
    """MAD-based std — ignores outliers (1.4826·MAD ≈ σ for a Gaussian)."""
    if len(rs) < 3:
        return 0.0
    med = _median(rs)
    return 1.4826 * _median([abs(v - med) for v in rs])


def still_windows(ts, px, py, range_mm=12.0, min_dur_s=0.8):
    """Maximal spans (t0, t1) where the ball position stays within `range_mm`
    — i.e. ~stationary. A meas-noise run is one big window; a demo's settled
    dwells are several."""
    n = len(ts)
    out = []
    i = 0
    while i < n:
        j = i
        xmin = xmax = px[i]
        ymin = ymax = py[i]
        while j + 1 < n:
            nx, ny = px[j + 1], py[j + 1]
            if (max(xmax, nx) - min(xmin, nx) > range_mm
                    or max(ymax, ny) - min(ymin, ny) > range_mm):
                break
            xmin, xmax = min(xmin, nx), max(xmax, nx)
            ymin, ymax = min(ymin, ny), max(ymax, ny)
            j += 1
        if ts[j] - ts[i] >= min_dur_s:
            out.append((ts[i], ts[j]))
            i = j + 1
        else:
            i += 1
    return out


def measurement_noise(meas_t, mx, my, windows):
    """Robust per-axis std of the RAW measurement, de-meaned within each still
    window → R (mm). Returns {'R_mm','std_x_mm','std_y_mm','n','n_windows'} or
    None."""
    rx, ry, nw = [], [], 0
    for t0, t1 in windows:
        idx = [k for k in range(len(meas_t)) if t0 <= meas_t[k] <= t1]
        if len(idx) < 5:
            continue
        nw += 1
        cx = _median([mx[k] for k in idx])
        cy = _median([my[k] for k in idx])
        rx += [mx[k] - cx for k in idx]
        ry += [my[k] - cy for k in idx]
    if len(rx) < 10:
        return None
    sx, sy = _robust_std(rx), _robust_std(ry)
    return {'R_mm': round((sx + sy) / 2, 3), 'std_x_mm': round(sx, 3),
            'std_y_mm': round(sy, 3), 'n': len(rx), 'n_windows': nw}


def outlier_rate(meas_t, mx, my, max_speed_mm_s=1500.0):
    """Fraction of consecutive raw measurements whose implied speed exceeds
    `max_speed_mm_s` — physically-impossible jumps = the false positives the
    innovation gate rejects. Returns (fraction, n_outliers, n_pairs)."""
    n_out = n = 0
    for i in range(1, len(meas_t)):
        dt = meas_t[i] - meas_t[i - 1]
        if dt <= 0:
            continue
        n += 1
        if math.hypot(mx[i] - mx[i - 1], my[i] - my[i - 1]) / dt > max_speed_mm_s:
            n_out += 1
    return (round(n_out / n, 4) if n else 0.0, n_out, n)
