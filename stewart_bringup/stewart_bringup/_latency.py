"""Latency math for the demo digests — pure NumPy, no ROS.

Extracted so the demo digest, `compare_demo_bags.py`, and any future
"ball-tap" analyzer share one verified implementation (mirrors the
`stewart_vision._image_quality` / `_ball_physics` pattern). Being
ROS-free, it is unit-tested on any machine (see
`stewart_bringup/test/test_latency.py`).

The headline tool here is `actuation_latency`: the *effective*
command->motion delay, measured by cross-correlating the commanded
tilt (`/ball_track/diagnostic`) against the measured platform tilt
(the platform IMU's fused orientation, ~240 Hz, or `/platform_rpy`
as a coarser fallback). This is the half of the control loop the
vision-only latency map (`oak_latency_map.md`) could never measure —
see `control_path_latency.md` §3.
"""
from __future__ import annotations

import numpy as np


def quat_to_roll_pitch_deg(quat):
    """ZYX roll/pitch (deg) from an (N,4) array of [qx,qy,qz,qw].
    The platform MTi-630 reports fused orientation, so this gives the
    high-rate 'what the platform actually did' tilt — finer than the
    ~17 Hz /platform_rpy topic. Vectorized; returns (roll, pitch)."""
    q = np.asarray(quat, dtype=np.float64).reshape(-1, 4)
    qx, qy, qz, qw = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(sinp))
    return roll, pitch


def resample_uniform(t_s, y, fs_hz, t0_s, t1_s):
    """Interp (t_s, y) onto a uniform grid [t0,t1] at fs. NaN outside
    the source span. Returns (grid_s, y_resampled)."""
    t_s = np.asarray(t_s, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    n = int(max(2, round((t1_s - t0_s) * fs_hz)))
    grid = t0_s + np.arange(n) / fs_hz
    if t_s.size < 2:
        return grid, np.full(n, np.nan)
    order = np.argsort(t_s)
    ys = np.interp(grid, t_s[order], y[order], left=np.nan, right=np.nan)
    return grid, ys


def xcorr_lag_s(ref_t_s, ref_y, resp_t_s, resp_y,
                fs_hz=200.0, max_lag_s=0.40):
    """Lag L>=0 (s) such that resp(t) best matches ref(t-L): how long
    the RESPONSE (measured platform tilt) trails the COMMAND (cmd
    tilt). The measured lag is the *effective* command->motion latency
    — transport + feeder ZOH + leg slew + the mechanical rise
    (verified against synthetic signals with known delay + rise: a
    pure delay is recovered exactly; a first-order rise adds a constant
    offset equal to its group delay).

    Sign-agnostic (the platform tilt may be sign-flipped vs the cmd
    convention): scores |normalised correlation|, reports the sign in
    the returned correlation. Resamples both series to a common uniform
    grid so different topic rates (cmd ~55 Hz, IMU ~240 Hz) line up.

    Returns (lag_s | None, peak_corr_signed, n_overlap)."""
    if (ref_t_s is None or resp_t_s is None
            or len(ref_t_s) < 4 or len(resp_t_s) < 4):
        return None, 0.0, 0
    t0 = max(float(ref_t_s[0]), float(resp_t_s[0]))
    t1 = min(float(ref_t_s[-1]), float(resp_t_s[-1]))
    if t1 - t0 < 3.0 * max_lag_s:
        return None, 0.0, 0
    _, a = resample_uniform(ref_t_s, ref_y, fs_hz, t0, t1)    # command
    _, b = resample_uniform(resp_t_s, resp_y, fs_hz, t0, t1)  # response
    good = np.isfinite(a) & np.isfinite(b)
    if good.sum() < 8:
        return None, 0.0, 0
    a = a - np.nanmean(a[good])
    b = b - np.nanmean(b[good])
    a[~good] = 0.0
    b[~good] = 0.0
    if np.linalg.norm(a) == 0.0:
        return None, 0.0, 0
    max_lag = int(round(max_lag_s * fs_hz))
    best = (0, 0.0)
    for L in range(0, max_lag + 1):
        aa, bb = (a, b) if L == 0 else (a[:-L], b[L:])
        nb = np.linalg.norm(bb)
        if nb == 0.0:
            continue
        c = float(np.dot(aa, bb) / (np.linalg.norm(aa) * nb))
        if abs(c) > abs(best[1]):
            best = (L, c)
    return best[0] / fs_hz, best[1], int(good.sum())


def actuation_latency(bt_t_ns, cmd_pitch, cmd_roll,
                      imu_t_ns, imu_rp, rpy_t_ns, rpy_data,
                      min_corr=0.3):
    """Command->motion latency by cross-correlating commanded tilt
    against the measured platform tilt. Prefers the high-rate platform
    IMU; falls back to /platform_rpy (~17 Hz) if the IMU orientation
    isn't populated (a zero quaternion -> all-zero rp -> useless).

    `*_t_ns` are int64 ROS bag receive-times (nanoseconds). `imu_rp`
    is an (N,2) [roll,pitch] array (deg); `rpy_data` is (N,3)
    [roll,pitch,yaw]. Returns a dict (source, per-axis lag + corr,
    headline `actuation_ms`) or None when there's no usable signal.
    The headline is the axis with the stronger correlation, so a quiet
    axis can't dominate the number."""
    if bt_t_ns is None or len(bt_t_ns) < 8:
        return None
    source = act_t = act_roll = act_pitch = None
    if (imu_t_ns is not None and len(imu_t_ns) > 8 and imu_rp is not None
            and getattr(imu_rp, 'size', 0) and np.any(np.isfinite(imu_rp))
            and float(np.nanstd(imu_rp)) > 1e-4):
        source = 'platform_imu'
        act_t = imu_t_ns
        act_roll, act_pitch = imu_rp[:, 0], imu_rp[:, 1]
    elif (rpy_t_ns is not None and len(rpy_t_ns) > 8
          and rpy_data is not None and getattr(rpy_data, 'size', 0)):
        source = 'platform_rpy'
        act_t = rpy_t_ns
        act_roll, act_pitch = rpy_data[:, 0], rpy_data[:, 1]
    else:
        return None
    # Common origin (ns->s) to preserve float64 precision on the lag.
    org = float(min(int(bt_t_ns[0]), int(act_t[0])))
    bt_s = (np.asarray(bt_t_ns, dtype=np.float64) - org) * 1e-9
    act_s = (np.asarray(act_t, dtype=np.float64) - org) * 1e-9
    p_lag, p_corr, p_n = xcorr_lag_s(bt_s, cmd_pitch, act_s, act_pitch)
    r_lag, r_corr, r_n = xcorr_lag_s(bt_s, cmd_roll, act_s, act_roll)
    out = {
        'source': source,
        'pitch_lag_ms': None if p_lag is None else round(p_lag * 1e3, 1),
        'pitch_corr': round(p_corr, 3),
        'roll_lag_ms': None if r_lag is None else round(r_lag * 1e3, 1),
        'roll_corr': round(r_corr, 3),
        'n_overlap': int(max(p_n, r_n)),
    }
    cands = [(abs(p_corr), p_lag), (abs(r_corr), r_lag)]
    cands = [(c, l) for c, l in cands if l is not None and c > min_corr]
    out['actuation_ms'] = (round(max(cands)[1] * 1e3, 1)
                           if cands else None)
    return out
