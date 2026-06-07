"""_breakaway.py — offline breakaway-angle extraction from a recorded ramp.

The live daemon ramps the plate and records a bag; THIS module reads that
bag's series and computes θ_s precisely offline, fixing the two things the
live single-number readout cannot:

  • latency — the ball is seen ~150 ms late, so the tilt at the moment we
    *detect* motion over-reads the tilt at the moment the ball *actually*
    moved. We find the displacement onset and shift it back by the bag's own
    photon→state latency (carried per-sample on /ball_state).
  • command vs reality — we read the ACTUAL plate tilt from the IMU at that
    corrected instant, referenced to the settled baseline, instead of the
    commanded ramp. That removes the leveling-zero offset and the command
    tracking error (two of the big position-dependence confounds), and using
    the settled baseline as zero cancels the plate's local resting slope.

Pure math (no ROS / no numpy) so digest_breakaway_bag.py is a thin shell and
this is unit-tested in test/test_breakaway.py.
"""
from __future__ import annotations

import math


def interp_at(ts, vs, tq):
    """Linear interpolation of vs(ts) at tq; clamps past the ends. `ts` must be
    ascending. None for empty input."""
    n = len(ts)
    if n == 0:
        return None
    if tq <= ts[0]:
        return vs[0]
    if tq >= ts[-1]:
        return vs[-1]
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= tq:
            lo = mid
        else:
            hi = mid
    span = ts[hi] - ts[lo]
    if span <= 0:
        return vs[lo]
    f = (tq - ts[lo]) / span
    return vs[lo] + f * (vs[hi] - vs[lo])


def quat_to_roll_pitch_deg(w, x, y, z):
    """IMU orientation quaternion → (roll, pitch) in degrees (aerospace ZYX)."""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    return math.degrees(roll), math.degrees(pitch)


def _median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return 0.0
    m = n // 2
    return s[m] if n % 2 else 0.5 * (s[m - 1] + s[m])


def find_rest_position(ts, px, py, baseline_s=0.8):
    """Median ball (x, y) over the first `baseline_s` seconds (the still
    segment) — the rest reference the ramp starts from."""
    if not ts:
        return None
    t0 = ts[0]
    xs = [px[i] for i in range(len(ts)) if ts[i] - t0 <= baseline_s]
    ys = [py[i] for i in range(len(ts)) if ts[i] - t0 <= baseline_s]
    if len(xs) < 3:
        k = max(3, len(px) // 10)
        xs, ys = px[:k], py[:k]
    return _median(xs), _median(ys)


def find_motion_onset(ts, px, py, px0, py0, min_travel_mm=25.0, noise_mm=4.0,
                      speed=None, v_still_mm_s=15.0):
    """Find when the ball STARTED the motion that carried it `min_travel_mm`
    from rest. Confirm at the first sample past min_travel, then walk back to
    the last time the ball was at REST.

    With `speed` (the KF |v| series) supplied, "rest" means |v| < v_still — so
    an early nudge that STALLS into a plateau (displaced from rest but no longer
    moving, e.g. a ball settling against the edge ring) is treated as rest and
    is NOT mistaken for the breakaway onset; the walk-back stops at the start of
    the final, sustained roll. Without speed it falls back to the position noise
    band (only valid when motion is monotonic). Returns
    (onset_t, confirm_t, peak_disp_mm) or None (never travelled far enough)."""
    n = len(ts)
    disp = [math.hypot(px[i] - px0, py[i] - py0) for i in range(n)]
    confirm = next((i for i in range(n) if disp[i] >= min_travel_mm), None)
    if confirm is None:
        return None
    j = confirm
    if speed is not None and len(speed) == n:
        while j > 0 and speed[j] >= v_still_mm_s:
            j -= 1
    else:
        while j > 0 and disp[j] > noise_mm:
            j -= 1
    return ts[j], ts[confirm], max(disp)


def _baseline_median(ts, vs, baseline_s):
    if not ts:
        return 0.0
    t0 = ts[0]
    seg = [vs[i] for i in range(len(ts)) if ts[i] - t0 <= baseline_s]
    return _median(seg if seg else vs[:5])


def analyze_breakaway(ball_ts, px, py, ball_lat,
                      imu_ts, imu_roll, imu_pitch,
                      ball_speed=None,
                      min_travel_mm=25.0, noise_mm=4.0, baseline_s=0.8):
    """Full extraction → result dict.

    The breakaway angle is the magnitude of the IMU tilt CHANGE from the
    settled baseline to the latency-corrected motion onset:
        θ_s = |(roll_onset−roll_base, pitch_onset−pitch_base)|.
    Magnitude (not a single axis) so it's robust to the IMU frame's mapping to
    the platform's pitch/roll axes.

    `ball_lat`: per-sample photon→state latency (s) aligned with `ball_ts`.
    """
    if len(ball_ts) < 5 or len(imu_ts) < 5:
        return {'ok': False, 'reason': 'not enough ball/IMU samples'}
    rest = find_rest_position(ball_ts, px, py, baseline_s)
    px0, py0 = rest
    onset = find_motion_onset(ball_ts, px, py, px0, py0,
                              min_travel_mm, noise_mm, speed=ball_speed)
    if onset is None:
        return {'ok': False,
                'reason': f'ball never travelled {min_travel_mm:.0f} mm'}
    onset_t, confirm_t, peak = onset
    # Latency-correct the onset (the ball is seen late) before reading the IMU.
    lat = interp_at(ball_ts, ball_lat, onset_t) if ball_lat else 0.0
    lat = lat if (lat and lat > 0) else 0.0
    t_true = onset_t - lat
    base_r = _baseline_median(imu_ts, imu_roll, baseline_s)
    base_p = _baseline_median(imu_ts, imu_pitch, baseline_s)
    d_roll = interp_at(imu_ts, imu_roll, t_true) - base_r
    d_pitch = interp_at(imu_ts, imu_pitch, t_true) - base_p
    theta_s = math.hypot(d_roll, d_pitch)
    return {
        'ok': True,
        'theta_s_deg': round(theta_s, 3),
        'd_roll_deg': round(d_roll, 3),
        'd_pitch_deg': round(d_pitch, 3),
        'onset_t': onset_t,
        'onset_t_corrected': t_true,
        'latency_s': round(lat, 4),
        'peak_travel_mm': round(peak, 1),
        'confirm_t': confirm_t,
        'rest_xy_mm': [round(px0, 1), round(py0, 1)],
    }
