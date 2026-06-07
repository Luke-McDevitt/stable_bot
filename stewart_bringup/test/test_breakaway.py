"""Unit tests for stewart_bringup._breakaway — the offline breakaway-angle
extraction the breakaway-bag digest runs. Pure math, no ROS / no bag.

Pins the pieces the method relies on: interpolation, the IMU quaternion→tilt
conversion, rest/onset detection (incl. wobble rejection), and the full
latency-corrected θ_s recovery from synthetic ball + IMU series.
"""
import math

from stewart_bringup._breakaway import (
    interp_at, quat_to_roll_pitch_deg, find_rest_position,
    find_motion_onset, analyze_breakaway,
)


def test_interp_at_basic_and_clamp():
    ts = [0.0, 1.0, 2.0]
    vs = [0.0, 10.0, 20.0]
    assert interp_at(ts, vs, 0.5) == 5.0
    assert interp_at(ts, vs, 1.5) == 15.0
    assert interp_at(ts, vs, -1.0) == 0.0       # clamp low
    assert interp_at(ts, vs, 9.0) == 20.0       # clamp high
    assert interp_at([], [], 1.0) is None


def test_quat_identity_is_level():
    r, p = quat_to_roll_pitch_deg(1.0, 0.0, 0.0, 0.0)
    assert math.isclose(r, 0.0, abs_tol=1e-9)
    assert math.isclose(p, 0.0, abs_tol=1e-9)


def test_quat_pure_pitch():
    # 10° pitch about y: q = (cos5°, 0, sin5°, 0)
    w, y = math.cos(math.radians(5)), math.sin(math.radians(5))
    r, p = quat_to_roll_pitch_deg(w, 0.0, y, 0.0)
    assert math.isclose(r, 0.0, abs_tol=1e-6)
    assert math.isclose(p, 10.0, abs_tol=1e-4)


def test_find_rest_position_is_baseline_median():
    ts = [i * 0.02 for i in range(100)]            # 2 s
    px = [50.0] * 50 + [50.0 + i for i in range(50)]   # moves after 1 s
    py = [20.0] * 100
    x0, y0 = find_rest_position(ts, px, py, baseline_s=0.8)
    assert math.isclose(x0, 50.0, abs_tol=0.5)     # baseline only → still
    assert math.isclose(y0, 20.0, abs_tol=0.5)


def test_find_motion_onset_rejects_wobble():
    # ball jiggles ±3 mm but never travels min_travel → no onset
    ts = [i * 0.02 for i in range(200)]
    px = [50.0 + 3.0 * math.sin(i) for i in range(200)]
    py = [20.0] * 200
    assert find_motion_onset(ts, px, py, 50.0, 20.0,
                             min_travel_mm=25.0) is None


def test_find_motion_onset_finds_real_motion():
    ts = [i * 0.02 for i in range(200)]            # 4 s
    # still until t=2 s (index 100), then rolls
    px = [50.0] * 100 + [50.0 + 400.0 * (i - 100) * 0.02 for i in range(100)]
    py = [20.0] * 200
    onset_t, confirm_t, peak = find_motion_onset(
        ts, px, py, 50.0, 20.0, min_travel_mm=25.0)
    assert 1.9 <= onset_t <= 2.1                    # onset near the 2 s mark
    assert confirm_t >= onset_t
    assert peak > 25.0


def test_onset_ignores_early_stall_with_speed():
    # Ball nudges to ~8 mm at t≈1 s (settling against the edge ring), STALLS
    # there for seconds, then really rolls past min_travel at t≈5 s. KF speed
    # tells the plateau (≈0) from the final roll, so the onset must be the late
    # roll — this is the edge-run bug that read the early nudge's low tilt.
    ts = [i * 0.02 for i in range(400)]            # 8 s @ 50 Hz
    px, py, spd = [], [], []
    for t in ts:
        if t < 1.0:
            x, v = 50.0, 0.0                       # at rest
        elif t < 1.3:
            x, v = 50.0 + 27.0 * (t - 1.0), 27.0   # quick nudge to ~8 mm
        elif t < 5.0:
            x, v = 58.0, 0.0                       # STALL / plateau
        else:
            x, v = 58.0 + 300.0 * (t - 5.0), 300.0  # real roll
        px.append(x); py.append(20.0); spd.append(v)
    onset_t, _, peak = find_motion_onset(
        ts, px, py, 50.0, 20.0, min_travel_mm=25.0, speed=spd)
    assert onset_t >= 4.8 and peak > 25.0          # the late roll, not the nudge
    # without speed the position-band walk-back is fooled back to the nudge
    onset_nospeed, _, _ = find_motion_onset(
        ts, px, py, 50.0, 20.0, min_travel_mm=25.0)
    assert onset_nospeed < 2.0                     # the bug the speed path fixes


def _synth_run(latency=0.15, true_theta=2.0, ramp_dps=0.5):
    """Synthesise a breakaway: pitch ramps at ramp_dps starting at t=1 s; the
    ball really breaks when pitch hits true_theta (real time), but is SEEN
    `latency` s late. Returns the analyze_breakaway kwargs."""
    t_break_real = 1.0 + true_theta / ramp_dps
    t_move_seen = t_break_real + latency
    ball_ts, px, py, lat = [], [], [], []
    t = 0.0
    while t <= t_break_real + latency + 1.0:
        ball_ts.append(t)
        px.append(50.0 if t < t_move_seen else 50.0 + 400.0 * (t - t_move_seen))
        py.append(20.0)
        lat.append(latency)
        t += 0.02                                   # 50 Hz
    imu_ts, imu_roll, imu_pitch = [], [], []
    t = 0.0
    while t <= t_break_real + latency + 1.0:
        imu_ts.append(t)
        imu_roll.append(0.0)
        imu_pitch.append(0.0 if t < 1.0 else ramp_dps * (t - 1.0))
        t += 0.01                                   # 100 Hz
    return dict(ball_ts=ball_ts, px=px, py=py, ball_lat=lat,
                imu_ts=imu_ts, imu_roll=imu_roll, imu_pitch=imu_pitch)


def test_analyze_recovers_true_angle_with_latency():
    r = analyze_breakaway(**_synth_run(latency=0.15, true_theta=2.0))
    assert r['ok']
    assert abs(r['theta_s_deg'] - 2.0) < 0.1        # recovers the true 2.0°
    assert r['peak_travel_mm'] > 25.0
    assert math.isclose(r['latency_s'], 0.15, abs_tol=1e-6)


def test_latency_correction_matters():
    # without the shift the angle reads high (tilt kept ramping during the lag)
    kw = _synth_run(latency=0.2, true_theta=2.0, ramp_dps=1.0)
    corrected = analyze_breakaway(**kw)['theta_s_deg']
    kw_nolat = dict(kw, ball_lat=[0.0] * len(kw['ball_lat']))
    uncorrected = analyze_breakaway(**kw_nolat)['theta_s_deg']
    assert uncorrected - corrected > 0.15           # ≈ ramp_dps × latency
    assert abs(corrected - 2.0) < 0.1


def test_analyze_rejects_pure_wobble():
    kw = _synth_run()
    kw['px'] = [50.0 + 3.0 * math.sin(i) for i in range(len(kw['ball_ts']))]
    r = analyze_breakaway(**kw)
    assert not r['ok']
