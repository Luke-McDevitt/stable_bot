"""Unit tests for stewart_bringup._latency — the command->motion
actuation-latency math used by the demo digest. Pure NumPy, no ROS,
no hardware.

If these pass, the cross-correlation recovers a known delay exactly,
a mechanical rise shows up only as a constant offset (so the reported
number is the true effective command->motion delay), the IMU-vs-rpy
source selection is correct, and the quaternion->tilt conversion is
right. That is what the digest's `latency_breakdown` relies on.
"""
import numpy as np
import pytest

from stewart_bringup._latency import (
    actuation_latency,
    dominant_period_s,
    quat_to_roll_pitch_deg,
    resample_uniform,
    step_response_metrics,
    step_train_metrics,
    xcorr_lag_s,
)


# --- synthetic signal generator -------------------------------------------

def _make_signals(true_lag_s, sign, fs=240.0, dur=20.0, noise=0.05,
                  cmd_rate=55.0, resp_rate=240.0, tau_s=0.02, seed=0,
                  freq=2.0):
    """Command = `freq` Hz square wave (the saturated orbital cmd tilt).
    Response = command delayed by true_lag_s, first-order-smoothed
    (mechanical rise, time-constant tau_s), sign-flipped by `sign`,
    decimated to resp_rate, + noise. Command decimated to cmd_rate.
    Returns (cmd_t_s, cmd_y, resp_t_s, resp_y) — emulating two ROS
    topics at different rates."""
    rng = np.random.default_rng(seed)
    n = int(dur * fs)
    t = np.arange(n) / fs
    cmd = 1.2 * np.sign(np.sin(2 * np.pi * freq * t))
    cmd = cmd + 0.1 * rng.standard_normal(n)
    lag_n = int(round(true_lag_s * fs))
    delayed = np.concatenate([np.zeros(lag_n), cmd])[:n]
    resp = np.zeros(n)
    alpha = 1.0 - np.exp(-1.0 / (tau_s * fs))
    for i in range(1, n):
        resp[i] = resp[i - 1] + alpha * (delayed[i] - resp[i - 1])
    resp = sign * resp + noise * rng.standard_normal(n)
    ci = np.unique((np.arange(0, dur, 1.0 / cmd_rate) * fs).astype(int))
    ci = ci[ci < n]
    ri = np.unique((np.arange(0, dur, 1.0 / resp_rate) * fs).astype(int))
    ri = ri[ri < n]
    return t[ci], cmd[ci], t[ri], resp[ri]


def _ns(t_s, base=1_700_000_000_000_000_000):
    """Seconds -> int64 ROS-bag-style nanosecond timestamps."""
    return (base + np.asarray(t_s) * 1e9).astype(np.int64)


# --- (A) pure transport delay must be recovered EXACTLY -------------------

@pytest.mark.parametrize("true_lag", [0.040, 0.080, 0.120, 0.200])
@pytest.mark.parametrize("sign", [+1.0, -1.0])
def test_pure_delay_recovered_exactly(true_lag, sign):
    ct, cy, rt, ry = _make_signals(true_lag, sign, tau_s=1e-4,
                                   seed=int(true_lag * 1e3) + int(sign))
    lag, corr, n = xcorr_lag_s(ct, cy, rt, ry, fs_hz=200.0)
    assert lag is not None
    assert abs(lag - true_lag) * 1000 <= 5.0       # within one 200 Hz bin
    assert abs(corr) > 0.9                          # strong, sign either way
    assert n > 100


# --- (B) a mechanical rise adds only a CONSTANT offset --------------------

def test_mechanical_rise_is_constant_offset():
    offsets = []
    for true_lag in (0.040, 0.080, 0.120, 0.200):
        ct, cy, rt, ry = _make_signals(true_lag, +1.0, tau_s=0.020,
                                       seed=int(true_lag * 1e3))
        lag, corr, _ = xcorr_lag_s(ct, cy, rt, ry, fs_hz=200.0)
        assert lag is not None
        offsets.append((lag - true_lag) * 1000.0)
    # The rise contributes the same group delay regardless of transport
    # delay, so the offset must be ~constant (not lag-dependent).
    assert max(offsets) - min(offsets) <= 6.0
    assert all(o > 0 for o in offsets)              # rise only adds delay


# --- (C) the actuation_latency wrapper ------------------------------------

def test_actuation_prefers_imu_and_recovers_lag():
    true_lag = 0.090
    ct, cy, rt, ry = _make_signals(true_lag, -1.0, tau_s=0.020, seed=7)
    _, cy_roll, _, ry_roll = _make_signals(true_lag, +1.0, tau_s=0.020,
                                           seed=8)
    imu_rp = np.stack([ry_roll, ry], axis=1)        # [roll, pitch]
    out = actuation_latency(_ns(ct), cy, cy_roll, _ns(rt), imu_rp,
                            None, None)
    assert out is not None
    assert out['source'] == 'platform_imu'
    # 90 ms transport + ~15 ms rise group delay.
    assert out['actuation_ms'] is not None
    assert abs(out['actuation_ms'] - 105.0) < 15.0


def test_actuation_falls_back_to_rpy_when_imu_degenerate():
    true_lag = 0.090
    ct, cy, rt, ry = _make_signals(true_lag, -1.0, tau_s=0.020, seed=7)
    _, cy_roll, _, ry_roll = _make_signals(true_lag, +1.0, tau_s=0.020,
                                           seed=8)
    imu_zero = np.zeros((rt.size, 2))               # unpopulated orientation
    rpy_data = np.stack([ry_roll, ry, np.zeros_like(ry)], axis=1)
    out = actuation_latency(_ns(ct), cy, cy_roll, _ns(rt), imu_zero,
                            _ns(rt), rpy_data)
    assert out is not None
    assert out['source'] == 'platform_rpy'
    assert out['actuation_ms'] is not None


def test_actuation_returns_none_without_tilt_source():
    ct, cy, _, _ = _make_signals(0.09, +1.0, seed=1)
    assert actuation_latency(_ns(ct), cy, cy, None, None, None, None) is None


def test_actuation_returns_none_with_too_few_commands():
    # < 8 command samples -> not enough to trust a lag estimate.
    bt = _ns(np.linspace(0, 0.1, 4))
    cy = np.array([0.0, 1.0, 0.0, 1.0])
    imu = np.stack([np.zeros(50), np.ones(50)], axis=1)
    assert actuation_latency(bt, cy, cy, _ns(np.linspace(0, 0.1, 50)),
                             imu, None, None) is None


# --- (D) quaternion -> roll/pitch -----------------------------------------

def test_quat_to_roll_pitch_known_angles():
    import math
    r, p, y = math.radians(10), math.radians(5), math.radians(0)
    cy_, sy_ = math.cos(y / 2), math.sin(y / 2)
    cp_, sp_ = math.cos(p / 2), math.sin(p / 2)
    cr_, sr_ = math.cos(r / 2), math.sin(r / 2)
    qw = cr_ * cp_ * cy_ + sr_ * sp_ * sy_
    qx = sr_ * cp_ * cy_ - cr_ * sp_ * sy_
    qy = cr_ * sp_ * cy_ + sr_ * cp_ * sy_
    qz = cr_ * cp_ * sy_ - sr_ * sp_ * cy_
    roll, pitch = quat_to_roll_pitch_deg([[qx, qy, qz, qw]])
    assert abs(float(roll[0]) - 10.0) < 0.05
    assert abs(float(pitch[0]) - 5.0) < 0.05


def test_quat_identity_is_level():
    roll, pitch = quat_to_roll_pitch_deg([[0.0, 0.0, 0.0, 1.0]])
    assert abs(float(roll[0])) < 1e-6
    assert abs(float(pitch[0])) < 1e-6


# --- (D2) periodicity detection + ambiguity flag --------------------------

@pytest.mark.parametrize("freq", [2.0, 3.0, 3.3])
def test_dominant_period_recovers_known_period(freq):
    ct, cy, _, _ = _make_signals(0.08, +1.0, freq=freq, seed=3)
    per = dominant_period_s(ct, cy, fs_hz=200.0)
    assert per is not None
    assert abs(per - 1.0 / freq) < 0.02      # within ~one bin


def test_dominant_period_none_for_aperiodic():
    # A single step (the kind of aperiodic excitation a tap test gives)
    # has no dominant period -> None, so it would NOT be flagged ambiguous.
    t = np.linspace(0, 10, 2000)
    y = np.where(t > 5.0, 1.0, 0.0)
    assert dominant_period_s(t, y, fs_hz=200.0) is None


def test_actuation_flags_ambiguous_on_fast_orbit():
    # 3.3 Hz orbit (period ~0.30 s) with a 0.20 s lag: 0.20 > period/2,
    # so the absolute lag is period-aliased -> must be flagged ambiguous,
    # mirroring the real demo2 orbital runs.
    ct, cy, rt, ry = _make_signals(0.185, -1.0, tau_s=0.02, freq=3.3, seed=5)
    _, cy_roll, _, ry_roll = _make_signals(0.185, +1.0, tau_s=0.02,
                                           freq=3.3, seed=6)
    imu_rp = np.stack([ry_roll, ry], axis=1)
    out = actuation_latency(_ns(ct), cy, cy_roll, _ns(rt), imu_rp, None, None)
    assert out is not None
    assert out['cmd_period_s'] is not None
    assert abs(out['cmd_period_s'] - 1.0 / 3.3) < 0.03
    assert out['ambiguous'] is True
    assert 'lag_candidates_s' in out and len(out['lag_candidates_s']) >= 2


def test_actuation_not_ambiguous_on_slow_orbit():
    # 1 Hz orbit (period 1.0 s) with a ~0.105 s effective lag: no half-
    # period alias (±0.5 s) lands in the [0, 0.4 s] search window, so the
    # lag is unambiguous even though the run is periodic.
    ct, cy, rt, ry = _make_signals(0.09, -1.0, tau_s=0.02, freq=1.0, seed=7)
    _, cy_roll, _, ry_roll = _make_signals(0.09, +1.0, tau_s=0.02,
                                           freq=1.0, seed=8)
    imu_rp = np.stack([ry_roll, ry], axis=1)
    out = actuation_latency(_ns(ct), cy, cy_roll, _ns(rt), imu_rp, None, None)
    assert out is not None
    assert out['ambiguous'] is False


# --- (D3) step_response_metrics — the Latency Bench tilt-step analysis ----

def _make_step(dead_s=0.04, step_deg=2.0, ramp_s=0.15, tau_s=0.03,
               dur=4.0, t_step=1.0, fs=400.0, cmd_rate=50.0,
               resp_rate=400.0, noise=0.01, seed=0):
    """A ramped tilt-step command + a delayed, first-order-smoothed
    response (emulates the bench: commanded tilt vs IMU-measured tilt at
    different topic rates). Returns (cmd_t, cmd_y, resp_t, resp_y)."""
    rng = np.random.default_rng(seed)
    n = int(dur * fs)
    t = np.arange(n) / fs
    cmd = np.clip((t - t_step) / ramp_s, 0.0, 1.0) * step_deg
    dead_n = int(round(dead_s * fs))
    delayed = np.concatenate([np.zeros(dead_n), cmd])[:n]
    resp = np.zeros(n)
    alpha = 1.0 - np.exp(-1.0 / (tau_s * fs))
    for i in range(1, n):
        resp[i] = resp[i - 1] + alpha * (delayed[i] - resp[i - 1])
    resp = resp + noise * rng.standard_normal(n)
    ci = np.unique((np.arange(0, dur, 1.0 / cmd_rate) * fs).astype(int))
    ci = ci[ci < n]
    ri = np.unique((np.arange(0, dur, 1.0 / resp_rate) * fs).astype(int))
    ri = ri[ri < n]
    return t[ci], cmd[ci], t[ri], resp[ri]


def test_step_metrics_gain_and_keys():
    ct, cy, rt, ry = _make_step(dead_s=0.04, step_deg=2.0, seed=1)
    m = step_response_metrics(ct, cy, rt, ry, fs_hz=400.0)
    assert m is not None
    assert abs(m['gain'] - 1.0) < 0.1            # IMU tracks the command
    assert abs(m['step_cmd'] - 2.0) < 0.1
    for k in ('dead_time_ms', 'xcorr_lag_ms', 'rise_10_90_ms',
              'settle_ms', 'overshoot_pct'):
        assert k in m
    assert m['rise_10_90_ms'] is not None and m['rise_10_90_ms'] > 0


def test_step_dead_time_small_for_fast_response():
    # ~40 ms transport with a fast (8 ms) response + short ramp: the
    # onset→onset dead time should sit just above the true 40 ms.
    ct, cy, rt, ry = _make_step(dead_s=0.04, tau_s=0.008, ramp_s=0.04, seed=5)
    m = step_response_metrics(ct, cy, rt, ry, fs_hz=400.0)
    assert m is not None
    assert 30.0 <= m['dead_time_ms'] <= 65.0


def test_step_dead_time_tracks_true_delay():
    # Measured dead = true transport + a ~constant rise-to-10% offset,
    # so across true delays the offset stays roughly constant (and the
    # measurement is monotonic in the true delay).
    offs, meas = [], []
    for dead in (0.02, 0.05, 0.10):
        ct, cy, rt, ry = _make_step(dead_s=dead, tau_s=0.008, ramp_s=0.04,
                                    seed=int(dead * 1000))
        m = step_response_metrics(ct, cy, rt, ry, fs_hz=400.0)
        assert m is not None and m['dead_time_ms'] is not None
        offs.append(m['dead_time_ms'] - dead * 1000.0)
        meas.append(m['dead_time_ms'])
    assert max(offs) - min(offs) < 12.0          # ~constant offset
    assert meas[0] < meas[1] < meas[2]           # monotonic in true delay


def test_step_metrics_none_without_step():
    t = np.linspace(0, 3, 300)
    cy = np.zeros_like(t)                          # no commanded step
    rt = np.linspace(0, 3, 1200)
    ry = 0.01 * np.random.default_rng(0).standard_normal(1200)
    assert step_response_metrics(t, cy, rt, ry) is None


def test_step_metrics_handles_negative_step():
    # A downward tilt step must work too (sign-correct gain ~1).
    ct, cy, rt, ry = _make_step(dead_s=0.05, step_deg=-2.0, tau_s=0.01,
                                ramp_s=0.05, seed=3)
    m = step_response_metrics(ct, cy, rt, ry, fs_hz=400.0)
    assert m is not None
    assert abs(m['gain'] - 1.0) < 0.15
    assert m['dead_time_ms'] is not None and m['dead_time_ms'] > 0


# --- (D4) step_train_metrics — the MULTI-REP bench analysis ---------------

def _make_step_train(reps=3, dead_s=0.04, step_deg=2.0, hold_s=0.8,
                     settle_s=0.8, tau_s=0.03, fs=400.0, cmd_rate=50.0,
                     resp_rate=240.0, noise=0.01, seed=0):
    """A TRAIN of tilt steps: settle, then reps of (hold up, settle down) —
    exactly what the Latency Bench commands. Response = delayed + smoothed.
    cmd decimated to cmd_rate (the bench's diag), resp to resp_rate (IMU)."""
    rng = np.random.default_rng(seed)
    segs = [(settle_s, 0.0)]
    for _ in range(reps):
        segs += [(hold_s, step_deg), (settle_s, 0.0)]
    total = sum(s for s, _ in segs)
    n = int(total * fs)
    t = np.arange(n) / fs
    cmd = np.zeros(n)
    acc = 0.0
    for dur, val in segs:
        cmd[(t >= acc) & (t < acc + dur)] = val
        acc += dur
    dead_n = int(round(dead_s * fs))
    delayed = np.concatenate([np.zeros(dead_n), cmd])[:n]
    resp = np.zeros(n)
    alpha = 1.0 - np.exp(-1.0 / (tau_s * fs))
    for i in range(1, n):
        resp[i] = resp[i - 1] + alpha * (delayed[i] - resp[i - 1])
    resp = resp + noise * rng.standard_normal(n)
    ci = np.unique((np.arange(0, total, 1.0 / cmd_rate) * fs).astype(int))
    ci = ci[ci < n]
    ri = np.unique((np.arange(0, total, 1.0 / resp_rate) * fs).astype(int))
    ri = ri[ri < n]
    return t[ci], cmd[ci], t[ri], resp[ri]


def test_step_train_detects_all_reps_and_gain():
    ct, cy, rt, ry = _make_step_train(reps=3, dead_s=0.04, cmd_rate=100.0,
                                      tau_s=0.008, seed=1)
    m = step_train_metrics(ct, cy, rt, ry)
    assert m is not None
    assert m['n_steps'] == 3                       # each rep found
    assert abs(m['gain'] - 1.0) < 0.12
    assert m['dead_time_ms'] is not None and 25 <= m['dead_time_ms'] <= 75
    assert m['rise_10_90_ms'] is not None and m['rise_10_90_ms'] > 0


def test_step_train_reports_real_undertilt():
    # The bench symptom: platform reaches only ~40% of commanded. The
    # per-rep gain must REPORT ~0.4, not mask it as ~1 or garbage.
    ct, cy, rt, ry = _make_step_train(reps=3, step_deg=2.0, cmd_rate=100.0,
                                      tau_s=0.01, seed=4)
    m = step_train_metrics(ct, cy, rt, ry * 0.4)
    assert m is not None
    assert abs(m['gain'] - 0.4) < 0.1


def test_step_train_sparse_cmd_still_finds_reps():
    # 10 Hz cmd (the bench's observed diag rate) — the reps + gain are
    # still recovered from the edges (dead time is coarse but present).
    ct, cy, rt, ry = _make_step_train(reps=3, dead_s=0.05, cmd_rate=10.0,
                                      tau_s=0.01, seed=2)
    m = step_train_metrics(ct, cy, rt, ry)
    assert m is not None and m['n_steps'] == 3
    assert abs(m['gain'] - 1.0) < 0.15


def test_step_train_none_without_edges():
    t = np.linspace(0, 5, 50)
    cy = np.zeros_like(t)                           # flat — no rising edge
    rt = np.linspace(0, 5, 1000)
    ry = np.zeros_like(rt)
    assert step_train_metrics(t, cy, rt, ry) is None


# --- (E) resample_uniform NaN-pads outside the source span ----------------

def test_resample_uniform_nan_outside_span():
    t = np.array([1.0, 1.5, 2.0])
    y = np.array([0.0, 1.0, 2.0])
    grid, ys = resample_uniform(t, y, fs_hz=10.0, t0_s=0.0, t1_s=3.0)
    assert np.isnan(ys[0])           # before span
    assert np.isnan(ys[-1])          # after span
    mid = np.isfinite(ys)
    assert mid.sum() > 0
    assert np.allclose(ys[mid].min(), 0.0, atol=0.2)
