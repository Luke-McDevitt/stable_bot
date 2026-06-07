"""Unit tests for stewart_bringup._kf_noise — KF measurement-noise (R) and
outlier estimation from bag series. Pure math, no ROS.
"""
import random

from stewart_bringup._kf_noise import (
    still_windows, measurement_noise, outlier_rate, _robust_std)


def test_robust_std_ignores_outliers():
    rng = random.Random(1)
    rs = [rng.gauss(0.0, 2.0) for _ in range(2000)] + [500.0, -500.0]
    assert abs(_robust_std(rs) - 2.0) < 0.4        # MAD shrugs off the spikes


def test_still_windows_finds_the_still_stretch():
    ts, px, py, t = [], [], [], 0.0
    while t <= 4.0:
        ts.append(t)
        px.append(50.0 if t < 2.0 else 50.0 + 200.0 * (t - 2.0))
        py.append(20.0); t += 0.02
    wins = still_windows(ts, px, py, range_mm=12.0, min_dur_s=0.8)
    assert wins and wins[0][0] < 0.1 and wins[0][1] >= 1.8


def test_measurement_noise_recovers_R():
    rng = random.Random(3)
    ts, mx, my, sx, sy, t = [], [], [], [], [], 0.0
    while t <= 3.0:
        ts.append(t)
        mx.append(50.0 + rng.gauss(0.0, 2.5))
        my.append(20.0 + rng.gauss(0.0, 2.5))
        sx.append(50.0); sy.append(20.0)           # posterior: still
        t += 0.02
    wins = still_windows(ts, sx, sy, range_mm=12.0)
    R = measurement_noise(ts, mx, my, wins)
    assert abs(R['R_mm'] - 2.5) < 0.5
    assert R['n'] > 100


def test_outlier_rate_flags_impossible_jumps():
    ts = [i * 0.02 for i in range(100)]
    mx = [50.0] * 100
    my = [20.0] * 100
    mx[50] = 150.0                                  # 100 mm in one frame = 5 m/s
    frac, n_out, n = outlier_rate(ts, mx, my, max_speed_mm_s=1500.0)
    assert n_out >= 1 and frac > 0
