"""Unit tests for stewart_bringup._ball_model — the gray-box ball-on-plate
forward model the Phase-1 fitter identifies and the predictor integrates.
Pure math, no ROS / no hardware.

Pins: Tier-1 drive (α·g·sinθ), the const-velocity reduction (level +
frictionless), the stiction breakaway, rolling-resistance sign, that a
fitted model actually predicts the tilt-driven acceleration const-velocity
misses, and the artifact round-trip.
"""
import math

from stewart_bringup._ball_model import (
    BallParams, BallForwardModel, ball_accel, simulate, G_MM_S2, SOLID_ALPHA,
    fit_measurement_noise, fit_rolling_resistance, breakaway_a_from_theta,
)


def test_tier1_drive_is_alpha_g_sin():
    # moving ball (above v_eps) so we exercise the drive, not the stiction gate
    ax, ay = ball_accel(100.0, 0.0, 0.1, 0.0, BallParams())
    assert math.isclose(ax, SOLID_ALPHA * G_MM_S2 * math.sin(0.1), rel_tol=1e-9)
    assert math.isclose(ay, 0.0, abs_tol=1e-9)


def test_level_frictionless_is_zero_accel():
    assert ball_accel(120.0, -40.0, 0.0, 0.0, BallParams()) == (0.0, 0.0)


def test_stiction_holds_then_breaks():
    p = BallParams(breakaway_a=500.0)
    # still ball, small tilt → drive < breakaway → stuck
    small = SOLID_ALPHA * G_MM_S2 * math.sin(0.05)      # ≈ 350
    assert small < 500.0
    assert ball_accel(0.0, 0.0, 0.05, 0.0, p) == (0.0, 0.0)
    # still ball, big tilt → drive > breakaway → moves
    ax, ay = ball_accel(0.0, 0.0, 0.12, 0.0, p)
    assert ax > 0.0


def test_rolling_resistance_opposes_motion():
    p = BallParams(c_roll=200.0)
    ax, ay = ball_accel(100.0, 0.0, 0.0, 0.0, p)        # level, moving +x
    assert math.isclose(ax, -200.0, rel_tol=1e-9)       # decel opposing +x
    assert math.isclose(ay, 0.0, abs_tol=1e-9)


def test_viscous_scales_with_velocity():
    p = BallParams(c_visc=2.0)
    ax, _ = ball_accel(150.0, 0.0, 0.0, 0.0, p)
    assert math.isclose(ax, -2.0 * 150.0, rel_tol=1e-9)


def test_integrate_level_equals_constant_velocity():
    # level + frictionless model == the legacy x + v·Td predictor (exact)
    m = BallForwardModel()
    px, py = m.integrate(10.0, 20.0, 5.0, -3.0, 0.25, tilt_history=None)
    assert math.isclose(px, 10.0 + 5.0 * 0.25, abs_tol=1e-6)
    assert math.isclose(py, 20.0 + -3.0 * 0.25, abs_tol=1e-6)


def test_integrate_with_tilt_beats_const_velocity():
    # the whole point of the model: from rest under a tilt it accelerates,
    # while constant-velocity would predict no motion.
    m = BallForwardModel()
    px, py = m.integrate(0.0, 0.0, 0.0, 0.0, 0.25, tilt_history=[(0.1, 0.0)])
    assert px > 5.0          # const-vel would give 0.0
    assert math.isclose(py, 0.0, abs_tol=1e-6)


def test_simulate_coasting_ball_decelerates():
    p = BallParams(c_roll=300.0)
    steps = 50
    px_t, py_t, vx_t, vy_t = simulate(
        0.0, 0.0, 800.0, 0.0, [0.0] * steps, [0.0] * steps, 0.01, p)
    assert vx_t[-1] < vx_t[0]                 # rolling resistance slowed it
    assert px_t[-1] > 0.0                     # but it still moved forward


def test_model_dict_round_trip():
    m = BallForwardModel(BallParams(alpha=0.7, c_roll=120.0,
                                    breakaway_a=350.0), dt=0.02)
    m2 = BallForwardModel.from_dict(m.to_dict())
    assert m2.dt == 0.02
    assert math.isclose(m2.params.c_roll, 120.0)
    assert math.isclose(m2.params.breakaway_a, 350.0)


# ----- identification (the fitter's math) ------------------------------------

def test_fit_measurement_noise_recovers_std():
    import random
    rng = random.Random(7)
    true_std = 1.5
    px = [50.0 + rng.gauss(0.0, true_std) for _ in range(3000)]
    py = [-20.0 + rng.gauss(0.0, true_std) for _ in range(3000)]
    r = fit_measurement_noise(px, py)
    assert abs(r['std_x_mm'] - true_std) < 0.15      # ~10% on 3000 samples
    assert abs(r['std_y_mm'] - true_std) < 0.15
    assert abs(r['R_mm'] - true_std) < 0.15
    assert r['n'] == 3000


def test_fit_measurement_noise_needs_data():
    assert fit_measurement_noise([1.0] * 3, [2.0] * 3) is None


def test_fit_rolling_resistance_recovers_params():
    # synthesise a coast under decel = c_roll + c_visc·v, then recover both
    c_roll, c_visc, dt = 250.0, 0.8, 0.01
    t, speed, v, tc = [], [], 900.0, 0.0
    while v > 5.0 and len(speed) < 5000:
        t.append(tc); speed.append(v)
        v -= (c_roll + c_visc * v) * dt
        tc += dt
    r = fit_rolling_resistance(t, speed)
    assert abs(r['c_roll'] - c_roll) < 25.0          # within ~10%
    assert abs(r['c_visc'] - c_visc) < 0.1


def test_fit_rolling_resistance_needs_data():
    assert fit_rolling_resistance([0.0, 0.1], [100.0, 90.0]) is None


def test_fit_rolling_resistance_rejects_collisions():
    # a clean coast plus an edge-ring collision (full speed -> 0 in one frame):
    # the ~10^4 mm/s2 spike must be rejected, not allowed to wreck the fit.
    c_roll, c_visc, dt = 250.0, 0.8, 0.01
    t, speed, v, tc = [], [], 900.0, 0.0
    while v > 30.0 and len(speed) < 2000:
        t.append(tc); speed.append(v)
        v -= (c_roll + c_visc * v) * dt
        tc += dt
    t.append(tc); speed.append(600.0); tc += dt     # ball re-flicked into wall
    t.append(tc); speed.append(0.0)                  # 600 -> 0: ~60000 mm/s2
    r = fit_rolling_resistance(t, speed)
    assert abs(r['c_roll'] - c_roll) < 40.0          # collision didn't dominate
    assert abs(r['c_visc'] - c_visc) < 0.15


def test_breakaway_a_from_theta_gates_ball_accel():
    # θ_s → breakaway_a is exactly the stiction threshold at that tilt
    a = breakaway_a_from_theta(5.0)
    assert math.isclose(
        a, SOLID_ALPHA * G_MM_S2 * math.sin(math.radians(5.0)), rel_tol=1e-9)
    th = math.radians(5.0)
    assert ball_accel(0.0, 0.0, th, 0.0, BallParams(breakaway_a=a * 1.001)) \
        == (0.0, 0.0)                                # just above ⇒ still stuck
    ax, _ = ball_accel(0.0, 0.0, th, 0.0, BallParams(breakaway_a=a * 0.999))
    assert ax > 0.0                                  # just below ⇒ breaks loose
