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
