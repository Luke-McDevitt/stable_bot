"""Unit tests for stewart_bringup._ball_predictor — the pluggable lead
predictor behind the use_model_predictor control-method A/B. Pure NumPy-free
math, no ROS.

Pins the invariants the control node relies on: 'const_vel' reproduces the
legacy `x + v·Td` inline math exactly, the Phase-0 'model' stub is
behaviour-neutral (falls back to const_vel), and a misbehaving model can
never escape the const-velocity safety fallback.
"""
import math

from stewart_bringup._ball_predictor import predict_lead, load_ball_model


def test_const_vel_is_exact():
    # 10 + 2*0.25 = 10.5 ; -5 + 3*0.25 = -4.25
    assert predict_lead(10.0, -5.0, 2.0, 3.0, 0.25) == (10.5, -4.25)


def test_zero_velocity_is_identity():
    assert predict_lead(5.0, 7.0, 0.0, 0.0, 0.30) == (5.0, 7.0)


def test_model_stub_falls_back_to_const_vel():
    # Phase-0: method='model' with no model == const_vel (neutral flip).
    a = predict_lead(10.0, -5.0, 2.0, 3.0, 0.25, method='model', model=None)
    b = predict_lead(10.0, -5.0, 2.0, 3.0, 0.25, method='const_vel')
    assert a == b


def test_model_is_used_when_present():
    class _FakeModel:
        def integrate(self, px, py, vx, vy, td, hist):
            # deliberately different from const-vel so we can see it ran
            return px + vx * td + 1.0, py + vy * td - 1.0
    out = predict_lead(0.0, 0.0, 1.0, 1.0, 1.0, method='model',
                       model=_FakeModel())
    assert out == (2.0, 0.0)            # (0+1+1, 0+1-1)


def test_broken_model_falls_back_safely():
    class _BoomModel:
        def integrate(self, *a):
            raise RuntimeError("model blew up")
    # must NOT raise — falls back to const-velocity
    out = predict_lead(4.0, 4.0, 2.0, 0.0, 0.5, method='model',
                       model=_BoomModel())
    assert out == (5.0, 4.0)           # const-vel: 4+2*0.5, 4+0


def test_nonfinite_model_output_is_rejected():
    # NaN/inf don't raise, so the try/except wouldn't catch them — the finite
    # guard must. A non-finite lead would otherwise command a wild tilt and
    # break the level hold the controller does at rest.
    class _NanModel:
        def integrate(self, px, py, vx, vy, td, hist):
            return float('nan'), float('inf')
    out = predict_lead(4.0, 4.0, 2.0, 0.0, 0.5, method='model',
                       model=_NanModel())
    assert out == (5.0, 4.0)           # const-vel, non-finite rejected


def test_wild_model_output_is_rejected():
    # A model that teleports the ball far beyond a const-velocity step is a
    # bad fit; it must be distrusted so the loop holds the const-velocity lead.
    class _WildModel:
        def integrate(self, px, py, vx, vy, td, hist):
            return px + 10000.0, py - 10000.0
    out = predict_lead(0.0, 0.0, 1.0, 1.0, 0.1, method='model',
                       model=_WildModel())
    assert out == predict_lead(0.0, 0.0, 1.0, 1.0, 0.1)   # → const-vel


def test_load_ball_model_phase0_is_none():
    assert load_ball_model('whatever/path.yaml') is None
    assert load_ball_model(None) is None
