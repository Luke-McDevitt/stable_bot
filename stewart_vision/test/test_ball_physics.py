"""Unit tests for stewart_vision._ball_physics — the source of truth for
the controller's gravity-projection feedforward coefficient α.

If these pass and you trust them, the foam-vs-ping-pong distinction is
correctly threaded through the rest of the pipeline.
"""
import math

import pytest

from stewart_vision._ball_physics import (
    BALL_PRESETS,
    GRAVITY_MPS2,
    HOLLOW_ALPHA,
    HOLLOW_SPHERE_MOI_FACTOR,
    SOLID_ALPHA,
    SOLID_SPHERE_MOI_FACTOR,
    alpha_from_moi_factor,
    feedforward_tilt_for_base_accel,
    get_preset,
    saturation_fraction,
)


# --- the math ---------------------------------------------------------------

def test_solid_sphere_moi_factor_matches_textbook():
    assert SOLID_SPHERE_MOI_FACTOR == pytest.approx(2 / 5)


def test_hollow_sphere_moi_factor_matches_textbook():
    assert HOLLOW_SPHERE_MOI_FACTOR == pytest.approx(2 / 3)


def test_alpha_from_moi_factor_solid():
    """Solid sphere on tilted plane: α = 5/7."""
    assert alpha_from_moi_factor(2 / 5) == pytest.approx(5 / 7)


def test_alpha_from_moi_factor_hollow():
    """Hollow sphere on tilted plane: α = 3/5."""
    assert alpha_from_moi_factor(2 / 3) == pytest.approx(3 / 5)


def test_solid_ball_accelerates_faster_than_hollow():
    """Foam (solid) accelerates ~19 % faster than ping-pong (hollow) at
    the same tilt. This is the whole reason the GUI exposes a ball-type
    dropdown — controller gains aren't transferable across ball types.
    """
    ratio = SOLID_ALPHA / HOLLOW_ALPHA
    # 5/7 ÷ 3/5 = 25/21 ≈ 1.1905
    assert ratio == pytest.approx(25 / 21, rel=1e-9)
    assert ratio > 1.18


# --- preset library ---------------------------------------------------------

REQUIRED_PRESET_KEYS = {
    'type', 'radius_mm', 'mass_g', 'density', 'moi_factor', 'alpha',
}


@pytest.mark.parametrize('name', list(BALL_PRESETS.keys()))
def test_preset_has_required_fields(name):
    p = BALL_PRESETS[name]
    missing = REQUIRED_PRESET_KEYS - p.keys()
    assert not missing, f"preset {name!r} missing keys: {missing}"


def test_foam_preset_uses_solid_constants():
    p = BALL_PRESETS['foam']
    assert p['density'] == 'solid'
    assert p['moi_factor'] == pytest.approx(SOLID_SPHERE_MOI_FACTOR)
    assert p['alpha'] == pytest.approx(SOLID_ALPHA)


def test_ping_pong_preset_uses_hollow_constants():
    p = BALL_PRESETS['ping_pong']
    assert p['density'] == 'hollow'
    assert p['moi_factor'] == pytest.approx(HOLLOW_SPHERE_MOI_FACTOR)
    assert p['alpha'] == pytest.approx(HOLLOW_ALPHA)


def test_get_preset_returns_a_copy():
    """Mutating the returned dict shouldn't poison BALL_PRESETS."""
    p = get_preset('foam')
    p['type'] = 'mutated'
    assert BALL_PRESETS['foam']['type'] == 'foam'


def test_get_preset_unknown_raises_with_helpful_message():
    with pytest.raises(KeyError) as ei:
        get_preset('basketball')
    assert 'foam' in str(ei.value)
    assert 'ping_pong' in str(ei.value)


def test_both_balls_are_40mm():
    """Vision pipeline (HSV thresh, ROI sizing) assumes 40 mm ball."""
    for name in ('foam', 'ping_pong'):
        assert BALL_PRESETS[name]['diameter_mm'] == pytest.approx(40.0)
        assert BALL_PRESETS[name]['radius_mm'] == pytest.approx(20.0)


# --- Active-stabilization (BALL_HOLD) feedforward (spec §11.7, deferred v10) -

def test_feedforward_zero_at_zero_base_accel():
    """No base motion → no anticipatory tilt."""
    assert feedforward_tilt_for_base_accel(0.0, SOLID_ALPHA) == 0.0


def test_feedforward_proportional_to_base_accel():
    """Doubling the base acceleration doubles the required tilt."""
    t1 = feedforward_tilt_for_base_accel(1.0, SOLID_ALPHA)
    t2 = feedforward_tilt_for_base_accel(2.0, SOLID_ALPHA)
    assert t2 == pytest.approx(2 * t1)


def test_feedforward_sign_matches_base_direction():
    """Positive base acceleration → positive tilt (per docstring sign convention)."""
    assert feedforward_tilt_for_base_accel(+1.0, SOLID_ALPHA) > 0
    assert feedforward_tilt_for_base_accel(-1.0, SOLID_ALPHA) < 0


def test_feedforward_hollow_needs_more_tilt_than_solid():
    """Same base accel: hollow ball (lower α) needs MORE tilt because
    the same gravity component produces less linear acceleration on it.
    """
    a = 1.0  # m/s²
    solid_tilt = feedforward_tilt_for_base_accel(a, SOLID_ALPHA)
    hollow_tilt = feedforward_tilt_for_base_accel(a, HOLLOW_ALPHA)
    assert hollow_tilt > solid_tilt
    # Specific ratio: hollow / solid = (5/7) / (3/5) = 25/21 ≈ 1.19
    assert hollow_tilt / solid_tilt == pytest.approx(25 / 21, rel=1e-9)


def test_feedforward_invalid_alpha_raises():
    with pytest.raises(ValueError):
        feedforward_tilt_for_base_accel(1.0, ball_alpha=0.0)
    with pytest.raises(ValueError):
        feedforward_tilt_for_base_accel(1.0, ball_alpha=-0.5)


def test_feedforward_uses_gravity_correctly():
    """Sanity: the formula divides by α·g, so doubling g halves the tilt."""
    a = 1.0
    t_normal_g = feedforward_tilt_for_base_accel(a, SOLID_ALPHA, g=GRAVITY_MPS2)
    t_double_g = feedforward_tilt_for_base_accel(a, SOLID_ALPHA, g=2 * GRAVITY_MPS2)
    assert t_double_g == pytest.approx(t_normal_g / 2)


def test_saturation_fraction_zero_below_limit():
    assert saturation_fraction(0.0, max_safe_tilt_rad=0.5) == 0.0
    assert saturation_fraction(0.1, max_safe_tilt_rad=0.5) == pytest.approx(0.2)


def test_saturation_fraction_one_at_limit():
    assert saturation_fraction(0.5, max_safe_tilt_rad=0.5) == pytest.approx(1.0)


def test_saturation_fraction_uses_absolute_value():
    """A negative tilt is just as saturated as a positive one of the same magnitude."""
    assert saturation_fraction(-0.4, 0.5) == pytest.approx(saturation_fraction(0.4, 0.5))


def test_saturation_fraction_invalid_max_raises():
    with pytest.raises(ValueError):
        saturation_fraction(0.1, max_safe_tilt_rad=0.0)
