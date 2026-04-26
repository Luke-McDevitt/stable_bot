"""Unit tests for stewart_vision._ball_physics — the source of truth for
the controller's gravity-projection feedforward coefficient α.

If these pass and you trust them, the foam-vs-ping-pong distinction is
correctly threaded through the rest of the pipeline.
"""
import math

import pytest

from stewart_vision._ball_physics import (
    BALL_PRESETS,
    HOLLOW_ALPHA,
    HOLLOW_SPHERE_MOI_FACTOR,
    SOLID_ALPHA,
    SOLID_SPHERE_MOI_FACTOR,
    alpha_from_moi_factor,
    get_preset,
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
