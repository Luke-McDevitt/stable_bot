"""Schema + sanity tests for stewart_vision/config/ball_safety.yaml.

A typo in this file silently produces a controller that, e.g., thinks
the dead-zone is 100 mm instead of 10 mm. These tests catch that.
"""
import os

import pytest
import yaml


def _safety_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'config', 'ball_safety.yaml')


@pytest.fixture(scope='module')
def safety() -> dict:
    p = _safety_path()
    assert os.path.isfile(p), f"missing {p}"
    with open(p) as f:
        return yaml.safe_load(f)


REQUIRED_KEYS = {
    'dead_zone_mm', 'ball_radius_mm', 'ball_density', 'margin_mm',
    'loss_timeout_ms', 'off_platform_radius_mm',
    'rearm_hysteresis_radius_mm', 'rearm_min_confidence',
    'rearm_min_dwell_s',
}


def test_all_required_keys_present(safety):
    missing = REQUIRED_KEYS - safety.keys()
    assert not missing, f"missing keys: {missing}"


def test_dead_zone_is_10mm_per_spec(safety):
    """Spec §10 — dead-zone is 10 mm clearance from the IMU bolt."""
    assert safety['dead_zone_mm'] == pytest.approx(10.0)


def test_ball_radius_is_20mm(safety):
    """40 mm-diameter ball (foam OR ping-pong)."""
    assert safety['ball_radius_mm'] == pytest.approx(20.0)


def test_dead_zone_plus_ball_fits_inside_platform(safety):
    """Dead-zone radius (ball center can't be inside) must be smaller
    than the platform radius — otherwise the ball can't be on the
    platform at all."""
    assert (safety['dead_zone_mm'] + safety['ball_radius_mm']
            < safety['off_platform_radius_mm'])


def test_rearm_hysteresis_is_inside_off_platform(safety):
    """Re-arm radius must be smaller than off-platform radius for the
    hysteresis to actually be hysteresis (otherwise the watchdog
    thrashes between abort and re-arm states)."""
    assert (safety['rearm_hysteresis_radius_mm']
            < safety['off_platform_radius_mm'])


def test_rearm_hysteresis_is_outside_dead_zone(safety):
    """Re-arm radius must exceed the dead-zone-plus-ball size, otherwise
    a ball legally placed for re-arm immediately violates the dead-zone."""
    assert (safety['rearm_hysteresis_radius_mm']
            > safety['dead_zone_mm'] + safety['ball_radius_mm'])


def test_ball_density_is_known_value(safety):
    """Maps to the alpha coefficient (5/7 vs 3/5) via _ball_physics."""
    assert safety['ball_density'] in ('solid', 'hollow')


def test_loss_timeout_is_positive_and_under_one_second(safety):
    """Spec §10 — 500 ms typical; > 1 s would let the ball roll off
    before the watchdog notices."""
    assert 0 < safety['loss_timeout_ms'] <= 1000


def test_rearm_min_confidence_is_in_unit_interval(safety):
    assert 0.0 <= safety['rearm_min_confidence'] <= 1.0


def test_rearm_min_dwell_is_positive(safety):
    assert safety['rearm_min_dwell_s'] > 0
