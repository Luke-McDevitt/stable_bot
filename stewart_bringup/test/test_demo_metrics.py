"""Unit tests for stewart_bringup._demo_metrics — point-to-point step-response
metrics used to compare controllers on identical gotos. Pure math, no ROS.
"""
import math

from stewart_bringup._demo_metrics import step_metrics, segment_gotos


def test_clean_step_no_overshoot():
    # first-order approach to 100 along x — never overshoots, perfectly straight
    t, bx, by, tc = [], [], [], 0.0
    while tc <= 4.0:
        t.append(tc); bx.append(100.0 * (1 - math.exp(-tc / 0.5)))
        by.append(0.0); tc += 0.02
    m = step_metrics(t, bx, by, 100.0, 0.0, tol_mm=15.0)
    assert m['overshoot_pct'] == 0.0
    assert m['reversals'] == 0
    assert m['max_lateral_mm'] < 1.0
    assert abs(m['path_ratio'] - 1.0) < 0.05
    assert m['rise_s'] is not None
    assert m['steady_state_err_mm'] < 5.0
    assert m['iae_mm_s'] > 0 and m['itae_mm_s2'] > 0


def test_overshoot_and_oscillation():
    # damped oscillation overshooting past the target
    t, bx, by, tc = [], [], [], 0.0
    while tc <= 5.0:
        x = 100.0 * (1 - math.exp(-tc / 0.4) * math.cos(2 * math.pi * tc / 0.8))
        t.append(tc); bx.append(x); by.append(0.0); tc += 0.02
    m = step_metrics(t, bx, by, 100.0, 0.0, tol_mm=15.0)
    assert m['overshoot_pct'] > 5.0          # it shot past 100
    assert m['reversals'] >= 2               # and oscillated
    assert m['settle_s'] >= m['rise_s']      # settling outlasts first arrival


def test_lateral_wander_off_the_line():
    # arcs sideways ~40 mm on the way from (0,0) to (100,0)
    t, bx, by, tc = [], [], [], 0.0
    while tc <= 2.0:
        f = tc / 2.0
        t.append(tc); bx.append(100.0 * f)
        by.append(40.0 * math.sin(math.pi * f)); tc += 0.02
    m = step_metrics(t, bx, by, 100.0, 0.0, tol_mm=15.0)
    assert m['max_lateral_mm'] > 30.0
    assert m['path_ratio'] > 1.1


def test_too_small_move_is_none():
    t = [i * 0.02 for i in range(20)]
    assert step_metrics(t, [50.0] * 20, [20.0] * 20, 51.0, 20.0) is None


def test_segment_gotos_splits_on_target_change():
    ref_t = [0.0, 0.5, 1.0, 2.0, 2.5, 3.0, 4.0]
    ref_xy = [(0, 0), (0, 0), (0, 0), (100, 0), (100, 0), (100, 0), (100, 0)]
    segs = segment_gotos(ref_t, ref_xy)
    assert len(segs) == 2
    assert segs[0][2] == (0, 0)
    assert segs[1][2] == (100, 0)
