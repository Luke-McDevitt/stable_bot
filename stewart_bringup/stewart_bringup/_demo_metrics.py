"""_demo_metrics.py — point-to-point (step-response) metrics for a BALL_TRACK
goto, for comparing controllers (e.g. PID vs physics-model) on identical
start→target moves.

rms-over-a-whole-run is too blunt: it's dominated by run length and the goto
sequence. For a single move from a known start to a known target the right
language is the classic step response — rise / overshoot / settle / steady
state — plus path quality (how straight) and oscillation. Each metric is
chosen so a *better* controller scores *lower* (or the noted direction).

Pure math (no ROS / numpy) → unit-tested in test/test_demo_metrics.py.
"""
from __future__ import annotations

import math


def segment_gotos(ref_t, ref_xy, min_move_mm=20.0, min_dwell_s=0.4):
    """Split a /ball_ref series into goto segments — each maximal stretch with a
    ~constant target. Returns [(t_start, t_end, (tx, ty)), ...]. A single
    place-and-send run yields one segment."""
    segs = []
    if not ref_t:
        return segs
    cur = ref_xy[0]
    t0 = ref_t[0]
    for i in range(1, len(ref_t)):
        if math.hypot(ref_xy[i][0] - cur[0], ref_xy[i][1] - cur[1]) > min_move_mm:
            if ref_t[i] - t0 >= min_dwell_s:
                segs.append((t0, ref_t[i], cur))
            cur = ref_xy[i]
            t0 = ref_t[i]
    if ref_t[-1] - t0 >= min_dwell_s:
        segs.append((t0, ref_t[-1], cur))
    return segs


def step_metrics(t, bx, by, tgt_x, tgt_y, tol_mm=15.0):
    """Step-response metrics for one goto. `t,bx,by` are the ball trajectory in
    the segment (start = first sample); target is (tgt_x, tgt_y). Returns a
    dict, or None if the move is too small to score.

    Lower-is-better unless noted:
      overshoot_pct      excursion PAST the target along the move axis, %% of
                         the move distance. The model's tilt anticipation should
                         cut this most.
      rise_s             time to FIRST reach the tolerance band.
      settle_s           time after which error stays inside 1.5×tol.
      steady_state_err   mean |error| over the last quarter (residual offset).
      settled_rms        jitter (rms error) over the last quarter — limit cycle.
      max_lateral_mm     worst perpendicular wander off the straight start→
                         target line (path adherence).
      rms_lateral_mm     rms of that wander.
      path_ratio         actual path length / straight-line distance (1.0 =
                         perfectly straight; higher = wandered/looped).
      reversals          # of along-axis direction changes (oscillation count).
    """
    n = len(t)
    if n < 5:
        return None
    sx, sy = bx[0], by[0]
    D = math.hypot(tgt_x - sx, tgt_y - sy)
    if D < 5.0:
        return None
    ux, uy = (tgt_x - sx) / D, (tgt_y - sy) / D          # along-axis unit
    prog = [(bx[i] - sx) * ux + (by[i] - sy) * uy for i in range(n)]
    lat = [abs(-(bx[i] - sx) * uy + (by[i] - sy) * ux) for i in range(n)]
    err = [math.hypot(bx[i] - tgt_x, by[i] - tgt_y) for i in range(n)]

    over_mm = max(0.0, max(prog) - D)
    rise = next((t[i] - t[0] for i in range(n) if err[i] < tol_mm), None)
    settle_idx = None
    for i in range(n):
        if err[i] >= 1.5 * tol_mm:
            settle_idx = i
    settle = (t[settle_idx] - t[0]) if settle_idx is not None else 0.0

    tail = max(1, n // 4)
    ss_err = sum(err[-tail:]) / tail
    settled_rms = (sum(e * e for e in err[-tail:]) / tail) ** 0.5

    path_len = sum(math.hypot(bx[i] - bx[i - 1], by[i] - by[i - 1])
                   for i in range(1, n))
    path_ratio = path_len / D
    max_lat = max(lat)
    rms_lat = (sum(v * v for v in lat) / n) ** 0.5

    reversals, last_dir = 0, 0
    iae = itae = 0.0
    for i in range(1, n):
        d = prog[i] - prog[i - 1]
        if abs(d) >= 0.5:                                # ignore noise jitter
            dr = 1 if d > 0 else -1
            if last_dir and dr != last_dir:
                reversals += 1
            last_dir = dr
        # classic step-response integrals (lower = better): IAE = ∫|e|dt,
        # ITAE = ∫t|e|dt (extra-penalises errors that linger).
        dt = t[i] - t[i - 1]
        iae += err[i] * dt
        itae += (t[i] - t[0]) * err[i] * dt

    return {
        'distance_mm': round(D, 1),
        'overshoot_pct': round(100.0 * over_mm / D, 1),
        'overshoot_mm': round(over_mm, 1),
        'rise_s': round(rise, 2) if rise is not None else None,
        'settle_s': round(settle, 2),
        'steady_state_err_mm': round(ss_err, 1),
        'settled_rms_mm': round(settled_rms, 1),
        'max_lateral_mm': round(max_lat, 1),
        'rms_lateral_mm': round(rms_lat, 1),
        'path_ratio': round(path_ratio, 2),
        'reversals': reversals,
        'iae_mm_s': round(iae, 1),
        'itae_mm_s2': round(itae, 1),
        'n': n,
    }
