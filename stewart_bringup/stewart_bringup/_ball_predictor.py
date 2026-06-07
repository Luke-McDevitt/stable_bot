"""_ball_predictor.py — pluggable lead-position predictor for BALL_TRACK.

The controller must act on where the ball WILL be when its tilt command
actually takes effect — i.e. one dead time `Td` (≈ see + actuation + control)
in the future — not where the camera last saw it. Today that lead is a
constant-velocity extrapolation `x + v·Td` (inline in
`stewart_control_node._ball_track_run`). This module makes the predictor a
single pluggable function so a fitted physics model can replace the
constant-velocity term behind the `use_model_predictor` gain, A/B-able
against the current behaviour — see `docs/ball_physics_modeling_plan.md` §8.1
and `docs/physics_model_implementation_plan.md`.

Pure functions, no ROS / no hardware → unit-tested in
`test/test_ball_predictor.py`.

Phase 0 (this file): the `'model'` method is a stub that falls back to
constant-velocity, so flipping the gain is behaviour-neutral until a real
model artifact exists. Phase 1 loads a fitted model and the `'model'` branch
forward-integrates the ball through the *known* commanded-tilt history.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple


def predict_lead(px: float, py: float, vx: float, vy: float, td_s: float,
                 method: str = 'const_vel',
                 tilt_history: Optional[Sequence] = None,
                 model=None) -> Tuple[float, float]:
    """Predicted ball position (px_lead, py_lead) a dead time `td_s` ahead.

    Args:
      px, py : current ball position (mm, platform frame).
      vx, vy : current ball velocity (mm/s).
      td_s   : dead time to extrapolate over (seconds).
      method : 'const_vel' (default; == today's controller) or 'model'.
      tilt_history : ring buffer of recently-commanded tilts (Phase 1 — the
                     known input the model integrates through). Ignored by
                     the const-velocity path and the Phase-0 stub.
      model  : a fitted forward model with `.integrate(px,py,vx,vy,td_s,
               tilt_history) -> (px_lead, py_lead)`, or None.

    Returns (px_lead, py_lead).

    Invariants the tests pin:
      - 'const_vel' is exactly `x + v·td` (so the controller's legacy inline
        math is reproduced bit-for-bit).
      - 'model' with `model is None` falls back to 'const_vel' (Phase-0
        behaviour-neutral flip).
    """
    cvx, cvy = px + vx * td_s, py + vy * td_s        # const-velocity lead
    if method == 'model' and model is not None:
        try:
            lx, ly = model.integrate(px, py, vx, vy, td_s, tilt_history)
            # The model is a *refinement* of the constant-velocity lead, not a
            # teleport. Accept it only if it is finite AND lands within one
            # const-velocity step (+50 mm slack) of the const-velocity lead;
            # anything wilder (a bad fit, a NaN that wouldn't raise, a runaway
            # integration) is rejected so it can never destabilise the loop —
            # the controller then holds level exactly as the PID path does.
            import math
            if math.isfinite(lx) and math.isfinite(ly):
                slack = math.hypot(vx, vy) * td_s + 50.0
                if math.hypot(lx - cvx, ly - cvy) <= slack:
                    return lx, ly
        except Exception:
            pass
    return cvx, cvy


def load_ball_model(path: Optional[str]):
    """Load a fitted forward-model artifact (ball_model.yaml), or None if
    absent / unbuilt / unreadable. Returns a `BallForwardModel` exposing
    `.integrate(px,py,vx,vy,td_s,tilt_history)`. None ⇒ constant-velocity
    everywhere — the safe default until the Phase-1 fitter writes the file."""
    import os
    if not path or not os.path.isfile(path):
        return None
    try:
        import yaml
        from stewart_bringup._ball_model import BallForwardModel
        with open(path) as f:
            return BallForwardModel.from_dict(yaml.safe_load(f))
    except Exception:
        return None
