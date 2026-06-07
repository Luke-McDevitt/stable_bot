"""_ball_model.py — fitted forward model of the ball-on-tilted-plate plant.

The gray-box dynamics the Phase-1 fitter (`scripts/fit_ball_forward_model.py`)
identifies and that the control predictor forward-integrates over the dead
time (`_ball_predictor.predict_lead`'s `'model'` branch). Pure Python (no
numpy) so it's cheap in the BALL_TRACK loop and unit-testable anywhere.

Frame & units: platform-frame position mm, velocity mm/s, tilt in radians
(tx = downhill slope toward +x, ty = toward +y). Gravity in mm/s².

Model hierarchy (`docs/ball_physics_modeling_plan.md` §2; fit only as deep as
the held-out horizon error needs — §10):
  Tier 1  rolling drive : a = α·g·sin(tilt),  α = 1/(1+I/mR²)  (5/7 solid)
  Tier 3  friction      : rolling resistance (Coulomb, opposes v) + viscous
                          + a stiction breakaway (a stationary ball below the
                          breakaway drive stays stuck — the "needs a startup
                          ramp to break loose" behaviour)
  Tier 2  plate coupling: hook (a_base / ω terms) — added only if a residual
                          demands it; not in the default params.

`alpha` mirrors `stewart_vision/_ball_physics.SOLID_ALPHA` (5/7); kept here so
the control package has no cross-package import.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Optional

G_MM_S2 = 9810.0          # gravity, mm/s²
SOLID_ALPHA = 5.0 / 7.0   # rolling coefficient for a uniform (foam) sphere


@dataclass
class BallParams:
    """Fittable plant parameters (gray-box). Defaults = Tier-1-only (pure
    rolling, no friction) so a fresh model reduces to the textbook plant; the
    fitter sets the friction terms from data."""
    alpha: float = SOLID_ALPHA        # rolling drive coefficient (—)
    c_roll: float = 0.0               # rolling resistance, mm/s² (opposes v)
    c_visc: float = 0.0               # viscous damping, 1/s (a = −c_visc·v)
    breakaway_a: float = 0.0          # stiction: a stationary ball needs the
                                      # drive to exceed this (mm/s²) to move
    v_eps: float = 1.0                # |v| below this is "stationary" (mm/s)

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> 'BallParams':
        d = d or {}
        f = {k: float(d[k]) for k in cls().__dict__ if k in d}
        return cls(**f)


def ball_accel(vx: float, vy: float, tx: float, ty: float,
               p: BallParams, g: float = G_MM_S2):
    """Instantaneous ball acceleration (ax, ay) in mm/s² — the plant core the
    fitter regresses and the integrator steps. Level + frictionless ⇒ (0, 0),
    so the model reduces to constant velocity."""
    ax_drive = p.alpha * g * math.sin(tx)
    ay_drive = p.alpha * g * math.sin(ty)
    speed = math.hypot(vx, vy)
    if speed < p.v_eps:
        # Stationary: stays stuck unless the drive beats the stiction
        # breakaway. (No kinetic friction subtracted at rest.)
        if math.hypot(ax_drive, ay_drive) < p.breakaway_a:
            return 0.0, 0.0
        return ax_drive, ay_drive
    # Moving: rolling resistance (Coulomb, opposing the velocity unit vector)
    # + viscous damping.
    inv = 1.0 / speed
    ax = ax_drive - p.c_roll * (vx * inv) - p.c_visc * vx
    ay = ay_drive - p.c_roll * (vy * inv) - p.c_visc * vy
    return ax, ay


def simulate(px, py, vx, vy, tilt_x_seq, tilt_y_seq, dt, p: BallParams):
    """Roll the model forward through a tilt SEQUENCE (semi-implicit Euler),
    returning the position+velocity trajectory. This is what the fitter scores
    against the recorded ball (input-replay, §7) and the sim-vs-real digest
    uses. `tilt_*_seq` are equal-length per-step tilt commands (rad)."""
    px_t, py_t, vx_t, vy_t = [px], [py], [vx], [vy]
    for tx, ty in zip(tilt_x_seq, tilt_y_seq):
        ax, ay = ball_accel(vx, vy, tx, ty, p)
        vx += ax * dt
        vy += ay * dt
        px += vx * dt
        py += vy * dt
        px_t.append(px); py_t.append(py); vx_t.append(vx); vy_t.append(vy)
    return px_t, py_t, vx_t, vy_t


class BallForwardModel:
    """Fitted model wrapper used by the control predictor. `.integrate()` is
    the dead-time forward prediction `predict_lead` calls; `.params` is what
    the fitter writes to `config/ball_model.yaml`."""

    def __init__(self, params: Optional[BallParams] = None, dt: float = 0.01):
        self.params = params or BallParams()
        self.dt = float(dt)

    def integrate(self, px, py, vx, vy, td_s, tilt_history=None):
        """Predict ball position a dead time `td_s` ahead, holding the most
        recent commanded tilt over the window (Phase-1 assumption — the
        command changes slowly vs Td; the tilt-history-as-function refinement
        is a documented next step). `tilt_history` is a sequence of recent
        (tx, ty) tilt commands in radians; None/empty ⇒ level."""
        if td_s <= 0.0:
            return px, py
        tx, ty = (tilt_history[-1] if tilt_history else (0.0, 0.0))
        n = max(1, int(round(td_s / self.dt)))
        h = td_s / n
        for _ in range(n):
            ax, ay = ball_accel(vx, vy, tx, ty, self.params)
            vx += ax * h
            vy += ay * h
            px += vx * h
            py += vy * h
        return px, py

    # --- artifact I/O -----------------------------------------------------
    def to_dict(self) -> dict:
        return {'schema': 'ball_forward_model/v1',
                'dt': self.dt, 'params': self.params.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> 'BallForwardModel':
        d = d or {}
        return cls(params=BallParams.from_dict(d.get('params')),
                   dt=float(d.get('dt', 0.01)))
