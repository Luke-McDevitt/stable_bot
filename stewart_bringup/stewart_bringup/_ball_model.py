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


# ----- identification (the fitter's math; pure + unit-tested) ----------------

def fit_measurement_noise(px, py):
    """Measurement noise R from a STILL-ball position series — the scatter
    about the mean is the per-axis vision std (mm). Feeds the KF `R` tune.
    Returns a dict or None."""
    n = len(px)
    if n < 10:
        return None
    mx = sum(px) / n
    my = sum(py) / n
    sx = (sum((x - mx) ** 2 for x in px) / (n - 1)) ** 0.5
    sy = (sum((y - my) ** 2 for y in py) / (n - 1)) ** 0.5
    return {'std_x_mm': round(sx, 3), 'std_y_mm': round(sy, 3),
            'R_mm': round((sx + sy) / 2.0, 3), 'n': n}


def fit_rolling_resistance(t_s, speed, v_floor=20.0, max_decel_mm_s2=3000.0,
                           min_dt_s=0.005):
    """From a COASTING (level-plate) speed-vs-time series, fit
    decel = c_roll + c_visc·v (Coulomb + viscous rolling resistance) by linear
    regression of the finite-difference deceleration against speed. Ignores
    the near-stop regime (< v_floor) where stiction, not rolling resistance,
    dominates, AND rejects decelerations above `max_decel_mm_s2` as collisions
    — a ball hitting the edge ring crashes from full speed to 0 in one frame
    (10⁴–10⁷ mm/s²), which would otherwise dominate the fit, whereas real
    rolling resistance on a level plate is at most a few hundred mm/s².
    Returns {'c_roll' (mm/s²), 'c_visc' (1/s), 'n'} or None."""
    vs, ds = [], []
    for i in range(len(t_s) - 1):
        dt = t_s[i + 1] - t_s[i]
        if dt < min_dt_s:
            continue          # skip near-duplicate timestamps (Δv/~0 = huge)
        v = 0.5 * (speed[i] + speed[i + 1])
        if v < v_floor:
            continue
        d = -(speed[i + 1] - speed[i]) / dt      # deceleration (positive)
        if d <= 0 or d > max_decel_mm_s2:
            continue                  # decelerating part only; reject collisions
        vs.append(v)
        ds.append(d)
    n = len(vs)
    if n < 5:
        return None
    sv = sum(vs); sd = sum(ds)
    svv = sum(v * v for v in vs)
    svd = sum(vs[i] * ds[i] for i in range(n))
    denom = n * svv - sv * sv
    if abs(denom) < 1e-9:
        return None
    c_visc = (n * svd - sv * sd) / denom
    c_roll = (sd - c_visc * sv) / n
    return {'c_roll': round(max(0.0, c_roll), 2),
            'c_visc': round(max(0.0, c_visc), 5), 'n': n}


def breakaway_a_from_theta(theta_deg, alpha=SOLID_ALPHA):
    """Stiction breakaway acceleration (mm/s²) from the measured breakaway
    tilt θ_s (deg): the drive a stationary ball must overcome = α·g·sin(θ_s)."""
    return alpha * G_MM_S2 * math.sin(math.radians(theta_deg))


def resample_uniform(t_s, vals, dt_grid=0.02):
    """Bin an irregular (t, vals) series onto a uniform `dt_grid` (median per
    bin). /ball_state is published from BOTH a ~100 Hz timer and an immediate
    event-driven path on each measurement, so it arrives in bursts with
    near-duplicate timestamps; differentiating that raw is meaningless. Median-
    per-bin collapses each burst to one representative sample on an even grid
    (and rejects the per-measurement velocity spike as an outlier), so a
    deceleration taken across bins is the real one. Returns (t_grid, v_grid)."""
    if len(t_s) < 2:
        return list(t_s), list(vals)
    t0 = t_s[0]
    bins = {}
    for i in range(len(t_s)):
        k = int((t_s[i] - t0) / dt_grid)
        bins.setdefault(k, []).append(vals[i])
    tg, vg = [], []
    for k in sorted(bins):
        s = sorted(bins[k])
        m = len(s) // 2
        tg.append(t0 + (k + 0.5) * dt_grid)
        vg.append(s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m]))
    return tg, vg
