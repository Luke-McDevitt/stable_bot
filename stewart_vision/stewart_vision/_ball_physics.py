"""Pure-math helpers for ball-on-tilted-plate dynamics.

Spec: ../../stewart_bringup/docs/closed_loop_ball_demos.md §9.

This module is the SINGLE SOURCE OF TRUTH for ball physical constants
used by the controller's gravity-projection feedforward and by the
GUI's ball-type dropdown. Both stewart_control_node (in stewart_bringup)
and ref_generator_node (here) should import from this module rather
than hardcoding alpha values.

Plant equation (rolling sphere on tilted plane, no slip):

    ẍ = α · g · sin θ        where  α = 1 / (1 + I / (m · R²))

Solid sphere (uniform density, e.g. foam):  I = (2/5) · m · R²  → α = 5/7
Hollow sphere (thin shell, e.g. ping-pong): I = (2/3) · m · R²  → α = 3/5

So foam (mass-matched to ping-pong, same diameter) accelerates
~19 % faster at the same tilt — controller feedforward and Kd both
need to be aware of which ball is on the platform.
"""
from __future__ import annotations

# I / (m·R²) — dimensionless moment-of-inertia coefficient.
SOLID_SPHERE_MOI_FACTOR = 2.0 / 5.0       # 0.4
HOLLOW_SPHERE_MOI_FACTOR = 2.0 / 3.0      # 0.6667


def alpha_from_moi_factor(moi_factor: float) -> float:
    """Return α such that ẍ = α · g · sin θ for the given moi_factor."""
    return 1.0 / (1.0 + moi_factor)


SOLID_ALPHA = alpha_from_moi_factor(SOLID_SPHERE_MOI_FACTOR)        # 5/7 ≈ 0.7143
HOLLOW_ALPHA = alpha_from_moi_factor(HOLLOW_SPHERE_MOI_FACTOR)      # 3/5  = 0.6


# Ball preset library. Keys are the values used in the GUI dropdown;
# the JSON-serialized preset is what flows through `/control_cmd`'s
# `ball_config:` payload and into the bag for post-run analysis.
BALL_PRESETS = {
    'foam': {
        'type': 'foam',
        'label': 'Foam (40 mm, mass-matched, solid)',
        'diameter_mm': 40.0,
        'radius_mm': 20.0,
        'mass_g': 2.7,
        'density': 'solid',
        'moi_factor': SOLID_SPHERE_MOI_FACTOR,
        'alpha': SOLID_ALPHA,
    },
    'ping_pong': {
        'type': 'ping_pong',
        'label': 'Ping-pong (40 mm, hollow)',
        'diameter_mm': 40.0,
        'radius_mm': 20.0,
        'mass_g': 2.7,
        'density': 'hollow',
        'moi_factor': HOLLOW_SPHERE_MOI_FACTOR,
        'alpha': HOLLOW_ALPHA,
    },
}


def get_preset(name: str) -> dict:
    """Look up a preset by name; raise KeyError with helpful message if missing."""
    if name not in BALL_PRESETS:
        valid = ', '.join(sorted(BALL_PRESETS.keys()))
        raise KeyError(
            f"unknown ball preset {name!r}; valid options: {valid}")
    return dict(BALL_PRESETS[name])  # shallow copy so callers can mutate


# -- Active stabilization (BALL_HOLD mode, v10) ------------------------------
#
# The math for the base-IMU feedforward used by Ball-Hold mode lives here so
# both the controller (stewart_control_node) and the unit tests can import
# the same source of truth. Implementation of the actual closed loop is
# deferred to v10 (see spec §11.7).

GRAVITY_MPS2 = 9.81


def feedforward_tilt_for_base_accel(
    base_accel_mps2: float,
    ball_alpha: float,
    g: float = GRAVITY_MPS2,
) -> float:
    """Return the anticipatory tilt (radians) that cancels a horizontal
    base-frame acceleration on a ball rolling on the platform.

    Derivation (small-angle):
        ẍ_ball_in_plate = α · g · sin θ  −  a_base
        For ẍ_ball_in_plate = 0:
            sin θ = a_base / (α · g)
        Small angle: θ ≈ a_base / (α · g)

    Sign convention: positive `base_accel_mps2` pushing the platform in
    the +x direction needs a positive tilt about the y-axis (right-hand
    rule), which makes the plate slope downward toward +x and creates
    gravity-induced ball acceleration in the +x direction matching the
    base.

    Inputs:
      base_accel_mps2: horizontal acceleration of the base in the
        platform's x or y axis (m/s²). Apply once per axis.
      ball_alpha: 5/7 for solid sphere, 3/5 for hollow sphere — pull
        from BALL_PRESETS based on which ball is in use.
      g: gravity (m/s²); override for testing.

    Returns: tilt angle in radians. Caller is responsible for clipping
    to the platform's tilt envelope (see global_limits.yaml).
    """
    if ball_alpha <= 0:
        raise ValueError(f"ball_alpha must be > 0; got {ball_alpha}")
    return base_accel_mps2 / (ball_alpha * g)


def saturation_fraction(
    commanded_tilt_rad: float,
    max_safe_tilt_rad: float,
) -> float:
    """How close are we to the platform's tilt envelope?

    Returns a value in [0, ∞). Values ≥ 1 mean we're saturated and
    can't fully compensate for the requested motion — the operator's
    base perturbation has exceeded what the Stewart workspace + tilt
    cap can reject. The GUI surfaces this as a status indicator and
    the controller should fall back to plate-frame holding when
    saturation > 0.95 sustained for >100 ms (spec §11.7).
    """
    if max_safe_tilt_rad <= 0:
        raise ValueError(f"max_safe_tilt_rad must be > 0; got {max_safe_tilt_rad}")
    return abs(commanded_tilt_rad) / max_safe_tilt_rad
