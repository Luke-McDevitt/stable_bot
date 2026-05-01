#!/usr/bin/env python3
"""auto_tune_node — autonomous controller-gain tuning for Stable-Bot.

Drives a sequence of "goto" trials at fixed-distance random targets,
scores each by a multi-component fitness, and uses coordinate-descent
hill-climbing to optimize the gain set in `ball_track_gains.yaml`.
Spec: stewart_bringup/docs/auto_tuning_plan.md.

What this node does (per trial):
  1. Apply candidate gains via /control_cmd ball_track_save_gains
  2. mode:LEVEL_HOLD for 2 s (let ball settle)
  3. Read current ball position from /ball_state
  4. Pick a random target at distance D from the ball (16 attempts to
     stay within 0.7·R_platform; shrink D once if all attempts fail)
  5. Publish mode:BALL_TRACK_GOTO with the target
  6. Collect /ball_state samples until settled (err<10mm for 1s) or
     trial_timeout (default 25 s)
  7. Compute fitness (rms, p95, settling, hold-fraction, calmness)
  8. Append a JSONL row to tuning_data/auto_tune_<UTC>/log.jsonl
  9. Hill-climb step: accept the perturbation if f > best + noise_floor,
     anneal the step size after 3 consecutive non-improvements
 10. Publish /auto_tune/status (JSON String) for the GUI

Triggers (service calls):
  /auto_tune/start  (Trigger)  — begin a tuning run; reads current
                                  gains from ball_track_gains.yaml as
                                  the starting point
  /auto_tune/stop   (Trigger)  — abort gracefully; saves current best
                                  gains to ball_track_gains.yaml

Topics:
  Subscribes:
    /ball_state         (geometry_msgs/PoseStamped) — KF posterior
    /control_result     (std_msgs/String) — handshake for gain save
  Publishes:
    /control_cmd        (std_msgs/String) — gain saves + mode commands
    /auto_tune/status   (std_msgs/String, latched JSON)

Pre-flight requirements (operator):
  - Platform armed and able to LEVEL_HOLD
  - Ball detector running (cv2 or v1_yolo); /ball_state at >5 Hz
  - Foam ring around platform rim so a bad gain set can't escape the ball
  - mode:BALL_TRACK_GOTO has been verified to track in the right
    direction (per stewart_bringup/docs/auto_tuning_plan.md §0)
"""
from __future__ import annotations

import glob
import json
import math
import os
import random
import signal
import subprocess
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import (Bool, Float32MultiArray, Float64,
                          Int32MultiArray, String)
from std_srvs.srv import Trigger

# SetPose lives in jugglebot_interfaces; we use it to reset platform Z
# to nominal between trials so accumulated tilt-induced Z drift can't
# turn the platform into a bowl that pulls the ball toward the center bolt.
try:
    from jugglebot_interfaces.srv import SetPose
    HAVE_SET_POSE = True
except ImportError:
    HAVE_SET_POSE = False


# ----- Tuning configuration ---------------------------------------------------

# Per-variable bounds, initial step size, and "categorical" flag. The
# values match auto_tuning_plan.md §4. Bounds clamp before any apply.
@dataclass
class Knob:
    name: str
    lo: float
    hi: float
    step: float
    categorical: bool = False  # if True, step ignored; perturbation flips the value

KNOBS: list[Knob] = [
    Knob('kp',                   0.001, 0.050, 0.002),
    Knob('kd',                   0.000, 0.100, 0.005),
    Knob('ki',                   0.000, 0.050, 0.002),
    Knob('max_tilt_deg',         1.0,   8.0,   0.5),
    Knob('pitch_sign',          -1.0,   1.0,   0.0, categorical=True),
    Knob('roll_sign',           -1.0,   1.0,   0.0, categorical=True),
]

# Trial protocol
TRIAL_DISTANCE_MM      = 60.0    # target distance from current ball position
PLATFORM_R_MM          = 200.0   # safe-region radius
SAFE_RADIUS_FRAC       = 0.70    # target must be within 0.7*R_platform
CENTER_EXCLUSION_MM    = 40.0    # target must be > this from platform center
                                  # (stops the ball getting parked on the
                                  # central mounting bolt)
TARGET_PICK_ATTEMPTS   = 16
SETTLE_BAND_MM         = 10.0    # err < this for SETTLE_HOLD_S → settled
SETTLE_HOLD_S          = 1.0
TRIAL_TIMEOUT_S        = 25.0
LEVEL_HOLD_BETWEEN_S   = 2.0     # minimum dwell, regardless of calm test
HOLD_TOL_MM            = 15.0    # diagnostic-only threshold for the
                                  # on_target_fraction stat in components.
                                  # f_hold itself is now continuous (see
                                  # CLOSENESS_SCALE_MM) — the threshold
                                  # was dropped because it gave a sample
                                  # 14mm from target full credit and a
                                  # sample 16mm from target zero credit,
                                  # which is the wrong shape.
CLOSENESS_SCALE_MM     = 20.0    # f_hold = mean(exp(-err / scale)).
                                  # err=0  → reward 1.00
                                  # err=10 → reward 0.61
                                  # err=20 → reward 0.37
                                  # err=40 → reward 0.14
                                  # err=60 → reward 0.05
                                  # Smooth: closer is always better,
                                  # and time-on-target is rewarded by
                                  # averaging across all samples.
# Inter-trial calm-down: wait until ball velocity drops below this
# threshold for CALM_DWELL_S before starting the next trial. Stops
# trials starting while the ball is still orbiting from the previous
# trial's commanded tilt.
CALM_VEL_THRESHOLD_MM_S = 30.0
CALM_DWELL_S            = 0.5
CALM_MAX_WAIT_S         = 8.0    # cap wait if ball never fully calms
RESET_Z_EVERY_N_TRIALS = 5       # call set_pose to nominal Z every N trials
                                  # to defeat accumulated drift
NOMINAL_Z_MM           = 75.0    # higher than the 30-50 demo range —
                                  # less ball-sticking on the center bolt
                                  # (per operator observation 2026-05-01)
RESET_LEVEL_DWELL_S    = 5.0     # after Z reset, hold LEVEL_HOLD this long
                                  # so platform fully settles before the
                                  # next trial starts

# Topics recorded into the per-session mcap bag. Mirrors the IVA /
# vision-debug topic set + adds the auto_tune-specific status and
# trial events. /auto_tune/trial events provide the natural
# subsection markers — replay trial-by-trial offline by filtering
# on those timestamps.
AUTOTUNE_BAG_TOPICS = [
    '/ball_state',
    '/ball_ref',
    '/ball_xy_mono',
    '/platform_pose',
    '/platform_rpy',
    '/platform/imu/data',
    '/control_cmd',
    '/control_result',
    '/auto_tune/status',
    '/auto_tune/trial',
    '/oak/ball/v0/yolo_pixel',
    '/oak/ball/v0/cv2_pixel',
    '/leg_encoders',
    '/status',
]

# Optimizer (Twiddle / coordinate descent with adaptive steps).
#
# Sebastian Thrun's "Twiddle" algorithm (AI for Robotics / Udacity
# SDC course) — well-tested for low-dim noisy black-box optimization,
# specifically PID tuning. Per knob: try +step, then -step, then
# shrink. Step grows ×1.1 on success, shrinks ×0.9 on both-directions
# failure. NO explicit noise floor: step adaptation absorbs noise
# (a noisy "miss" just moves us to the next knob; eventually we'll
# revisit at a smaller step where the signal is bigger relative to
# the perturbation).
#
# Earlier hand-rolled hill-climb had three bugs operator caught:
#   - NOISE_FLOOR=0.02 was wider than typical step improvements
#     (~0.018) → real improvements rejected as "noise."
#   - Only ever tried +step (random direction); never the opposite.
#   - Aggressive ×0.7 step shrink after 3 fails → converged to
#     micro-steps before exploring enough.
#
# Twiddle's step grows on success — so a series of accepted moves
# accelerates the search rather than slowing it. And shrink only
# fires after BOTH directions of a knob fail, doubling the
# information per shrink decision.
TWIDDLE_GROW           = 1.1
TWIDDLE_SHRINK         = 0.9
SAFETY_BAD_TRIALS      = 5       # 5x f<SAFETY_BAD_THRESH → halt
SAFETY_BAD_THRESH      = 0.10
# Lucky-baseline guard. If the running best hasn't been beaten in
# N trials, re-evaluate it to refresh the fitness measurement.
# This regresses lucky one-shot highs toward the truth.
REEVAL_AFTER_REJECTIONS = 6
REEVAL_WEIGHT_NEW       = 0.6    # new sample gets 60% weight, old 40%
# Sign knobs (pitch_sign, roll_sign) are categorical and direction-
# tested manually before tuning starts. After ONE rejected flip, we
# have strong evidence the current sign is correct — flipping a
# correct sign causes the controller to push the ball AWAY from the
# target, which collapses fitness (typical Δf > 0.2, well above
# noise floor 0.02). Locking after 1 rejection gives ~95%+
# statistical confidence per the noise-vs-effect-size argument.
# Set to a higher number on hardware that hasn't been pre-tested.
SIGN_FLIP_LOCK_AFTER_REJECTS = 2

# ----- STEP_ID mode configuration -------------------------------------------
#
# STEP_ID is an alternative tuning mode that replaces random-target Twiddle
# with a deterministic protocol designed to compute controller gains
# analytically rather than search for them. See:
#   stewart_bringup/docs/auto_tuning_plan.md  (original Twiddle plan)
# Rationale for this alternative path: the random-target picker generated
# trial-to-trial fitness noise (~50 mm RMS) larger than the gain effect
# Twiddle was trying to detect, so the search couldn't separate signal from
# luck (typical session: 27 trials, 0 acceptances). STEP_ID instead:
#   1. characterizes the cascaded plant with one open-loop tilt step
#      (ball trajectory ≈ ½·G_eff·sin(θ)·t²; fit the parabola → G_eff)
#   2. computes Kp = ωn²/G_eff, Kd = 2·ζ·ωn/G_eff analytically
#   3. verifies with 4 closed-loop goto trials between known marker
#      positions (deterministic start/target → directly comparable bags
#      across sessions)

# Open-loop tilt-step parameters. Default 3° tilt (operator can
# override via /step_id/cmd_tilt_magnitude) for 1.0 s gives the ball
# enough time to break stiction (typically 200-400 ms after
# commanded tilt is reached) AND accumulate measurable acceleration
# in the fit window. The earlier 0.5 s window was too short — when
# stiction held the ball for 300 ms, the fit only saw 50 ms of real
# motion and reported a falsely small G_eff.
STEP_ID_OPEN_LOOP_TILT_DEG    = 3.0
STEP_ID_OPEN_LOOP_DURATION_S  = 1.0
STEP_ID_OPEN_LOOP_RAMP_S      = 0.15   # ramp-in/ramp-out so the
                                       # platform doesn't overshoot
                                       # the commanded tilt
STEP_ID_OPEN_LOOP_MOTION_ONSET_MM = 3.0  # ball-displacement threshold
                                         # to declare "the ball has
                                         # started moving." The fit
                                         # window starts here when
                                         # this is later than the
                                         # ramp-end (handles late
                                         # stiction breakthrough).
STEP_ID_OPEN_LOOP_VPEAK_LIMIT_MM_S = 1500.0  # safety: if the ball
                                              # hits this SUSTAINED
                                              # velocity during tilt,
                                              # end early so it
                                              # doesn't escape. Raised
                                              # from 350 → 1500 after
                                              # 211307Z showed single-
                                              # frame vision-noise
                                              # spikes (1234 mm/s on
                                              # rep 2) firing the
                                              # cutoff inconsistently
                                              # across replicates,
                                              # producing 51% CV
                                              # G_eff. Foam ring
                                              # handles real escape
                                              # cases.
STEP_ID_OPEN_LOOP_VPEAK_CONSENSUS_N = 3      # require this many of
                                              # the last 5 velocity
                                              # samples to exceed the
                                              # limit before cutting.
                                              # Filters single-frame
                                              # vision glitches that
                                              # were tripping the
                                              # safety cutoff
                                              # inconsistently.
STEP_ID_PRE_TILT_LEVEL_S      = 1.0    # hold flat for this long before
                                       # tilting, so the parabola fit
                                       # has a clean t=0 baseline
STEP_ID_POST_TILT_LEVEL_S     = 1.0    # hold flat after tilting so the
                                       # ball decelerates within the bag
# Number of open-loop tilt-step trials to run for plant identification.
# Default 3 — single-trial plant ID was the #1 weakness called out in
# the post-202650Z critique: across four consecutive sessions on the
# same hardware, single-trial G_eff varied 35 → 224 → 92 → 64 (5×
# spread). With n=3 we get a sample mean ± std and can refuse to
# recommend gains when the cv exceeds STEP_ID_OL_CV_THRESHOLD —
# clear "plant is too noisy / stiction is too unpredictable for
# analytic tuning" failure mode rather than shipping a bad number.
# Override at runtime via /step_id/cmd_n_replicates (Float64; nearest
# integer). Operator can dial down to 1 for fast iteration on
# methodology, or up to 5 for high-confidence runs.
STEP_ID_OL_REPLICATES_DEFAULT = 3
STEP_ID_OL_CV_THRESHOLD       = 0.30   # coefficient of variation
                                       # (std/mean) above which we
                                       # refuse to recommend gains

# Closed-loop verification parameters
STEP_ID_GOTO_TIMEOUT_S        = 25.0   # per-trial goto timeout
STEP_ID_TARGET_OCCLUSION_S    = 1.0    # target marker continuously
                                       # occluded for this long → settled
STEP_ID_INTER_TRIAL_SETTLE_S  = 3.0    # after a goto trial ends,
                                       # explicitly hold the platform
                                       # flat at start_z for this long
                                       # so the operator can place the
                                       # ball on the next marker
                                       # without it rolling off due to
                                       # residual tilt from the prior
                                       # trial. Required because mode:
                                       # LEVEL_HOLD alone doesn't zero
                                       # current_rpy — it just stops
                                       # BALL_TRACK; the level loop
                                       # then tracks whatever rpy was
                                       # last commanded.
STEP_ID_INTER_TRIAL_DWELL_S   = 3.0    # dwell after operator confirms
                                       # ball on next-start marker
STEP_ID_OPERATOR_PROMPT_S     = 300.0  # max wait for operator to place
                                       # ball on the expected marker
                                       # (5 min). Session ends gracefully
                                       # if exceeded.

# Cross-sensor gating: the ball is "on marker M" iff
#   (a) marker M is missing from /platform_pose/marker_ids for ≥ this
#       long continuously, AND
#   (b) the ball state position is within this many mm of marker M's
#       center (independent confirmation from the KF).
STEP_ID_OCCLUSION_DWELL_S     = 0.5
STEP_ID_BALL_ON_MARKER_TOL_MM = 30.0

# Analytic gain calculation
STEP_ID_TARGET_ZETA           = 0.7    # damping ratio (mild overshoot)
STEP_ID_FALLBACK_OMEGA_N      = 3.0    # rad/s, used when Td_observed
                                       # can't be measured (e.g., ball
                                       # didn't move enough)
STEP_ID_OMEGA_N_CAP           = 5.0    # rad/s, hard ceiling regardless
                                       # of dead-time
# Vision + control + level-PI cascade dead-time. Fixed at 120 ms based
# on observation of the actual loop (vision capture + USB + KF +
# control tick + IK + leg response). Used in the bandwidth-limit
# calculation ωn_target = min(0.5/Td, OMEGA_N_CAP). Earlier code
# used the motion-onset time as Td, which conflated stiction
# breakthrough (~500 ms on this hardware) with control delay (~120
# ms) — recommendation came out absurdly slow because 0.5/0.658 =
# 0.76 rad/s. Using the right Td keeps the bandwidth target
# reasonable.
STEP_ID_LATENCY_TD_S          = 0.12

# Initial-acceleration window in the closed-loop step response. We fit
# G_eff from the first this-many ms after the goto command, before the
# Kd term takes over (controller's reaction lags the position response
# by Kd's pole, so the leading edge is ≈ open-loop ẍ = G·θ_cmd).
STEP_ID_INIT_ACCEL_WINDOW_S   = 0.20

# Marker-pair sequence offsets (relative to the operator-chosen start
# marker N). All ≤ +2 so straight-line ball paths stay ≥ 85 mm from
# the platform centre — never over the bolt.
STEP_ID_MARKER_OFFSETS = [
    (+0, +1),   # adjacent, ~92 mm — clean step-response (no saturation)
    (+1, +3),   # two-apart, ~170 mm — bandwidth check (mild saturation)
    (+3, +4),   # adjacent, ~92 mm — repeat clean read on different axis
    (+4, +6),   # two-apart, ~170 mm — repeat bandwidth check
]

STEP_ID_BAG_TOPICS = [
    '/ball_state',
    '/ball_ref',
    '/ball_xy_mono',
    '/platform_pose',
    '/platform_pose/marker_ids',
    '/platform_pose/markers_visible',
    '/platform_rpy',
    '/platform/imu/data',
    '/control_cmd',
    '/control_result',
    '/auto_tune/status',
    '/auto_tune/trial',
    '/ball_track/diagnostic',  # 50 Hz controller phase + cmd tilts;
                               # used by digest's tilt_timeseries.png
                               # to visualise saturation, stiction
                               # relief events, and inner-loop
                               # tracking quality
    '/oak/config',             # JSON snapshot @ ~5 Hz: v0_backend,
                               # v1_arch, blob filenames, exposure,
                               # focus, jpeg_fps. Tags every session
                               # with which detector was running so
                               # apples-to-apples comparison across
                               # sessions actually works.
    '/oak/health',             # per-stream Hz + per-path latency
                               # @ 5 Hz. Diagnoses whether the
                               # vision pipeline is keeping up
                               # during the session.
    '/oak/ball/v0/yolo_pixel',
    '/oak/ball/v0/cv2_pixel',
    '/leg_encoders',     # kept: stuck-leg detection during open-loop tilt
    '/leg_currents',     # kept: same
    '/status',
]


def _load_marker_layout() -> list[tuple[int, float, float]]:
    """Read marker_layout.yaml from the stewart_vision package share dir.
    Returns list of (id, x_mm, y_mm) sorted by id. Falls back to the
    source-tree copy if the share dir is missing (dev mode).

    Coordinates are in PLATFORM frame (the natural frame for marker
    geometry). Use _rotate_platform_to_imu() to transform into IMU
    frame for cross-sensor comparison against /ball_state."""
    candidates = []
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        candidates.append(os.path.join(
            get_package_share_directory('stewart_vision'),
            'config', 'marker_layout.yaml'))
    except Exception:
        pass
    candidates.append(os.path.expanduser(
        '~/stable_bot_repo/stewart_vision/config/marker_layout.yaml'))
    for path in candidates:
        if os.path.isfile(path):
            with open(path) as f:
                d = yaml.safe_load(f)
            out = []
            for m in d.get('markers', []):
                out.append((int(m['id']),
                            float(m['x_mm']),
                            float(m['y_mm'])))
            out.sort(key=lambda r: r[0])
            return out
    return []


def _load_aruco_imu_alignment() -> Optional[list[list[float]]]:
    """Read the 2x2 rotation that takes platform-frame xy → IMU-frame xy.
    Same file ball_localizer_node consumes — keeping STEP_ID's
    cross-sensor checks in IMU frame consistent with /ball_state.

    Returns the 2x2 matrix as a list of two-row lists, or None if
    the file is missing (in which case STEP_ID falls back to an
    identity rotation — equivalent to assuming platform == IMU
    frame, which is wrong on this hardware but lets the session
    proceed with looser position tolerance)."""
    candidates = []
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        candidates.append(os.path.join(
            get_package_share_directory('stewart_vision'),
            'config', 'aruco_imu_alignment.yaml'))
    except Exception:
        pass
    candidates.append(os.path.expanduser(
        '~/stable_bot_repo/stewart_vision/config/'
        'aruco_imu_alignment.yaml'))
    for path in candidates:
        if os.path.isfile(path):
            try:
                with open(path) as f:
                    d = yaml.safe_load(f) or {}
                m = d.get('matrix')
                if (isinstance(m, list) and len(m) == 2
                        and all(isinstance(r, list) and len(r) == 2
                                for r in m)):
                    return [[float(x) for x in r] for r in m]
            except Exception:
                continue
    return None


def _rotate_platform_to_imu(xy: tuple[float, float],
                            R: Optional[list[list[float]]]
                            ) -> tuple[float, float]:
    """Apply the platform→IMU rotation. R None → identity passthrough."""
    if R is None:
        return xy
    x, y = xy
    return (R[0][0] * x + R[0][1] * y,
            R[1][0] * x + R[1][1] * y)


# Fitness weights (sum should equal 1.0).
# Operator priority (2026-04-30): "as close to target for as long as
# possible." f_hold (continuous closeness × time, see CLOSENESS_SCALE_MM)
# is the dominant signal. f_err and f_p95 are kept for outlier-bound
# pressure (a trial that average-closes well but has 1s of orbit-out
# should still be penalized). f_settle credits "got there fast." f_calm
# is a small tiebreaker — closeness already implies non-orbiting, so
# velocity is mostly redundant.
W_ERR    = 0.20
W_P95    = 0.10
W_SETTLE = 0.10
W_HOLD   = 0.50
W_CALM   = 0.10


# ----- Helpers ---------------------------------------------------------------

def _now_utc_compact() -> str:
    return time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())


def _gains_yaml_path() -> str:
    """Resolve the canonical ball_track_gains.yaml location. Tries the
    installed share dir first (so changes by ball_track_save_gains are
    picked up), then the source repo as a dev fallback."""
    try:
        from ament_index_python.packages import (
            get_package_share_directory)
        share = get_package_share_directory('stewart_bringup')
        p = os.path.join(share, 'config', 'ball_track_gains.yaml')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.expanduser(
        '~/stable_bot_repo/stewart_bringup/config/ball_track_gains.yaml')


def _load_gains_yaml() -> dict:
    path = _gains_yaml_path()
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    return d


# ----- Trial state ----------------------------------------------------------

@dataclass
class TrialSample:
    t: float           # ROS time, seconds
    x: float           # ball x, mm
    y: float           # ball y, mm


@dataclass
class TrialResult:
    target: tuple[float, float]
    ball_start: tuple[float, float]
    samples: list[TrialSample]
    duration_s: float
    settled: bool
    settling_time_s: float
    aborted: bool = False
    abort_reason: str = ''


# ----- The node -------------------------------------------------------------

class AutoTuneNode(Node):
    def __init__(self):
        super().__init__('auto_tune')

        # state
        self._ball_state: Optional[PoseStamped] = None
        self._control_result_inbox: list[str] = []
        self._running = False
        self._abort = False
        self._tune_thread = None
        # Sign-knob master switch. Default ON (sign flips disabled),
        # because for any setup that's been direction-tested once
        # (which Stable-Bot has — see ball_track_gains.yaml comments),
        # touching pitch_sign / roll_sign during tuning is pure
        # downside: best case it confirms what you already know;
        # worst case it briefly inverts the controller and the ball
        # escapes. GUI checkbox toggles this live; env var sets the
        # boot default. Set DISABLE_SIGN_FLIPS=0 only on fresh
        # hardware that hasn't been direction-tested yet.
        self._disable_sign_flips = (
            os.environ.get('DISABLE_SIGN_FLIPS', '1') == '1')

        # ROS i/o
        self.create_subscription(
            PoseStamped, '/ball_state', self._on_ball_state, 10)
        self.create_subscription(
            String, '/control_result', self._on_control_result, 50)
        # STEP_ID needs the per-frame list of detected marker IDs to
        # decide which marker the ball is occluding. Latest-only state.
        self._marker_ids: list[int] = []
        self._marker_ids_t: float = 0.0
        self.create_subscription(
            Int32MultiArray, '/platform_pose/marker_ids',
            self._on_marker_ids, 10)
        # /status snapshot for arming + level-enabled pre-flight check.
        # Refuse to start STEP_ID if the system isn't armed — otherwise
        # the open-loop tilt does nothing and G_eff comes out as ~0
        # (observed 2026-05-01 17:34:55Z session: G_eff=3.87 mm/s²/°,
        # ball moved <1 mm during the entire 0.5 s tilt, recommended
        # gains came out useless because the platform never physically
        # responded to the SetPose).
        self._status_snapshot: dict = {}
        self._status_t: float = 0.0
        self.create_subscription(
            String, '/status', self._on_status, 10)
        # STEP_ID session state. None when not running.
        self._step_id_running = False
        self._step_id_thread = None
        self._step_id_operator_confirmed = False
        self._step_id_marker_layout = _load_marker_layout()
        # Platform→IMU rotation for cross-sensor position checks. The
        # /ball_state topic is in IMU frame (post the §0 fix in
        # ref_generator + ball_localizer); marker_layout is in
        # platform frame. We rotate marker centres into IMU frame
        # at session start so the distance-to-marker check actually
        # works. None → identity passthrough (warning logged).
        self._step_id_R_platform_to_imu = _load_aruco_imu_alignment()
        if self._step_id_R_platform_to_imu is None:
            self.get_logger().warn(
                'aruco_imu_alignment.yaml not loaded — STEP_ID '
                'cross-sensor position check will use identity '
                'rotation (looser tolerance).')
        # Operator-set start height for the STEP_ID session. Drives
        # the platform Z used by both the open-loop tilt trial and
        # the inter-trial pose reset. Default 80 mm: per operator
        # observation 2026-05-01, this Z keeps the legs "tight enough"
        # that disarm sag is small while still being inside the
        # platform's natural operating envelope. GUI publishes the
        # Float64 as the operator changes the input; we read the
        # latest value at session start.
        self._step_id_start_z_mm: float = 80.0
        self.create_subscription(
            Float64, '/step_id/cmd_start_z',
            self._on_step_id_start_z_cmd, 1)
        # Operator-set tilt magnitude for the open-loop trial. Default
        # 3° matches the bang-bang accel_tilt_deg — empirically the
        # smallest tilt that reliably overcomes foam-on-vinyl static
        # friction. The previous default 1° was below the stiction
        # breakaway threshold; the ball stayed put even though the
        # platform did tilt, producing G_eff fits well below the
        # physical floor.
        self._step_id_tilt_magnitude_deg: float = 3.0
        self.create_subscription(
            Float64, '/step_id/cmd_tilt_magnitude',
            self._on_step_id_tilt_magnitude_cmd, 1)
        # Operator-set number of open-loop replicates for plant ID.
        # See STEP_ID_OL_REPLICATES_DEFAULT in the constants block.
        self._step_id_n_replicates: int = (
            STEP_ID_OL_REPLICATES_DEFAULT)
        self.create_subscription(
            Float64, '/step_id/cmd_n_replicates',
            self._on_step_id_n_replicates_cmd, 1)
        # Latest IMU rpy (deg), so the open-loop trial can verify the
        # platform actually achieved the commanded tilt vs failing
        # silently. /platform_rpy is the IMU body roll/pitch/yaw at
        # ~20 Hz. We diagnose post-trial: if achieved < 50% of
        # commanded → platform didn't tilt (arming/level issue);
        # if achieved ≈ commanded but ball didn't move → static
        # friction holding the ball, need a larger tilt.
        self._step_id_imu_rpy: Optional[tuple[float, float]] = None
        self._step_id_imu_rpy_t: float = 0.0
        self.create_subscription(
            Float32MultiArray, '/platform_rpy',
            self._on_platform_rpy, 10)
        # Latest /ball_track/diagnostic snapshot — populated only
        # while BALL_TRACK is running. 11-element Float32MultiArray
        # per stewart_control_node._ball_track_run:
        #   [0] t_rel_s          (loop-relative time)
        #   [1] phase_code       (0 settle, 1 accel, 2 coast,
        #                         3 brake, 4 stiction_break, 5 pid,
        #                         -1 stale-state branch)
        #   [2] tilt_pitch_cmd_deg
        #   [3] tilt_roll_cmd_deg
        #   [4] ex_mm
        #   [5] ey_mm
        #   [6] err_mag_mm
        #   [7] v_toward_mm_s
        #   [8] vel_mag_mm_s
        #   [9] ux               (unit error)
        #   [10] uy
        # Used by the verification trial sample loop to attach the
        # controller's commanded tilt + active phase to each ball-
        # state sample. Lets the digest plot commanded-vs-achieved
        # tilt over time (saturation visible) and a phase strip
        # (when did stiction relief fire?).
        self._step_id_ball_track_diag: Optional[list[float]] = None
        self._step_id_ball_track_diag_t: float = 0.0
        self.create_subscription(
            Float32MultiArray, '/ball_track/diagnostic',
            self._on_ball_track_diag, 20)
        # Latest /oak/config snapshot (JSON String, ~5 Hz). Carries
        # v0_backend, v1_arch, exposure, etc. Captured into each
        # trial's output so the digest can label plots with which
        # detector was running and the comparison across sessions
        # is actually apples-to-apples.
        self._step_id_oak_config: Optional[dict] = None
        self._step_id_oak_config_t: float = 0.0
        self.create_subscription(
            String, '/oak/config',
            self._on_oak_config, 5)
        self.pub_cmd = self.create_publisher(String, '/control_cmd', 10)
        # Latched-style status: rosbridge clients pick up the latest
        # snapshot on connect (we'll re-publish it every status update).
        self.pub_status = self.create_publisher(
            String, '/auto_tune/status', 10)

        self.create_service(
            Trigger, '/auto_tune/start', self._srv_start)
        self.create_service(
            Trigger, '/auto_tune/stop', self._srv_stop)
        # STEP_ID — analytic-gains alternative to Twiddle. Shares the
        # /auto_tune/stop service (same kill behaviour). Status events
        # land on /auto_tune/status (state field includes 'step_id_*'
        # values) so the GUI gets one unified view of "what's the
        # tuner doing right now."
        self.create_service(
            Trigger, '/step_id/start', self._srv_step_id_start)
        # Operator handshake for the hard-pause inter-trial gate
        # (see STEP_ID_OPERATOR_PROMPT_S in the constants block).
        # When the session is waiting for the ball to be placed on
        # marker N, the GUI shows "place ball on marker N" + a Confirm
        # button; that button calls this service.
        self.create_service(
            Trigger, '/step_id/operator_confirm',
            self._srv_step_id_operator_confirm)
        # Walks tuning_data/auto_tune_*/summary.json, picks the
        # session with the highest best_fitness, applies its gains.
        # Useful after a rough session: "go back to whatever worked
        # best in any past run."
        self.create_service(
            Trigger, '/auto_tune/restore_best', self._do_restore_best)

        # GUI checkbox publishes a Bool here. True = sign flips
        # disabled (default); False = include sign knobs in the
        # Twiddle rotation. Takes effect on next knob pick.
        self.create_subscription(
            Bool, '/auto_tune/cmd_disable_sign_flips',
            self._on_disable_sign_flips_cmd, 1)

        # Per-trial event topic — GUI accumulates these into a history
        # log. Distinct from /auto_tune/status (which is the rolling
        # snapshot); each trial publishes here exactly once after its
        # fitness is computed.
        self.pub_trial = self.create_publisher(
            String, '/auto_tune/trial', 50)

        # SetPose client for the periodic Z reset that defeats
        # accumulated tilt-induced Z drift. None if the interfaces
        # package is missing — we just skip the reset and warn.
        if HAVE_SET_POSE:
            self._set_pose_cli = self.create_client(
                SetPose, '/set_pose')
        else:
            self._set_pose_cli = None
            self.get_logger().warn(
                'jugglebot_interfaces.SetPose not available — '
                'skipping periodic Z reset; expect Z drift on long runs')

        # status snapshot
        self._status = {
            'state': 'idle',
            'trial': 0,
            'max_trials': 0,
            'best_fitness': 0.0,
            'best_gains': {},
            'last_trial_fitness': 0.0,
            'last_step': '',
            'started_at': '',
            'log_dir': '',
            'ball_source': '',
        }
        self._publish_status()

        self.get_logger().info(
            'auto_tune ready — call /auto_tune/start to begin')

    # ----- ROS callbacks ----------------------------------------------------

    def _on_ball_state(self, msg: PoseStamped) -> None:
        self._ball_state = msg

    def _on_control_result(self, msg: String) -> None:
        # /control_result is JSON-or-flat-text; we just stash the last
        # 50 raw payloads so a trial step can scan for its handshake.
        self._control_result_inbox.append(msg.data)
        if len(self._control_result_inbox) > 50:
            self._control_result_inbox.pop(0)

    def _srv_start(self, req, res):
        if self._running:
            res.success = False
            res.message = 'auto_tune already running'
            return res
        self._abort = False
        import threading
        self._tune_thread = threading.Thread(
            target=self._run_tuning_session, daemon=True)
        self._tune_thread.start()
        res.success = True
        res.message = 'auto_tune started'
        return res

    def _on_disable_sign_flips_cmd(self, msg: Bool) -> None:
        """GUI checkbox handler. Live update — takes effect on next
        knob pick (existing trial finishes its current step)."""
        old = self._disable_sign_flips
        self._disable_sign_flips = bool(msg.data)
        if old != self._disable_sign_flips:
            self.get_logger().info(
                f"sign-flip toggle → "
                f"{'disabled' if self._disable_sign_flips else 'enabled'}")
            self._publish_status()

    def _srv_stop(self, req, res):
        if not self._running and not self._step_id_running:
            res.success = False
            res.message = 'auto_tune not running'
            return res
        # Set abort and publish ONE quick LEVEL_HOLD so any in-flight
        # ball-track tilt is immediately superseded. The full
        # platform-shutdown sequence (multi-publish + SetPose +
        # sleeps to defend against the BALL_TRACK_GOTO/LEVEL_HOLD
        # ordering race) happens in the session thread's finally
        # clause via _stop_platform_definitively. Doing the long
        # sequence here would block the service handler.
        self._abort = True
        self._publish_mode('LEVEL_HOLD')
        res.success = True
        res.message = ('auto_tune stopping — leveling platform '
                       '(takes ~1 s)')
        return res

    # ----- Trial loop -------------------------------------------------------

    def _run_tuning_session(self, max_trials: int = 50) -> None:
        self._running = True
        bag_proc = None
        try:
            log_dir = os.path.expanduser(
                f'~/stable_bot_repo/tuning_data/auto_tune_{_now_utc_compact()}')
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, 'log.jsonl')
            summary_path = os.path.join(log_dir, 'summary.json')
            self.get_logger().info(f'auto_tune log → {log_dir}')

            # Spawn the bag recorder. /auto_tune/trial events tagged in
            # the bag itself become natural subsection markers — replay
            # tools can split on those timestamps. Same MCAP format as
            # the IVA / vision-debug bags.
            bag_dir = os.path.join(log_dir, 'bag')
            bag_proc = self._start_bag(bag_dir)

            self._status.update({
                'state': 'running',
                'trial': 0,
                'max_trials': max_trials,
                'started_at': time.strftime(
                    '%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'log_dir': log_dir,
                'best_fitness': 0.0,
            })
            self._publish_status()

            gains = _load_gains_yaml()
            steps = {k.name: k.step for k in KNOBS}
            # Twiddle state: per knob, what's the next direction to
            # try? 'plus' → next perturbation is +step. 'minus' →
            # +step already failed, try -step next. 'next' → both
            # failed, shrink and move on. Continuous knobs only;
            # categorical signs use the existing sign-lock mechanism.
            twiddle_state: dict[str, str] = {
                k.name: 'plus' for k in KNOBS if not k.categorical}

            # Evaluate the starting point first.
            self.get_logger().info(
                'Trial 0 (baseline) — evaluating current gains.yaml')
            r0 = self._run_trial(gains)
            f0, c0 = self._compute_fitness(r0)
            self._append_trial_log(log_path, 0, gains, r0, f0, c0,
                                   step_taken='baseline', algo='baseline')
            best_gains = dict(gains)
            best_fitness = f0
            self._status.update({
                'trial': 0,
                'best_fitness': f0,
                'best_gains': dict(gains),
                'last_trial_fitness': f0,
                'last_step': 'baseline',
            })
            self._publish_status()
            self._update_summary(summary_path, 1, best_fitness, best_gains, 0)

            consec_bad = 0
            consec_rejections = 0   # for lucky-baseline re-eval
            t_start = time.time()
            knob_idx = 0
            sign_locked: dict[str, bool] = {
                k.name: False for k in KNOBS if k.categorical}
            sign_reject_count: dict[str, int] = {
                k.name: 0 for k in KNOBS if k.categorical}
            for trial in range(1, max_trials + 1):
                if self._abort:
                    self.get_logger().info('auto_tune aborted by user')
                    break

                # Periodic Z reset so accumulated tilt-induced drift
                # doesn't turn the platform into a bowl pulling the
                # ball to the center bolt. Blocking + 5s dwell so the
                # platform actually reaches Z=NOMINAL and fully levels
                # before the next trial starts.
                if (RESET_Z_EVERY_N_TRIALS > 0
                        and (trial - 1) % RESET_Z_EVERY_N_TRIALS == 0):
                    self.get_logger().info(
                        f'  Z reset → {NOMINAL_Z_MM:.0f} mm '
                        f'+ {RESET_LEVEL_DWELL_S:.0f} s level')
                    # Cut any in-flight controls before the pose move.
                    self._publish_mode('LEVEL_HOLD')
                    self._sleep_with_abort(0.3)
                    # Blocking SetPose call so we don't move on until
                    # the platform reaches the commanded pose. Cap at
                    # 4 s in case the service hangs.
                    self._reset_pose_to_nominal(blocking=True)
                    self._sleep_with_abort(RESET_LEVEL_DWELL_S)
                    if self._abort:
                        break

                # Twiddle knob picker: round-robin through unlocked
                # continuous knobs. Sign knobs skipped entirely when
                # _disable_sign_flips is True (default — see init).
                for _ in range(len(KNOBS) * 2):
                    kn = KNOBS[knob_idx % len(KNOBS)]
                    knob_idx += 1
                    if kn.categorical:
                        if self._disable_sign_flips:
                            continue
                        if sign_locked.get(kn.name):
                            continue
                    break

                # Build candidate gains based on the knob's state.
                if kn.categorical:
                    # Sign knobs: try a flip. Sign-lock mechanism
                    # remains the same.
                    test_gains = dict(best_gains)
                    test_gains[kn.name] = (
                        -1.0 if best_gains.get(kn.name, 1.0) > 0 else 1.0)
                    step_repr = f'{kn.name} flip → {test_gains[kn.name]:+.0f}'
                    direction = 'flip'
                else:
                    state = twiddle_state[kn.name]
                    cur = float(best_gains.get(kn.name, 0.0))
                    if state == 'plus':
                        delta = +steps[kn.name]
                        direction = 'plus'
                    else:  # 'minus'
                        delta = -steps[kn.name]
                        direction = 'minus'
                    new = max(kn.lo, min(kn.hi, cur + delta))
                    test_gains = dict(best_gains)
                    test_gains[kn.name] = new
                    step_repr = (f'{kn.name} {cur:.4f} → {new:.4f} '
                                 f'(Δ={delta:+.4f}, dir={direction})')

                self.get_logger().info(
                    f'Trial {trial}/{max_trials}: {step_repr}')
                r = self._run_trial(test_gains)
                if r.aborted:
                    self.get_logger().warn(
                        f'  trial aborted: {r.abort_reason} — skipping')
                    continue
                f, c = self._compute_fitness(r)
                # Direct comparison — NO noise floor. Twiddle's
                # step adaptation handles noise: a noisy reject
                # just moves on to the next knob; we'll revisit
                # this knob later at a different step size where
                # the signal/noise ratio may differ. Real
                # improvements (even small ones) get recorded.
                accepted = f > best_fitness
                if accepted:
                    best_fitness = f
                    best_gains = dict(test_gains)
                    consec_rejections = 0
                    if not kn.categorical:
                        # Twiddle "grow on success": expand the step
                        # so subsequent moves in this direction can
                        # cover more ground per trial.
                        steps[kn.name] *= TWIDDLE_GROW
                        # Reset state to 'plus' so the next visit to
                        # this knob continues exploring the same
                        # productive direction first.
                        twiddle_state[kn.name] = ('plus' if direction == 'plus'
                                                  else 'minus')
                        self.get_logger().info(
                            f'  ✓ accepted; step expanded: '
                            f'{kn.name} step → {steps[kn.name]:.4f}')
                else:
                    consec_rejections += 1
                    if not kn.categorical:
                        # Twiddle state advance.
                        if twiddle_state[kn.name] == 'plus':
                            # +step failed; queue -step for next visit.
                            twiddle_state[kn.name] = 'minus'
                            self.get_logger().info(
                                f'  ✗ rejected (+); will try - next visit')
                        else:
                            # Both directions failed — shrink and reset.
                            steps[kn.name] *= TWIDDLE_SHRINK
                            twiddle_state[kn.name] = 'plus'
                            self.get_logger().info(
                                f'  ✗ rejected (-); both dirs failed — '
                                f'step shrunk: {kn.name} → '
                                f'{steps[kn.name]:.4f}')
                    # Sign-knob lock-after-N-rejects (unchanged).
                    if kn.categorical:
                        sign_reject_count[kn.name] += 1
                        if (sign_reject_count[kn.name]
                                >= SIGN_FLIP_LOCK_AFTER_REJECTS):
                            sign_locked[kn.name] = True
                            self.get_logger().info(
                                f'  {kn.name} locked at '
                                f'{best_gains.get(kn.name, 0):+.0f} '
                                f'after {sign_reject_count[kn.name]} '
                                f'rejected flip(s)')
                self._append_trial_log(
                    log_path, trial, test_gains, r, f, c,
                    step_taken=f'{step_repr} ({"accepted" if accepted else "rejected"})',
                    algo='twiddle')

                # Safety: 5 consecutive bad trials → halt.
                if f < SAFETY_BAD_THRESH:
                    consec_bad += 1
                    if consec_bad >= SAFETY_BAD_TRIALS:
                        self.get_logger().error(
                            f'{SAFETY_BAD_TRIALS} consecutive trials with '
                            f'fitness < {SAFETY_BAD_THRESH:.2f} — halting. '
                            f'Controller may be broken (sign flip, ball '
                            f'escape).')
                        break
                else:
                    consec_bad = 0

                self._status.update({
                    'trial': trial,
                    'best_fitness': best_fitness,
                    'best_gains': dict(best_gains),
                    'last_trial_fitness': f,
                    'last_step': step_repr,
                })
                self._publish_status()
                self._update_summary(summary_path, trial + 1,
                                     best_fitness, best_gains,
                                     int(time.time() - t_start))

                # Lucky-baseline guard. If the running best hasn't
                # been beaten in REEVAL_AFTER_REJECTIONS trials, it
                # may have been a noisy outlier. Re-evaluate the
                # same gains; weighted-average the two samples so
                # the recorded fitness regresses toward the truth.
                if consec_rejections >= REEVAL_AFTER_REJECTIONS:
                    self.get_logger().info(
                        f'  re-evaluating current best after '
                        f'{consec_rejections} rejections')
                    rr = self._run_trial(best_gains)
                    if not rr.aborted and rr.samples:
                        f_re, c_re = self._compute_fitness(rr)
                        new_best = (REEVAL_WEIGHT_NEW * f_re +
                                    (1.0 - REEVAL_WEIGHT_NEW)
                                    * best_fitness)
                        self.get_logger().info(
                            f'  reeval f={f_re:.3f}; best '
                            f'{best_fitness:.3f} → {new_best:.3f}')
                        best_fitness = new_best
                        self._append_trial_log(
                            log_path, trial, best_gains, rr, f_re, c_re,
                            step_taken=(f'reeval (best was '
                                        f'{best_fitness:.3f}, now '
                                        f'{new_best:.3f})'),
                            algo='reeval')
                    consec_rejections = 0

            # Save best gains back to the yaml so the controller picks
            # them up after the session ends.
            self._apply_gains(best_gains)
            self.get_logger().info(
                f'auto_tune done. best_fitness={best_fitness:.3f} '
                f'best_gains={best_gains}')
            self._status['state'] = 'aborted' if self._abort else 'done'
            self._publish_status()
        except Exception as e:
            self.get_logger().error(f'auto_tune crashed: {e!r}')
            self._status['state'] = 'crashed'
            self._publish_status()
        finally:
            # Always return the platform to LEVEL_HOLD when the session
            # ends — done, aborted, or crashed. This guarantees the
            # controller is no longer commanding tilts after Stop.
            try:
                self._publish_mode('LEVEL_HOLD')
                self._reset_pose_to_nominal(blocking=False)
            except Exception:
                pass
            # Stop the bag recorder cleanly so the mcap finalizes.
            if bag_proc is not None:
                self._stop_bag(bag_proc)
            self._running = False

    def _run_trial(self, gains: dict) -> TrialResult:
        # 1. Apply gains
        if not self._apply_gains(gains):
            return TrialResult(
                target=(0, 0), ball_start=(0, 0),
                samples=[], duration_s=0.0,
                settled=False, settling_time_s=0.0,
                aborted=True, abort_reason='gain save failed')
        # 2. LEVEL_HOLD until ball calms (low velocity for >0.5s).
        # Stops the next trial from starting on top of an orbit
        # left over from the previous one — operator observed
        # back-to-back trials inheriting velocity and amplifying.
        calmed = self._wait_for_calm()
        if self._abort:
            return TrialResult(target=(0, 0), ball_start=(0, 0),
                               samples=[], duration_s=0,
                               settled=False, settling_time_s=0,
                               aborted=True, abort_reason='abort during settle')
        if not calmed:
            self.get_logger().warn(
                f'  ball never calmed within {CALM_MAX_WAIT_S:.0f}s '
                f'— trial may be unfair, proceeding anyway')
        # 3. Current ball position
        ball_xy = self._latest_ball_xy()
        if ball_xy is None:
            return TrialResult(target=(0, 0), ball_start=(0, 0),
                               samples=[], duration_s=0,
                               settled=False, settling_time_s=0,
                               aborted=True, abort_reason='no /ball_state')
        # 4. Target at fixed distance
        target = self._pick_target(ball_xy)
        if target is None:
            return TrialResult(
                target=(0, 0), ball_start=ball_xy, samples=[],
                duration_s=0.0, settled=False, settling_time_s=0.0,
                aborted=True,
                abort_reason='no safe target found (ball at rim?)')
        # 5. Publish goto. Diagnostic log: confirms the math here is
        # exact. Picker is target = ball + D·(cos θ, sin θ), so the
        # actual_dist printout should always equal TRIAL_DISTANCE_MM
        # to within float-precision rounding. If it ever doesn't,
        # there's a bug to find.
        actual_dist = math.hypot(target[0] - ball_xy[0],
                                 target[1] - ball_xy[1])
        self.get_logger().info(
            f'  ball=({ball_xy[0]:+.1f}, {ball_xy[1]:+.1f}) → '
            f'target=({target[0]:+.1f}, {target[1]:+.1f})  '
            f'dist={actual_dist:.1f}mm '
            f'(expected {TRIAL_DISTANCE_MM:.0f})')
        self._publish_mode('BALL_TRACK_GOTO',
                           extra={'x_mm': target[0], 'y_mm': target[1]})
        # 6. Collect samples
        samples, settled, settling_t, dur = self._collect_trial(target)
        return TrialResult(
            target=target, ball_start=ball_xy, samples=samples,
            duration_s=dur, settled=settled, settling_time_s=settling_t)

    def _collect_trial(self, target: tuple[float, float]
                       ) -> tuple[list[TrialSample], bool, float, float]:
        samples: list[TrialSample] = []
        t_start = time.time()
        last_break_band_t: Optional[float] = None
        settled = False
        settling_t = 0.0
        while not self._abort:
            elapsed = time.time() - t_start
            if elapsed > TRIAL_TIMEOUT_S:
                break
            # Pull latest ball state (best-effort; sub callbacks land
            # on a separate executor thread when MultiThreadedExecutor
            # is used, but this node uses the default single-threaded
            # one, so we sleep to yield).
            if self._ball_state is not None:
                pos = self._ball_state.pose.position
                t_now = time.time() - t_start
                samples.append(TrialSample(t=t_now, x=pos.x, y=pos.y))
                err = math.hypot(pos.x - target[0], pos.y - target[1])
                if err > SETTLE_BAND_MM:
                    last_break_band_t = elapsed
                else:
                    held = elapsed - (last_break_band_t or 0.0)
                    if last_break_band_t is None:
                        # never been outside band → effectively at 0
                        held = elapsed
                    if held >= SETTLE_HOLD_S:
                        settled = True
                        settling_t = (
                            (last_break_band_t or 0.0) + SETTLE_HOLD_S)
                        break
            time.sleep(0.02)
        return samples, settled, settling_t, time.time() - t_start

    # ----- Fitness ----------------------------------------------------------

    def _compute_fitness(self, r: TrialResult) -> tuple[float, dict]:
        if not r.samples:
            return 0.0, {'f_err': 0, 'f_p95': 0, 'f_settle': 0,
                         'f_hold': 0, 'f_calm': 0}
        xs = np.array([s.x for s in r.samples])
        ys = np.array([s.y for s in r.samples])
        ts = np.array([s.t for s in r.samples])
        err = np.hypot(xs - r.target[0], ys - r.target[1])
        rms_err = float(np.sqrt(np.mean(err * err)))
        p95_err = float(np.percentile(err, 95))
        on_target_frac = float(np.mean(err < HOLD_TOL_MM))
        # Continuous time-weighted closeness reward. Per-sample reward
        # is exp(-err/scale); averaging across samples means a trial
        # that hovered at 5mm for 10s scores higher than one that
        # darted in to 2mm for 200ms then orbited at 50mm. This is
        # what the operator means by "close for as long as possible."
        closeness = np.exp(-err / CLOSENESS_SCALE_MM)
        mean_closeness = float(np.mean(closeness))
        # Speed via finite difference (no velocity in PoseStamped).
        if len(xs) > 2:
            dt = np.diff(ts)
            vx = np.diff(xs) / np.maximum(dt, 1e-3)
            vy = np.diff(ys) / np.maximum(dt, 1e-3)
            mean_speed = float(np.mean(np.hypot(vx, vy)))
        else:
            mean_speed = 0.0

        # Ball-stuck-on-center detector. If the ball spent most of the
        # trial near the center bolt (regardless of target), the trial
        # is degenerate — the controller couldn't move it off and the
        # operator has to physically un-stick it. Penalize hard.
        center_dist = np.hypot(xs, ys)
        stuck_frac = float(np.mean(center_dist < CENTER_EXCLUSION_MM))

        f_err    = 1.0 / (1.0 + rms_err / 50.0)
        f_p95    = 1.0 / (1.0 + p95_err / 100.0)
        f_settle = (max(0.0, 1.0 - r.settling_time_s / 15.0)
                    if r.settled else 0.0)
        f_hold   = mean_closeness
        f_calm   = 1.0 / (1.0 + mean_speed / 100.0)

        fitness = (W_ERR * f_err + W_P95 * f_p95 + W_SETTLE * f_settle
                   + W_HOLD * f_hold + W_CALM * f_calm)
        # Multiplicative penalty: any trial where the ball spent
        # >40% of samples inside the center exclusion zone gets its
        # fitness scaled by (1 - stuck_frac), which collapses the
        # score even if rms / hold accidentally look OK because the
        # target was placed close to center too.
        if stuck_frac > 0.4:
            fitness *= max(0.0, 1.0 - stuck_frac)
        components = {
            'f_err': f_err, 'f_p95': f_p95, 'f_settle': f_settle,
            'f_hold': f_hold, 'f_calm': f_calm,
            'rms_err_mm': rms_err, 'p95_err_mm': p95_err,
            'on_target_fraction': on_target_frac,
            'mean_closeness': mean_closeness,
            'mean_speed_mm_per_s': mean_speed,
            'stuck_on_center_fraction': stuck_frac,
        }
        return fitness, components

    # ----- Target picker ----------------------------------------------------

    def _pick_target(self, ball_xy: tuple[float, float]
                     ) -> Optional[tuple[float, float]]:
        """Pick a target at EXACTLY TRIAL_DISTANCE_MM from the ball
        that satisfies the safe-zone and center-exclusion constraints.

        Deterministic angle sweep (5° increments, randomized start)
        so the distance is always D — never shrunk. If the ball's
        position is incompatible with the constraints (e.g., parked
        next to the rim where every direction at D goes off-platform),
        returns None and the operator-visible trial is skipped.

        Earlier impl used random sampling with shrink-on-failure,
        which let some trials get D=42 while others got D=60 — the
        operator noticed varying difficulty across trials. Fixed by
        making D inviolate.
        """
        D = TRIAL_DISTANCE_MM
        safe_R = SAFE_RADIUS_FRAC * PLATFORM_R_MM
        thetas = [i * math.pi / 36 for i in range(72)]  # 5° step
        random.shuffle(thetas)
        for theta in thetas:
            tx = ball_xy[0] + D * math.cos(theta)
            ty = ball_xy[1] + D * math.sin(theta)
            r_target = math.hypot(tx, ty)
            if r_target > safe_R:
                continue
            if r_target < CENTER_EXCLUSION_MM:
                continue
            return (tx, ty)
        return None

    # ----- Control-cmd helpers ----------------------------------------------

    def _publish_mode(self, mode: str, extra: dict = None) -> None:
        msg = String()
        if extra:
            msg.data = f'mode:{mode} {json.dumps(extra)}'
        else:
            msg.data = f'mode:{mode}'
        self.pub_cmd.publish(msg)

    def _apply_gains(self, gains: dict) -> bool:
        # Send as a /control_cmd ball_track_save_gains JSON. The
        # control node validates + clamps + persists + reloads.
        clean = {k: float(v) for k, v in gains.items()
                 if isinstance(v, (int, float))}
        msg = String()
        msg.data = f'ball_track_save_gains:{json.dumps(clean)}'
        self.pub_cmd.publish(msg)
        # Brief settling delay; control node reloads within ~20 ms but
        # the publish→subscribe RTT can be a few ms.
        time.sleep(0.10)
        return True

    def _reset_pose_to_nominal(self, blocking: bool = False) -> bool:
        """Call /set_pose to reset the platform to nominal Z, level,
        zero yaw. Used between groups of trials to defeat the slow
        Z drift the user observed on long runs ("the platform turns
        into a bowl that pulls the ball toward the center bolt").
        Non-blocking by default so we don't stall the trial loop;
        the controller picks up the new pose on its next tick.
        """
        if self._set_pose_cli is None:
            return False
        if not self._set_pose_cli.service_is_ready():
            # The control node may take a moment to advertise.
            self._set_pose_cli.wait_for_service(timeout_sec=0.5)
            if not self._set_pose_cli.service_is_ready():
                return False
        req = SetPose.Request()
        req.x = 0.0
        req.y = 0.0
        req.z = float(NOMINAL_Z_MM)
        req.roll = 0.0
        req.pitch = 0.0
        req.yaw = 0.0
        req.blocking = bool(blocking)
        try:
            self._set_pose_cli.call_async(req)
            return True
        except Exception as e:
            self.get_logger().warn(f'set_pose call failed: {e!r}')
            return False

    def _latest_ball_xy(self) -> Optional[tuple[float, float]]:
        # Wait briefly if /ball_state hasn't arrived yet.
        for _ in range(20):
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                return (float(p.x), float(p.y))
            time.sleep(0.05)
        return None

    def _sleep_with_abort(self, seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end and not self._abort:
            time.sleep(0.05)

    def _wait_for_calm(self, max_wait_s: float = CALM_MAX_WAIT_S,
                       vel_threshold_mm_s: float = CALM_VEL_THRESHOLD_MM_S,
                       calm_dwell_s: float = CALM_DWELL_S,
                       min_wait_s: float = LEVEL_HOLD_BETWEEN_S) -> bool:
        """Hold LEVEL_HOLD until the ball's measured velocity drops
        below vel_threshold_mm_s for calm_dwell_s, or max_wait_s
        elapses. Min hold time is min_wait_s regardless — gives the
        platform itself a chance to settle even if /ball_state lies
        about velocity (e.g., dropped frames). Returns True if the
        ball actually calmed; False if we hit max_wait_s while still
        moving (orbit, escape, etc.)."""
        self._publish_mode('LEVEL_HOLD')
        t_start = time.time()
        last_xy = None
        last_t = None
        calm_start: Optional[float] = None
        while not self._abort:
            now = time.time()
            elapsed = now - t_start
            if elapsed >= max_wait_s:
                return False
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                xy = (float(p.x), float(p.y))
                if last_xy is not None and last_t is not None:
                    dt = max(now - last_t, 1e-3)
                    vel = math.hypot(xy[0] - last_xy[0],
                                     xy[1] - last_xy[1]) / dt
                    if vel < vel_threshold_mm_s:
                        if calm_start is None:
                            calm_start = now
                        elif (now - calm_start >= calm_dwell_s
                              and elapsed >= min_wait_s):
                            return True
                    else:
                        calm_start = None
                last_xy = xy
                last_t = now
            time.sleep(0.05)
        return False

    # ----- Logging ----------------------------------------------------------

    def _append_trial_log(self, log_path: str, trial: int,
                          gains: dict, r: TrialResult,
                          fitness: float, components: dict,
                          step_taken: str, algo: str) -> None:
        row = {
            'trial': trial,
            'ts': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'gains': {k: float(v) for k, v in gains.items()
                      if isinstance(v, (int, float))},
            'target': {'x_mm': r.target[0], 'y_mm': r.target[1]},
            'ball_start': {'x_mm': r.ball_start[0],
                           'y_mm': r.ball_start[1]},
            'fitness': float(fitness),
            'components': components,
            'settled': r.settled,
            'settling_time_s': r.settling_time_s,
            'duration_s': r.duration_s,
            'n_samples': len(r.samples),
            'aborted': r.aborted,
            'abort_reason': r.abort_reason,
            'algo': algo,
            'step_taken': step_taken,
        }
        with open(log_path, 'a') as f:
            f.write(json.dumps(row) + '\n')
        # Also publish to /auto_tune/trial so the GUI can accumulate a
        # live history without re-fetching the file.
        msg = String()
        msg.data = json.dumps(row)
        self.pub_trial.publish(msg)

    def _update_summary(self, summary_path: str, n_trials: int,
                        best_fitness: float, best_gains: dict,
                        elapsed_s: int) -> None:
        d = {
            'n_trials': n_trials,
            'best_fitness': float(best_fitness),
            'best_gains': {k: float(v) for k, v in best_gains.items()
                           if isinstance(v, (int, float))},
            'elapsed_s': int(elapsed_s),
        }
        with open(summary_path, 'w') as f:
            json.dump(d, f, indent=2)

    def _publish_status(self) -> None:
        # Always include the live config flags so the GUI checkbox
        # can reflect the running state (especially after reconnect).
        self._status['disable_sign_flips'] = bool(self._disable_sign_flips)
        msg = String()
        msg.data = json.dumps(self._status)
        self.pub_status.publish(msg)

    # ----- Bag recorder ----------------------------------------------------

    def _start_bag(self, out_dir: str):
        """Spawn `ros2 bag record` for the auto-tune topic set, mcap.
        Returns the Popen handle (None on failure). Same pattern as
        gui_server's vision-debug bag recorder."""
        try:
            os.makedirs(os.path.dirname(out_dir), exist_ok=True)
            cmd = (
                'source /opt/ros/kilted/setup.bash && '
                'source /home/sorak/ros2_ws/install/local_setup.bash && '
                f'exec ros2 bag record -s mcap -o {out_dir} '
                + ' '.join(AUTOTUNE_BAG_TOPICS)
            )
            proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            self.get_logger().info(f'auto_tune bag → {out_dir}')
            # Tiny pause so ros2 bag finishes opening writers before
            # the first /auto_tune/status hits.
            time.sleep(0.3)
            return proc
        except Exception as e:
            self.get_logger().warn(
                f'failed to spawn bag recorder: {e!r}')
            return None

    def _stop_bag(self, proc) -> None:
        """SIGTERM the bag recorder process group so the mcap
        finalizes cleanly (SIGKILL would leave the file unrecoverable
        unless mcap has a reindex tool, which it does, but cleaner
        to do it right)."""
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5.0)
            self.get_logger().info('auto_tune bag closed')
        except Exception as e:
            self.get_logger().warn(f'bag stop: {e!r}')

    # ----- STEP_ID mode ----------------------------------------------------
    #
    # Analytic-gains tuning, alternative to Twiddle. See the constants
    # block at the top of this file for design rationale. Flow:
    #
    #   1. operator places ball on any marker, presses Start
    #   2. STEP_ID detects which marker is being occluded → start_marker
    #   3. open-loop tilt-step trial: command 1° tilt for 0.5 s,
    #      record ball trajectory, fit parabola → G_eff (cascaded plant
    #      gain in mm/s² per degree of commanded tilt)
    #   4. compute Kp = ωn²/G_eff, Kd = 2ζωn/G_eff with ωn adaptively
    #      chosen based on observed dead-time, ζ = 0.7
    #   5. closed-loop verification: 4 marker-pair goto trials in a
    #      walk-around-the-ring sequence (offsets +1, +2, +1, +2), with
    #      hard inter-trial pauses prompting the operator to place the
    #      ball on the expected start marker if it didn't end up there
    #
    # All trials recorded into one mcap. digest_step_id_bag.py post-
    # processes for plots and refines G_eff from the closed-loop initial-
    # acceleration windows. Recommended gains never auto-applied; operator
    # promotes via GUI button (separate from this node).

    def _on_marker_ids(self, msg: Int32MultiArray) -> None:
        self._marker_ids = list(msg.data)
        self._marker_ids_t = time.monotonic()

    def _on_status(self, msg: String) -> None:
        try:
            self._status_snapshot = json.loads(msg.data)
            self._status_t = time.monotonic()
        except (json.JSONDecodeError, AttributeError):
            pass

    def _on_step_id_start_z_cmd(self, msg: Float64) -> None:
        """GUI updates the desired start height for STEP_ID sessions.
        Soft-clamped to a reasonable envelope; control node's set_pose
        also has its own limits, but this clamp gives the operator a
        more useful error message than 'out of envelope'."""
        z = float(msg.data)
        if z < 25.0 or z > 120.0:
            self.get_logger().warn(
                f'/step_id/cmd_start_z={z:.1f} mm outside reasonable '
                f'envelope [25, 120]; ignoring')
            return
        self._step_id_start_z_mm = z
        self.get_logger().info(
            f'STEP_ID start height set to {z:.1f} mm')

    def _on_step_id_tilt_magnitude_cmd(self, msg: Float64) -> None:
        """GUI updates the open-loop tilt magnitude for STEP_ID. Hard
        upper bound of 8° so a typo in the input can't blow past
        max_tilt and send the ball flying."""
        deg = float(msg.data)
        if deg < 0.5 or deg > 8.0:
            self.get_logger().warn(
                f'/step_id/cmd_tilt_magnitude={deg:.1f}° outside '
                f'reasonable envelope [0.5, 8.0]; ignoring')
            return
        self._step_id_tilt_magnitude_deg = deg
        self.get_logger().info(
            f'STEP_ID open-loop tilt magnitude set to {deg:.2f}°')

    def _on_platform_rpy(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 2:
            self._step_id_imu_rpy = (float(msg.data[0]),
                                      float(msg.data[1]))
            self._step_id_imu_rpy_t = time.monotonic()

    def _on_ball_track_diag(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 11:
            self._step_id_ball_track_diag = list(msg.data)
            self._step_id_ball_track_diag_t = time.monotonic()

    def _on_oak_config(self, msg: String) -> None:
        try:
            self._step_id_oak_config = json.loads(msg.data)
            self._step_id_oak_config_t = time.monotonic()
        except (json.JSONDecodeError, AttributeError):
            pass

    def _vision_backend_snapshot(self) -> dict:
        """Compact snapshot of the detector currently feeding the
        controller. Captured into each trial's output. Falls back
        to empty if /oak/config hasn't been received yet."""
        cfg = self._step_id_oak_config or {}
        return {
            'v0_backend': cfg.get('v0_backend'),
            'v1_arch': cfg.get('v1_arch'),
            'v0_blob': cfg.get('v0_blob'),
            'v1_blob': cfg.get('v1_blob'),
            'rgb_fps': cfg.get('rgb_fps'),
            'mono_fps': cfg.get('mono_fps'),
            'exp_us': cfg.get('exp_us'),
            'iso': cfg.get('iso'),
            'jpeg_fps': cfg.get('jpeg_fps'),
            'config_age_s': (
                time.monotonic() - self._step_id_oak_config_t
                if self._step_id_oak_config_t > 0 else None),
        }

    def _on_step_id_n_replicates_cmd(self, msg: Float64) -> None:
        """GUI sets the open-loop replicate count. Float64 because
        the existing /step_id/cmd_* topics are all Float64; we round
        to nearest int and clamp."""
        n = int(round(float(msg.data)))
        if n < 1 or n > 6:
            self.get_logger().warn(
                f'/step_id/cmd_n_replicates={n} outside [1, 6]; '
                f'ignoring')
            return
        self._step_id_n_replicates = n
        self.get_logger().info(
            f'STEP_ID open-loop replicate count set to {n}')

    def _check_preflight(self) -> tuple[bool, str]:
        """Pre-flight check before starting any tuning session.
        Returns (ok, message). Currently checks:
          - /status received recently (proves control_node alive)
          - 'armed': True (otherwise SetPose commands do nothing)
          - 'level_enabled': True (otherwise tilt commands aren't
            tracked by the level-PI loop and the open-loop fit will
            measure something other than the cascaded plant)
        """
        if (self._status_t == 0.0
                or time.monotonic() - self._status_t > 2.0):
            return (False,
                    'no recent /status from stewart_control — '
                    'is the control node running?')
        s = self._status_snapshot
        if not s.get('armed'):
            return (False,
                    'system not armed — STEP_ID needs the legs in '
                    'closed-loop position control to actually tilt. '
                    'Click ARM ALL in the GUI first.')
        if not s.get('level_enabled'):
            return (False,
                    'level loop disabled — STEP_ID measures the '
                    'cascaded plant (level-PI inner loop must be '
                    'tracking commanded tilt). Enable the level '
                    'loop first.')
        return (True, 'ok')

    def _srv_step_id_start(self, req, res):
        if self._running or self._step_id_running:
            res.success = False
            res.message = 'auto_tune already running'
            return res
        if not self._step_id_marker_layout:
            res.success = False
            res.message = 'marker_layout.yaml not loaded — STEP_ID needs it'
            return res
        # Pre-flight: arming + level loop. Refuse to start if the
        # legs aren't in CLOSED_LOOP — observed scenario where an
        # entire STEP_ID session ran with the platform unarmed,
        # G_eff came out as 3.87 mm/s²/° (vs ~120 expected) because
        # the platform never physically tilted, recommended gains
        # were 430× current and useless.
        ok, msg = self._check_preflight()
        if not ok:
            res.success = False
            res.message = f'preflight failed: {msg}'
            self.get_logger().warn(f'step_id refused start: {msg}')
            return res
        self._abort = False
        self._step_id_operator_confirmed = False
        import threading
        self._step_id_thread = threading.Thread(
            target=self._run_step_id_session, daemon=True)
        self._step_id_thread.start()
        res.success = True
        res.message = 'step_id started'
        return res

    def _srv_step_id_operator_confirm(self, req, res):
        """Operator clicked the Confirm button after placing the ball
        on the prompted marker. The session loop polls this flag."""
        self._step_id_operator_confirmed = True
        res.success = True
        res.message = 'operator confirmation noted'
        return res

    def _marker_xy(self, marker_id: int) -> Optional[tuple[float, float]]:
        """Marker centre in PLATFORM frame (the natural frame for
        goto targets — ref_generator handles the platform→IMU
        rotation downstream)."""
        for mid, x, y in self._step_id_marker_layout:
            if mid == marker_id:
                return (x, y)
        return None

    def _marker_xy_imu(self, marker_id: int
                       ) -> Optional[tuple[float, float]]:
        """Marker centre in IMU frame, for cross-sensor distance
        comparisons against /ball_state (which is in IMU frame
        after the §0 alignment fix)."""
        xy = self._marker_xy(marker_id)
        if xy is None:
            return None
        return _rotate_platform_to_imu(
            xy, self._step_id_R_platform_to_imu)

    def _detect_starting_marker(self, timeout_s: float = 60.0
                                ) -> tuple[Optional[int], str]:
        """Wait until exactly one marker ID is consistently missing
        from /platform_pose/marker_ids for STEP_ID_OCCLUSION_DWELL_S
        AND the ball state position is within
        STEP_ID_BALL_ON_MARKER_TOL_MM of that marker's IMU-frame centre.

        Returns (marker_id_or_None, reason). Reasons:
          - 'detected'   — marker found, normal completion
          - 'aborted'    — operator pressed Stop
          - 'timeout'    — timeout_s elapsed
          - 'no_topic'   — never received a /platform_pose/marker_ids
                           message (vision pipeline may be down or the
                           new build isn't running)

        Cross-sensor confirmation: the missing-ID test alone could be
        fooled by a transient occlusion (operator's hand, glare); the
        ball-position test alone could be fooled by KF drift while
        actually off-platform. Requiring both protects against either."""
        all_ids = {mid for mid, _, _ in self._step_id_marker_layout}
        t_start = time.monotonic()
        candidate: Optional[int] = None
        candidate_since: Optional[float] = None
        last_status_t = 0.0
        while not self._abort:
            if time.monotonic() - t_start > timeout_s:
                if self._marker_ids_t == 0.0:
                    return (None, 'no_topic')
                return (None, 'timeout')
            now = time.monotonic()
            visible = set(self._marker_ids) & all_ids
            missing = all_ids - visible
            ball = self._latest_ball_xy()

            # Live-status diagnostics: every 1 s, refresh the prompt
            # with the current state of the gate so the operator can
            # see exactly why we're not progressing.
            if now - last_status_t > 1.0:
                last_status_t = now
                if self._marker_ids_t == 0.0:
                    diag = ('Waiting for /platform_pose/marker_ids — '
                            'is platform_pose_node running with the '
                            'new build?')
                else:
                    if not visible:
                        diag = ('Topic alive but 0/8 markers detected '
                                '— camera blocked or ball off-centre?')
                    else:
                        miss_str = (','.join(str(m) for m in sorted(missing))
                                    if missing else 'none')
                        diag = (f'{len(visible)}/8 visible, missing: '
                                f'[{miss_str}]')
                        if len(missing) == 1 and ball is not None:
                            m = next(iter(missing))
                            mxy = self._marker_xy_imu(m)
                            if mxy is not None:
                                d = math.hypot(ball[0] - mxy[0],
                                               ball[1] - mxy[1])
                                diag += (f'; ball→m{m} dist={d:.0f} mm '
                                         f'(need ≤'
                                         f'{STEP_ID_BALL_ON_MARKER_TOL_MM:.0f})')
                        elif ball is None:
                            diag += '; no /ball_state yet'
                self._status['prompt'] = diag
                self._publish_status()

            # Need exactly one missing ID — if 0, the ball isn't
            # blocking anything (or operator hasn't placed it yet);
            # if >1, the camera is partially blocked or the ball is
            # straddling two markers (unusual).
            if len(missing) == 1:
                m = next(iter(missing))
                # Cross-sensor check uses the IMU-frame marker centre
                # so it's in the same frame as /ball_state.
                marker_xy_imu = self._marker_xy_imu(m)
                if ball is not None and marker_xy_imu is not None:
                    dist = math.hypot(ball[0] - marker_xy_imu[0],
                                      ball[1] - marker_xy_imu[1])
                    if dist <= STEP_ID_BALL_ON_MARKER_TOL_MM:
                        if candidate != m:
                            candidate = m
                            candidate_since = now
                        elif (now - (candidate_since or now)
                              >= STEP_ID_OCCLUSION_DWELL_S):
                            return (m, 'detected')
                    else:
                        candidate = None
                        candidate_since = None
                else:
                    candidate = None
                    candidate_since = None
            else:
                candidate = None
                candidate_since = None
            time.sleep(0.05)
        return (None, 'aborted')

    def _wait_for_ball_on_marker(self, expected_marker: int,
                                 timeout_s: float
                                 = STEP_ID_OPERATOR_PROMPT_S) -> bool:
        """Hard pause: ALWAYS waits for the operator to click Confirm
        before proceeding, even if the ball appears to be on the
        expected marker organically. This is the user-requested (A)
        behavior: "the goal is to have repeatable starting and ending
        positions so we can directly compare runs."

        Why no organic-arrival shortcut: a hand hovering over a marker
        plus a transiently-matching KF position can trip the gate (one
        such session ran trial 2 immediately after trial 1 because of
        this; the ball wasn't actually on the expected start marker).
        Operator-in-the-loop confirmation eliminates that whole class
        of false-positive starts.

        The Confirm button still cross-checks via _is_ball_on_marker
        before accepting — operator can't rush the platform by
        clicking before placing the ball. Returns True on success,
        False on timeout/abort.
        """
        marker_xy = self._marker_xy(expected_marker)
        if marker_xy is None:
            return False
        t_start = time.monotonic()
        self._step_id_operator_confirmed = False
        # Tell the GUI / log we're waiting.
        self._status['state'] = 'step_id_waiting_for_ball'
        self._status['waiting_for_marker'] = int(expected_marker)
        self._status['prompt'] = (
            f'Place ball on marker {expected_marker}, then click '
            f'"✓ Ball is on marker" to continue.')
        self._publish_status()
        last_status_t = 0.0
        while not self._abort:
            now = time.monotonic()
            if now - t_start > timeout_s:
                self._status['prompt'] = (
                    f'Timeout: operator never confirmed ball on '
                    f'marker {expected_marker}.')
                self._publish_status()
                return False
            # Live diagnostic so the operator can see the gate state
            # (same UX as _detect_starting_marker).
            if now - last_status_t > 1.0:
                last_status_t = now
                if self._is_ball_on_marker(expected_marker):
                    diag = (f'Ball appears to be on marker '
                            f'{expected_marker}. Click "✓ Ball is on '
                            f'marker" when ready.')
                else:
                    diag = (f'Place ball on marker {expected_marker}. '
                            f'Then click Confirm.')
                self._status['prompt'] = diag
                self._publish_status()
            if self._step_id_operator_confirmed:
                # Cross-sensor verify before accepting — the operator
                # may have clicked while the ball was still rolling
                # or before placing it. Re-arm so a future re-press
                # is required after the next failed verify.
                self._step_id_operator_confirmed = False
                if self._is_ball_on_marker(expected_marker):
                    return True
                self._status['prompt'] = (
                    f'Confirm pressed but ball not yet detected on '
                    f'marker {expected_marker} — re-place and click '
                    f'Confirm again.')
                self._publish_status()
            time.sleep(0.05)
        return False

    def _is_ball_on_marker(self, marker_id: int) -> bool:
        """Snapshot test: is the ball on the given marker right now?
        Both the occlusion AND ball-position checks must pass. Used by
        _wait_for_ball_on_marker and the closed-loop settle gate.
        Position check rotates marker centre into IMU frame so it
        compares against /ball_state in the same frame."""
        all_ids = {mid for mid, _, _ in self._step_id_marker_layout}
        visible = set(self._marker_ids) & all_ids
        if marker_id not in (all_ids - visible):
            return False
        ball = self._latest_ball_xy()
        marker_xy_imu = self._marker_xy_imu(marker_id)
        if ball is None or marker_xy_imu is None:
            return False
        d = math.hypot(ball[0] - marker_xy_imu[0],
                       ball[1] - marker_xy_imu[1])
        return d <= STEP_ID_BALL_ON_MARKER_TOL_MM

    def _snapshot_occluded_marker(self) -> Optional[int]:
        """Single-frame check: which marker is the ball occluding,
        if any? Returns the id when exactly one marker is missing
        from /platform_pose/marker_ids AND /ball_state agrees with
        that marker's IMU-frame centre to within tolerance.
        Returns None when 0 or >1 markers are missing, or when the
        cross-sensor check fails.
        """
        all_ids = {mid for mid, _, _ in self._step_id_marker_layout}
        visible = set(self._marker_ids) & all_ids
        missing = all_ids - visible
        if len(missing) != 1:
            return None
        m = next(iter(missing))
        ball = self._latest_ball_xy()
        marker_xy_imu = self._marker_xy_imu(m)
        if ball is None or marker_xy_imu is None:
            return None
        d = math.hypot(ball[0] - marker_xy_imu[0],
                       ball[1] - marker_xy_imu[1])
        if d > STEP_ID_BALL_ON_MARKER_TOL_MM:
            return None
        return m

    def _confirm_marker_window(self, dwell_s: float = 0.25
                               ) -> Optional[int]:
        """Verify cross-sensor agreement over a short window after
        the operator pressed Confirm. Filters out single-frame noise
        (blurred marker detection, KF transient) so the click only
        accepts when the gate has been stable for the dwell.
        Returns marker id if the same marker is detected throughout
        the dwell, else None."""
        end = time.monotonic() + dwell_s
        chosen: Optional[int] = None
        while time.monotonic() < end and not self._abort:
            m = self._snapshot_occluded_marker()
            if m is None:
                return None
            if chosen is None:
                chosen = m
            elif chosen != m:
                return None
            time.sleep(0.04)
        return chosen

    def _wait_for_ball_on_any_marker(self,
                                     timeout_s: float
                                     = STEP_ID_OPERATOR_PROMPT_S
                                     ) -> Optional[int]:
        """Operator-confirmed inter-trial gate. Waits for the
        operator to place the ball on ANY marker and click the
        Confirm button. Returns the detected marker id, or None on
        timeout/abort.

        Why "any marker" instead of a specific one: per operator
        request 2026-05-01 — "I can place it on whatever marker to
        start the process off so it is easier on me." Some markers
        are physically closer to the operator's desk than others;
        forcing a specific one was just inconvenience without any
        benefit (the trial offset cycle gives the diagnostic variety,
        not the absolute starting position).

        Cross-sensor gate (occlusion + ball position) still applies,
        plus a 250 ms post-confirm dwell so transient detection
        flicker can't false-trigger acceptance."""
        self._step_id_operator_confirmed = False
        self._status['state'] = 'step_id_waiting_for_ball'
        self._status.pop('waiting_for_marker', None)
        self._status['prompt'] = (
            'Place ball on any marker, then click "✓ Ball is on '
            'marker" to continue.')
        self._publish_status()
        t_start = time.monotonic()
        last_status_t = 0.0
        candidate: Optional[int] = None
        candidate_since: Optional[float] = None
        while not self._abort:
            now = time.monotonic()
            if now - t_start > timeout_s:
                self._status['prompt'] = (
                    'Timeout: operator never confirmed.')
                self._publish_status()
                return None
            # Debounce candidate detection.
            m = self._snapshot_occluded_marker()
            if m is not None:
                if candidate != m:
                    candidate = m
                    candidate_since = now
            else:
                candidate = None
                candidate_since = None
            # Live diagnostic — same as _detect_starting_marker but
            # for "any marker."
            if now - last_status_t > 1.0:
                last_status_t = now
                if (candidate is not None
                        and candidate_since is not None
                        and (now - candidate_since
                             >= STEP_ID_OCCLUSION_DWELL_S)):
                    diag = (f'Detected ball on marker {candidate}. '
                            f'Click "✓ Ball is on marker" to start '
                            f'the trial.')
                elif candidate is not None:
                    diag = (f'Ball appears on marker {candidate} — '
                            f'hold steady, then click Confirm.')
                else:
                    diag = ('Place ball on any marker, then click '
                            'Confirm.')
                self._status['prompt'] = diag
                self._publish_status()
            # Operator pressed Confirm: verify gate is stable over a
            # short dwell, then accept.
            if self._step_id_operator_confirmed:
                self._step_id_operator_confirmed = False
                confirmed = self._confirm_marker_window(0.25)
                if confirmed is not None:
                    return confirmed
                self._status['prompt'] = (
                    'Confirm pressed but ball not detected on a '
                    'marker. Re-place and click Confirm again.')
                self._publish_status()
            time.sleep(0.05)
        return None

    def _safe_open_loop_direction(self, marker_id: int,
                                  magnitude_deg: Optional[float] = None
                                  ) -> tuple[float, float]:
        """Choose a (roll_deg, pitch_deg) command that drives the ball
        from the start marker toward the platform centre.

        magnitude_deg: tilt magnitude in degrees. None falls back to
        STEP_ID_OPEN_LOOP_TILT_DEG (the legacy constant). Operator
        normally sets this via /step_id/cmd_tilt_magnitude.

        Sign math (from controls_journey.md):
          +pitch  → +X edge moves DOWN → ball slides toward +X
          +roll   → +Y edge moves UP   → ball slides toward −Y

        So to move the ball toward (0, 0) from (x0, y0): need negative
        velocity in both x and y. Pick the dominant component:
          if |x0| > |y0|: command -sign(x0)·pitch (x-direction)
          else:           command +sign(y0)·roll (y-direction;
                                                  +roll → −y motion)
        """
        xy = self._marker_xy(marker_id)
        if xy is None:
            return (0.0, 0.0)
        x, y = xy
        mag = (float(magnitude_deg) if magnitude_deg is not None
               else STEP_ID_OPEN_LOOP_TILT_DEG)
        if abs(x) >= abs(y):
            # Use pitch. +pitch drives ball toward +X; we want toward 0,
            # so pitch sign = -sign(x).
            pitch = -math.copysign(mag, x) if x != 0 else mag
            return (0.0, pitch)
        else:
            # Use roll. +roll drives ball toward -Y; we want toward 0,
            # so roll sign = +sign(y).
            roll = math.copysign(mag, y) if y != 0 else mag
            return (roll, 0.0)

    def _run_step_id_session(self) -> None:
        """STEP_ID main thread. Phases:
          1. detect starting marker
          2. open-loop tilt-step → G_eff
          3. compute recommended gains (printed, NOT auto-applied)
          4. closed-loop 4-trial marker-pair sequence with current gains
          5. write summary; operator can promote recommended gains via
             GUI after reviewing the digest
        """
        self._step_id_running = True
        bag_proc = None
        log_dir = ''
        try:
            log_dir = os.path.expanduser(
                f'~/stable_bot_repo/tuning_data/step_id_{_now_utc_compact()}')
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(log_dir, 'log.jsonl')
            summary_path = os.path.join(log_dir, 'summary.json')
            bag_dir = os.path.join(log_dir, 'bag')
            self.get_logger().info(f'step_id log → {log_dir}')
            bag_proc = self._start_step_id_bag(bag_dir)

            current_gains = _load_gains_yaml()
            session = {
                'started_at': time.strftime(
                    '%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'log_dir': log_dir,
                'current_gains': {
                    k: float(v) for k, v in current_gains.items()
                    if isinstance(v, (int, float))},
                # Session-level vision-backend snapshot. Most
                # sessions use one backend throughout, so this is
                # the "headline" tag for the whole run. Per-trial
                # snapshots in phases.open_loop / phases.verification
                # carry the value at the moment of each trial in
                # case the operator switched mid-session.
                'vision_backend_at_start':
                    self._vision_backend_snapshot(),
                'phases': {},
            }
            self._status.update({
                'state': 'step_id_starting',
                'log_dir': log_dir,
                'started_at': session['started_at'],
                'prompt': 'Place ball on any marker.',
            })
            self._publish_status()

            # --- Phase 1: detect starting marker -------------------------
            self.get_logger().info(
                'step_id phase 1: detecting starting marker '
                '(place ball on any marker)')
            self._status['state'] = 'step_id_detecting_marker'
            self._publish_status()
            start_marker, reason = self._detect_starting_marker(
                timeout_s=300.0)
            if start_marker is None:
                # Differentiate exit reasons so the operator-visible
                # message reflects what actually happened. Earlier
                # version reported "5 min timeout" for every None
                # return — including operator Stop, which is
                # misleading.
                msg_by_reason = {
                    'aborted':  'aborted by operator (Stop pressed)',
                    'timeout':  'no starting marker detected within '
                                '5 min — try placing the ball again '
                                'and ensure exactly one marker is '
                                'occluded',
                    'no_topic': '/platform_pose/marker_ids never '
                                'published — is the new build live? '
                                'Check `ros2 topic hz '
                                '/platform_pose/marker_ids`',
                }
                self._fail_step_id(
                    log_dir, summary_path, session,
                    msg_by_reason.get(reason, f'detect failed: {reason}'))
                return
            self.get_logger().info(
                f'step_id: ball detected on marker {start_marker}')
            session['start_marker'] = int(start_marker)

            # --- Phase 2: open-loop characterization (REPLICATED) ----
            #
            # Run the open-loop tilt step n_replicates times to get
            # mean ± std of G_eff. Single-trial plant ID was the #1
            # weakness called out in the critique: same hardware
            # produced G_eff=35, 224, 92, 64 across four consecutive
            # sessions (5× spread). With n=3 replicates we get a
            # sample mean and can refuse to recommend gains if
            # the spread (cv = std/mean) exceeds threshold.
            #
            # Between replicates the operator places the ball back on
            # any marker (using the same any-marker-confirm gate as
            # the verification trials). This gives n independent
            # trials, each from a clean starting state. The first
            # replicate uses the start_marker that the operator
            # already placed the ball on at session start.
            tilt_mag = float(self._step_id_tilt_magnitude_deg)
            n_reps = int(self._step_id_n_replicates)
            ol_replicates: list[dict] = []
            current_start = start_marker
            for rep in range(n_reps):
                # Replicate 0 uses the original start_marker. For
                # replicate ≥1 we ask the operator to place the ball
                # on any marker — they may pick a different one or
                # the same; each placement is independent.
                if rep > 0:
                    self._status.update({
                        'state': 'step_id_open_loop',
                        'prompt': (
                            f'Open-loop replicate {rep+1}/{n_reps}: '
                            f'place ball on any marker (the previous '
                            f'trial ran from marker '
                            f'{current_start}; ball has since moved).'),
                    })
                    self._publish_status()
                    new_start = self._wait_for_ball_on_any_marker()
                    if new_start is None:
                        self.get_logger().warn(
                            f'  replicate {rep+1}: operator never '
                            f'confirmed — proceeding with '
                            f'{len(ol_replicates)} replicate(s) so far')
                        break
                    current_start = new_start
                    self._sleep_with_abort(
                        STEP_ID_INTER_TRIAL_SETTLE_S)
                    if self._abort:
                        break
                roll_cmd, pitch_cmd = self._safe_open_loop_direction(
                    current_start, magnitude_deg=tilt_mag)
                self._status.update({
                    'state': 'step_id_open_loop',
                    'prompt': (
                        f'Open-loop replicate {rep+1}/{n_reps}: '
                        f'tilting {tilt_mag:.1f}° from marker '
                        f'{current_start} (roll={roll_cmd:+.1f}°, '
                        f'pitch={pitch_cmd:+.1f}°)'),
                })
                self._publish_status()
                self.get_logger().info(
                    f'step_id phase 2 replicate {rep+1}/{n_reps}: '
                    f'open-loop from marker {current_start} '
                    f'(roll={roll_cmd:+.2f}°, pitch={pitch_cmd:+.2f}°)')
                ol = self._run_open_loop_trial(
                    current_start, roll_cmd, pitch_cmd, log_path)
                if ol.get('aborted'):
                    self.get_logger().warn(
                        f'  replicate {rep+1} aborted: '
                        f'{ol.get("abort_reason")}; continuing')
                    continue
                ol_replicates.append(ol)

            if not ol_replicates:
                self._fail_step_id(
                    log_dir, summary_path, session,
                    'all open-loop replicates aborted; nothing to fit. '
                    'Check armed + level + ball detection, then retry.')
                return

            # Aggregate G_eff across replicates. Use whichever fit
            # method each replicate selected (parabolic vs velocity-
            # based) so the mean reflects the full robust pipeline.
            g_effs = [t['g_eff_mm_s2_per_deg']
                      for t in ol_replicates
                      if t.get('g_eff_mm_s2_per_deg', 0) > 0]
            tds_obs = [t.get('td_observed_s', 0.15)
                       for t in ol_replicates]
            achievements = [t.get('imu_achievement_ratio', 0.0)
                            for t in ol_replicates]
            if not g_effs:
                self._fail_step_id(
                    log_dir, summary_path, session,
                    'no usable G_eff fits across '
                    f'{len(ol_replicates)} replicates')
                return
            g_eff_mean = float(np.mean(g_effs))
            g_eff_std = float(np.std(g_effs, ddof=1)
                              if len(g_effs) > 1 else 0.0)
            g_eff_cv = (g_eff_std / g_eff_mean
                        if g_eff_mean > 0 else 1.0)
            td_mean = float(np.mean(tds_obs))
            achievement_mean = float(np.mean(achievements))

            aggregate = {
                'n_replicates': len(g_effs),
                'g_eff_mean_mm_s2_per_deg': g_eff_mean,
                'g_eff_std_mm_s2_per_deg': g_eff_std,
                'g_eff_cv': g_eff_cv,
                'g_effs': [float(g) for g in g_effs],
                'td_mean_s': td_mean,
                'imu_achievement_mean': achievement_mean,
            }
            session['phases']['open_loop_aggregate'] = aggregate
            session['phases']['open_loop_replicates'] = ol_replicates
            # Back-compat: keep phases.open_loop = first replicate so
            # the existing digest's plant_gain_fit.png works without
            # change. The new digest reads open_loop_aggregate when
            # available and falls back to open_loop otherwise.
            session['phases']['open_loop'] = ol_replicates[0]
            self.get_logger().info(
                f'step_id phase 2 aggregate: G_eff = {g_eff_mean:.1f} '
                f'± {g_eff_std:.1f} mm/s²/° '
                f'(cv={g_eff_cv*100:.0f}%, n={len(g_effs)})')

            # --- Phase 3: compute recommended gains ----------------------
            g_eff = g_eff_mean
            # Spread gate: refuse to recommend if the replicates
            # disagree by more than 30%. Either stiction is making
            # the plant gain unpredictable (different friction
            # breakaway each trial), or vision is intermittently
            # corrupting one of the fits, or the IMU achievement is
            # inconsistent. Whatever the cause, an analytic gain
            # derived from such a noisy estimate is dangerous.
            if g_eff_cv > STEP_ID_OL_CV_THRESHOLD:
                # Diagnose what's likely driving the inconsistency
                # by inspecting per-replicate state. A wide spread
                # in tilt_actual_duration across replicates is the
                # smoking-gun for "safety cutoff fired
                # inconsistently"; a wide spread in IMU achievement
                # suggests level-loop variability; a wide spread
                # in motion_onset suggests stiction breakaway is
                # genuinely random.
                durs = [t.get('tilt_actual_duration_s', 0.0)
                        for t in ol_replicates]
                imus = [t.get('imu_achievement_ratio', 0.0)
                        for t in ol_replicates]
                onsets = [(t.get('fit', {}) or {}).get(
                    'motion_onset_s') or 0.0
                          for t in ol_replicates]
                dur_spread = (max(durs) - min(durs)) / max(durs, default=1)
                imu_spread = max(imus) - min(imus)
                onset_spread = max(onsets) - min(onsets)
                hints = []
                if dur_spread > 0.25:
                    hints.append(
                        f'tilt durations varied widely across '
                        f'replicates (min={min(durs):.2f}s, '
                        f'max={max(durs):.2f}s) — safety cutoff '
                        f'likely fired inconsistently due to '
                        f'vision-noise velocity spikes')
                if imu_spread > 0.30:
                    hints.append(
                        f'IMU achievement varied widely '
                        f'({min(imus)*100:.0f}%-{max(imus)*100:.0f}% '
                        f'of commanded) — level-PI inner loop '
                        f'tracked inconsistently')
                if onset_spread > 0.20:
                    hints.append(
                        f'motion onset varied widely '
                        f'(min={min(onsets):.2f}s, '
                        f'max={max(onsets):.2f}s) — stiction '
                        f'breakaway is genuinely random; tighten '
                        f'platform / clean ball-deck contact')
                hints_str = (' Specific hint: '
                             + '; '.join(hints) + '.'
                             if hints else '')
                msg = (
                    f'plant gain too inconsistent across '
                    f'{len(g_effs)} replicates: G_eff = '
                    f'{g_eff_mean:.0f} ± {g_eff_std:.0f} '
                    f'(cv={g_eff_cv*100:.0f}%, '
                    f'individual values: '
                    f'{[round(g, 1) for g in g_effs]}).'
                    f'{hints_str} '
                    f'Refusing to recommend gains from a noisy '
                    f'estimate.')
                self._fail_step_id(log_dir, summary_path, session, msg)
                return
            # Sanity gate (per-trial mean): physical floor is roughly
            # 60 mm/s²/°. G_eff well below 20 means the BALL didn't
            # move much across replicates. Use the mean IMU
            # achievement to discriminate the two physically-distinct
            # failure modes:
            #   - achieved < 0.5 × commanded → platform didn't tilt
            #     (arming or level-loop issue)
            #   - achieved ≈ commanded but ball still didn't move
            #     → static friction held the ball. Bigger tilt needed.
            if g_eff < 20.0:
                first = ol_replicates[0]
                achieved = first.get('imu_achieved_tilt_deg', 0.0)
                commanded = first.get('imu_commanded_tilt_deg',
                                      tilt_mag)
                if achievement_mean < 0.5:
                    msg = (
                        f'open-loop trials: commanded {commanded:.2f}° '
                        f'tilt but IMU only reached '
                        f'{achievement_mean*100:.0f}% of commanded '
                        f'on average. Platform is not physically '
                        f'tilting. Confirm the system is armed and '
                        f'the level loop is enabled, then restart.')
                else:
                    suggested = max(commanded + 1.5,
                                    min(commanded * 2.0, 6.0))
                    msg = (
                        f'open-loop trials: platform tilted to '
                        f'{achievement_mean*100:.0f}% of commanded '
                        f'{commanded:.2f}° on average, but ball barely '
                        f'moved (G_eff={g_eff:.1f} mm/s²/°). Likely '
                        f'cause: static friction holding the ball. '
                        f'Increase the open-loop tilt magnitude — try '
                        f'{suggested:.1f}° via the GUI input — and '
                        f'restart.')
                self._fail_step_id(log_dir, summary_path, session, msg)
                return
            recommended = self._compute_recommended_gains(
                g_eff_mm_s2_per_deg=g_eff,
                td_observed_s=td_mean,
                current_gains=current_gains)
            recommended['g_eff_std_mm_s2_per_deg'] = g_eff_std
            recommended['g_eff_cv'] = g_eff_cv
            recommended['n_replicates'] = len(g_effs)
            session['phases']['recommendation'] = recommended
            self.get_logger().info(
                f'step_id phase 3: recommended gains '
                f'(based on G_eff={g_eff:.1f}±{g_eff_std:.1f} mm/s²/°, '
                f'n={len(g_effs)}): '
                f'kp={recommended["kp"]:.4f} '
                f'kd={recommended["kd"]:.4f}')
            self._status.update({
                'state': 'step_id_recommendation_ready',
                'g_eff_mm_s2_per_deg': float(g_eff),
                'recommended_gains': recommended,
                'prompt': (
                    f'Recommended: kp={recommended["kp"]:.4f}, '
                    f'kd={recommended["kd"]:.4f} '
                    f'(current: kp={current_gains.get("kp"):.4f}, '
                    f'kd={current_gains.get("kd"):.4f}). '
                    f'Verification trials starting with CURRENT gains.'),
            })
            self._publish_status()
            # Save partial summary now in case verification aborts.
            self._write_step_id_summary(summary_path, session)

            # --- Phase 4: closed-loop verification -----------------------
            #
            # Operator-driven start: each trial, the operator places
            # the ball on whatever marker is convenient (some are
            # closer to their desk than others) and clicks Confirm.
            # The trial's target is computed by adding a fixed offset
            # to whatever marker the ball was detected on. The offset
            # cycle [+1, +2, +1, +2] still gives the diagnostic
            # variety (two adjacent ~92 mm steps + two two-apart
            # ~170 mm steps) regardless of which marker the operator
            # chose for each trial.
            self._status['state'] = 'step_id_verification'
            self._publish_status()
            verification_trials = []
            VERIFICATION_OFFSETS = [1, 2, 1, 2]
            for i, off_target in enumerate(VERIFICATION_OFFSETS):
                self._status.update({
                    'state': f'step_id_verification_trial_{i+1}',
                    'verification_trial_idx': i + 1,
                    'verification_trial_count':
                        len(VERIFICATION_OFFSETS),
                    'verification_offset': int(off_target),
                    'prompt': (
                        f'Trial {i+1}/{len(VERIFICATION_OFFSETS)}: '
                        f'place ball on any marker; goto will be '
                        f'{off_target} marker(s) clockwise.'),
                })
                self._publish_status()
                detected_start = self._wait_for_ball_on_any_marker()
                if detected_start is None:
                    self.get_logger().warn(
                        f'  trial {i+1}: operator never confirmed '
                        f'— aborting verification phase')
                    break
                target = (detected_start + off_target) % 8
                self.get_logger().info(
                    f'step_id phase 4 trial {i+1}/'
                    f'{len(VERIFICATION_OFFSETS)}: '
                    f'marker {detected_start} → marker {target} '
                    f'(offset +{off_target})')
                self._sleep_with_abort(STEP_ID_INTER_TRIAL_DWELL_S)
                if self._abort:
                    break
                trial_data = self._run_closed_loop_marker_trial(
                    detected_start, target, i + 1, log_path)
                verification_trials.append(trial_data)
            session['phases']['verification'] = verification_trials

            self._status.update({
                'state': 'step_id_done',
                'prompt': (
                    f'Done. Recommended kp={recommended["kp"]:.4f}, '
                    f'kd={recommended["kd"]:.4f}. '
                    f'Press Promote to apply.'),
            })
            self._publish_status()
            self._write_step_id_summary(summary_path, session)
            self.get_logger().info(
                f'step_id session complete. summary → {summary_path}')
        except Exception as e:
            self.get_logger().error(f'step_id crashed: {e!r}')
            self._status['state'] = 'step_id_crashed'
            self._status['prompt'] = f'crashed: {e!r}'
            self._publish_status()
        finally:
            # Robust shutdown: guarantee BALL_TRACK is stopped and
            # current_rpy is zeroed before we mark the session done.
            # Why the multi-publish dance instead of one LEVEL_HOLD:
            #   - call_async on SetPose returns IMMEDIATELY on the
            #     client side regardless of req.blocking, so we can't
            #     wait on it that way; we have to give it wall-clock
            #     time to land at the control node.
            #   - If a BALL_TRACK_GOTO was published from a trial loop
            #     just before abort fired, there's an ordering race.
            #     Publishing LEVEL_HOLD multiple times spaced by short
            #     sleeps ensures whichever order the control node sees,
            #     the LAST message it processes is LEVEL_HOLD.
            # Total ~0.8 s of platform-shutdown time. Acceptable.
            try:
                self._stop_platform_definitively()
            except Exception:
                pass
            if bag_proc is not None:
                self._stop_bag(bag_proc)
            self._step_id_running = False

    def _stop_platform_definitively(self) -> None:
        """Bring the platform to level and ensure BALL_TRACK is stopped,
        defending against the BALL_TRACK_GOTO/LEVEL_HOLD ordering race.

        Sequence (~0.8 s wall-clock):
          1. publish LEVEL_HOLD                  → stops BALL_TRACK if
                                                    enabled
          2. sleep 100 ms                        → let control node
                                                    drain any queued
                                                    /control_cmd messages
          3. publish LEVEL_HOLD again            → if any BALL_TRACK_GOTO
                                                    was queued behind us,
                                                    this supersedes it
          4. sleep 200 ms
          5. SetPose to (0,0,nominal_z) rpy=0   → forces current_rpy=0
                                                    via _do_set_pose so
                                                    even with level loop
                                                    OFF the platform sits
                                                    flat
          6. sleep 500 ms                        → let SetPose actually
                                                    move the legs
          7. publish LEVEL_HOLD final            → belt-and-braces
        """
        try:
            self._publish_mode('LEVEL_HOLD')
        except Exception:
            pass
        time.sleep(0.10)
        try:
            self._publish_mode('LEVEL_HOLD')
        except Exception:
            pass
        time.sleep(0.20)
        try:
            self._reset_pose_to_nominal(blocking=False)
        except Exception:
            pass
        time.sleep(0.50)
        try:
            self._publish_mode('LEVEL_HOLD')
        except Exception:
            pass

    def _fail_step_id(self, log_dir, summary_path, session, reason):
        self.get_logger().error(f'step_id failed: {reason}')
        session['failure'] = reason
        self._write_step_id_summary(summary_path, session)
        self._status['state'] = 'step_id_failed'
        self._status['prompt'] = reason
        self._publish_status()

    def _write_step_id_summary(self, path, session):
        with open(path, 'w') as f:
            json.dump(session, f, indent=2)

    def _start_step_id_bag(self, out_dir):
        """Same shape as _start_bag but with the STEP_ID-specific topic
        list (fewer twiddle-only fields, plus marker_ids).
        """
        try:
            os.makedirs(os.path.dirname(out_dir), exist_ok=True)
            cmd = (
                'source /opt/ros/kilted/setup.bash && '
                'source /home/sorak/ros2_ws/install/local_setup.bash && '
                f'exec ros2 bag record -s mcap -o {out_dir} '
                + ' '.join(STEP_ID_BAG_TOPICS)
            )
            proc = subprocess.Popen(
                ['/bin/bash', '-c', cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid)
            self.get_logger().info(f'step_id bag → {out_dir}')
            time.sleep(0.3)
            return proc
        except Exception as e:
            self.get_logger().warn(
                f'failed to spawn step_id bag recorder: {e!r}')
            return None

    # ----- Open-loop tilt-step trial ---------------------------------------

    def _run_open_loop_trial(self, start_marker: int,
                             roll_deg: float, pitch_deg: float,
                             log_path: str) -> dict:
        """Open-loop characterization: drop to LEVEL_HOLD, command the
        platform to tilt to (roll_deg, pitch_deg) for STEP_ID_OPEN_LOOP_
        DURATION_S, then return to flat. Record ball trajectory.
        Fits the constant-tilt window with a parabola → G_eff.

        Returns dict suitable for embedding in the session JSON. Carries
        time series 't', 'x', 'y' so the digest can re-fit if needed.
        """
        out: dict = {
            'start_marker': int(start_marker),
            'roll_deg': float(roll_deg),
            'pitch_deg': float(pitch_deg),
            'duration_s': float(STEP_ID_OPEN_LOOP_DURATION_S),
            'aborted': False,
        }
        if self._set_pose_cli is None:
            out['aborted'] = True
            out['abort_reason'] = (
                'jugglebot_interfaces.SetPose unavailable — open-loop '
                'tilt cannot be commanded')
            return out
        # Make sure BALL_TRACK is off — we don't want closed-loop control
        # fighting the open-loop tilt.
        self._publish_mode('LEVEL_HOLD')
        self._sleep_with_abort(0.3)
        # Pre-tilt level pose at the operator-chosen start Z.
        start_z = float(self._step_id_start_z_mm)
        out['start_z_mm'] = start_z
        if self._set_pose_cli is not None:
            req = SetPose.Request()
            req.x = 0.0
            req.y = 0.0
            req.z = start_z
            req.roll = 0.0
            req.pitch = 0.0
            req.yaw = 0.0
            req.blocking = True
            try:
                self._set_pose_cli.call_async(req)
            except Exception as e:
                out['aborted'] = True
                out['abort_reason'] = f'pre-tilt set_pose failed: {e!r}'
                return out
        self._sleep_with_abort(STEP_ID_PRE_TILT_LEVEL_S)
        if self._abort:
            out['aborted'] = True
            out['abort_reason'] = 'aborted before tilt'
            return out

        ball0 = self._latest_ball_xy()
        if ball0 is None:
            out['aborted'] = True
            out['abort_reason'] = 'no ball state at trial start'
            return out
        out['ball_start_xy'] = list(ball0)
        # Snapshot vision backend at trial start. Different detectors
        # produce different position-noise profiles → different
        # G_eff fits, different verification trial outcomes. Tagging
        # every trial with backend lets us compare apples-to-apples
        # across sessions (cv2 vs v0_nn vs v1_yolo, v6 vs v8).
        out['vision_backend_at_start'] = (
            self._vision_backend_snapshot())
        # Snapshot platform state at trial start. Without this we
        # can't reproduce / diagnose session-to-session divergences
        # like 202650Z (IMU achievement 101%) vs 203101Z (achievement
        # 38%) three minutes apart on the same hardware. The state
        # of the level loop, current_rpy, and recent set_pose history
        # all influence open-loop fit quality.
        try:
            ss = self._status_snapshot or {}
            out['platform_state_at_start'] = {
                'armed': ss.get('armed'),
                'level_enabled': ss.get('level_enabled'),
                'current_xyz': ss.get('current_xyz'),
                'current_rpy': ss.get('current_rpy'),
                'imu_fresh': ss.get('imu_fresh'),
                'level_corr_roll_deg': ss.get('level_corr_roll_deg'),
                'level_corr_pitch_deg': ss.get('level_corr_pitch_deg'),
                'level_err_roll_deg': ss.get('level_err_roll_deg'),
                'level_err_pitch_deg': ss.get('level_err_pitch_deg'),
                'status_age_s': (
                    time.monotonic() - self._status_t
                    if self._status_t > 0 else None),
            }
        except Exception:
            out['platform_state_at_start'] = {'error': 'capture failed'}

        samples: list[dict] = []
        # Send tilt command. The control node's level-PI loop tracks
        # current_rpy → IMU; we just tell it what to track. Non-blocking
        # so we can sample during the trajectory.
        if self._set_pose_cli is not None:
            req = SetPose.Request()
            req.x = 0.0
            req.y = 0.0
            req.z = start_z
            req.roll = float(roll_deg)
            req.pitch = float(pitch_deg)
            req.yaw = 0.0
            req.blocking = False
            try:
                self._set_pose_cli.call_async(req)
            except Exception as e:
                out['aborted'] = True
                out['abort_reason'] = f'set_pose call failed: {e!r}'
                return out
        t0 = time.monotonic()
        cmd_active_until = t0 + STEP_ID_OPEN_LOOP_DURATION_S
        last_xy = None
        last_t = None
        peak_v_observed = 0.0
        # Ring buffer of the last 5 instantaneous velocity samples
        # for the multi-frame consensus safety cutoff. See comment
        # below at the cutoff check for the rationale; in short,
        # single-frame velocity spikes from vision noise (observed
        # 1234 mm/s on rep 2 of 211307Z) used to fire the cutoff
        # inconsistently across replicates, producing 51% CV
        # G_eff. The consensus filter requires 3 of last 5 to be
        # over the limit before cutting — single glitches no
        # longer trip it.
        recent_v: list[float] = []
        while not self._abort:
            now = time.monotonic()
            if now >= cmd_active_until:
                break
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                imu = self._step_id_imu_rpy or (0.0, 0.0)
                samples.append({'t': now - t0,
                                'x': float(p.x), 'y': float(p.y),
                                'imu_roll_deg': float(imu[0]),
                                'imu_pitch_deg': float(imu[1]),
                                'phase': 'tilt'})
                # Live velocity estimate for the safety cutoff. If
                # the ball is moving fast enough to escape the
                # platform during the remaining tilt, end the tilt
                # early — but only after t > 0.3 s so we still get
                # SOME constant-tilt data even on a fast-stiction
                # release.
                if last_xy is not None and last_t is not None:
                    dt = max(now - last_t, 1e-3)
                    vx = (float(p.x) - last_xy[0]) / dt
                    vy = (float(p.y) - last_xy[1]) / dt
                    v = math.hypot(vx, vy)
                    if v > peak_v_observed:
                        peak_v_observed = v
                    # Multi-frame consensus safety cutoff. Vision
                    # noise can spike a single instantaneous v to
                    # 1000+ mm/s; we only end the tilt early when
                    # at least N of the last 5 samples are over the
                    # limit (sustained motion, not a glitch).
                    recent_v.append(v)
                    if len(recent_v) > 5:
                        recent_v.pop(0)
                    n_over = sum(
                        1 for vi in recent_v
                        if vi > STEP_ID_OPEN_LOOP_VPEAK_LIMIT_MM_S)
                    if (n_over >= STEP_ID_OPEN_LOOP_VPEAK_CONSENSUS_N
                            and (now - t0) > 0.3):
                        self.get_logger().info(
                            f'  open-loop: cutting tilt short at '
                            f'{(now-t0)*1000:.0f} ms — ball '
                            f'sustained over '
                            f'{STEP_ID_OPEN_LOOP_VPEAK_LIMIT_MM_S:.0f} '
                            f'mm/s '
                            f'({n_over}/5 samples)')
                        break
                last_xy = (float(p.x), float(p.y))
                last_t = now
            time.sleep(0.01)
        out['peak_velocity_mm_s'] = float(peak_v_observed)
        out['tilt_actual_duration_s'] = float(time.monotonic() - t0)

        # Return to flat and record decel phase.
        if self._set_pose_cli is not None:
            req = SetPose.Request()
            req.x = 0.0
            req.y = 0.0
            req.z = start_z
            req.roll = 0.0
            req.pitch = 0.0
            req.yaw = 0.0
            req.blocking = False
            try:
                self._set_pose_cli.call_async(req)
            except Exception:
                pass
        decel_until = time.monotonic() + STEP_ID_POST_TILT_LEVEL_S
        while not self._abort and time.monotonic() < decel_until:
            now = time.monotonic()
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                imu = self._step_id_imu_rpy or (0.0, 0.0)
                samples.append({'t': now - t0,
                                'x': float(p.x), 'y': float(p.y),
                                'imu_roll_deg': float(imu[0]),
                                'imu_pitch_deg': float(imu[1]),
                                'phase': 'flat'})
            time.sleep(0.01)
        out['samples'] = samples

        # Two G_eff fits, picked between based on which is more
        # plausible for the actual data:
        #   1. Parabolic position-fit during the post-motion-onset
        #      window. Best when motion is smooth from early in the
        #      trial.
        #   2. Velocity-based: peak_v / ∫|θ_imu| dτ over [0, peak_v_t].
        #      Robust to stiction holding the ball through most of
        #      the parabolic fit window — peak velocity captures the
        #      integrated effect of all tilt regardless of when motion
        #      onset happened. This is what the prior session
        #      (step_id_20260501T191841Z) needed: stiction broke at
        #      0.66 s, safety cutoff fired 2 ms later, parabolic fit
        #      had 0 ms of motion data and reported G_eff=35 (vs the
        #      ~250-300 implied by peak_v=462 mm/s).
        g_eff_p, td_parabolic, fit_meta_p = self._fit_open_loop(
            samples, roll_deg, pitch_deg)
        g_eff_v, fit_meta_v = self._fit_velocity_based_g_eff(
            samples, roll_deg, pitch_deg)
        # Pick the more plausible value. Velocity-based wins when
        # parabolic returns a low or implausibly-low value AND
        # velocity-based is in the plausible range.
        #
        # Threshold raised 2026-05-01 from 50 to 100 after observing
        # in 202650Z that parabolic = 92 (looked plausible) but the
        # ball had only 0.18 s of fit-window data after stiction
        # broke; velocity-based 189 was the better physically-
        # plausible answer. Stiction systematically biases parabolic
        # low because the early no-motion samples drag the
        # 2nd-order fit's curvature down. Up to ~100 mm/s²/° the
        # parabolic fit is presumed contaminated by stiction;
        # velocity-based wins on the integral (which already
        # includes the no-motion phase as zero contribution from
        # the ball but full contribution from the IMU).
        prefer_velocity = (g_eff_p < 100.0 and 50.0 <= g_eff_v <= 500.0)
        if prefer_velocity:
            g_eff = g_eff_v
            fit_meta = {
                **fit_meta_p, **fit_meta_v,
                'fit_method': 'velocity_based',
                'parabolic_g_eff_mm_s2_per_deg': float(g_eff_p),
            }
        else:
            g_eff = g_eff_p
            fit_meta = {
                **fit_meta_p, **fit_meta_v,
                'fit_method': 'parabolic',
                'velocity_g_eff_mm_s2_per_deg': float(g_eff_v),
            }
        td = td_parabolic
        out['g_eff_mm_s2_per_deg'] = float(g_eff)
        out['g_eff_parabolic_mm_s2_per_deg'] = float(g_eff_p)
        out['g_eff_velocity_mm_s2_per_deg'] = float(g_eff_v)
        out['td_observed_s'] = float(td)
        out['fit'] = fit_meta
        out['fit_method'] = fit_meta['fit_method']

        # IMU achievement diagnosis. During the fit window, what tilt
        # did the platform actually reach (per the IMU) vs what we
        # commanded? Lets the failure path differentiate "platform
        # didn't tilt" (arming/level issue) from "platform tilted but
        # ball didn't move" (stiction, need bigger tilt).
        fit_window = fit_meta.get('fit_window_s', [0.0, 0.0])
        commanded_mag = math.hypot(roll_deg, pitch_deg)
        achievement_ratio = 0.0
        achieved_imu_tilt_deg = 0.0
        try:
            tilt_imu_samples = [
                s for s in samples
                if (s.get('phase') == 'tilt'
                    and 'imu_roll_deg' in s
                    and fit_window[0] <= s['t'] <= fit_window[1])
            ]
            if tilt_imu_samples:
                # Use the same axis the command used so we don't
                # average a deliberately-near-zero axis with the
                # commanded one.
                if abs(pitch_deg) >= abs(roll_deg):
                    vals = [abs(s['imu_pitch_deg'])
                            for s in tilt_imu_samples]
                else:
                    vals = [abs(s['imu_roll_deg'])
                            for s in tilt_imu_samples]
                achieved_imu_tilt_deg = float(sum(vals) / len(vals))
                if commanded_mag > 1e-3:
                    achievement_ratio = (achieved_imu_tilt_deg
                                         / commanded_mag)
        except Exception:
            pass
        out['imu_commanded_tilt_deg'] = float(commanded_mag)
        out['imu_achieved_tilt_deg'] = achieved_imu_tilt_deg
        out['imu_achievement_ratio'] = achievement_ratio

        # Log the trial event so the GUI's history panel sees it.
        msg = String()
        msg.data = json.dumps({
            'phase': 'open_loop',
            'start_marker': int(start_marker),
            'roll_deg': float(roll_deg),
            'pitch_deg': float(pitch_deg),
            'g_eff_mm_s2_per_deg': float(g_eff),
            'td_observed_s': float(td),
            'imu_achieved_tilt_deg': float(achieved_imu_tilt_deg),
            'imu_achievement_ratio': float(achievement_ratio),
            'n_samples': len(samples),
        })
        self.pub_trial.publish(msg)
        with open(log_path, 'a') as f:
            f.write(json.dumps({
                'trial': 0, 'algo': 'step_id_open_loop',
                'ts': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                **{k: v for k, v in out.items() if k != 'samples'},
                'n_samples': len(samples),
            }) + '\n')
        return out

    def _fit_open_loop(self, samples: list[dict], roll_deg: float,
                       pitch_deg: float) -> tuple[float, float, dict]:
        """Fit ball trajectory during the post-motion-onset window with
        a parabola: position(t) = x0 + v0·t + ½·a·t². The acceleration
        is divided by the IMU-ACTUAL tilt during the same window
        (not the commanded tilt — the level-PI inner loop typically
        only achieves 70-90% of commanded, so dividing by commanded
        underestimates G_eff by 1/achievement-ratio).

        Dead-time td is estimated as the time from tilt command (t=0)
        to when |ball position| diverges from baseline by > 5 mm.
        On foam-on-vinyl this is the static-friction breakthrough
        time, typically 100-400 ms depending on tilt magnitude.

        Fit window: starts at max(motion-onset-time, ramp-end) so
        when stiction holds the ball for 300+ ms, the fit only sees
        actually-accelerating samples instead of stationary ones.

        Returns (g_eff_mm_s2_per_deg, td_s, fit_meta_dict)."""
        if len(samples) < 10:
            return (0.0, 0.0, {'error': 'too few samples'})
        # Direction of expected motion in (x, y). Per
        # _safe_open_loop_direction:
        #   +pitch  → ball moves +x  (controls_journey: Ry(p)·(1,0,0)
        #             puts +X edge below 0 for p>0)
        #   +roll   → ball moves -y  (Rx(r)·(0,1,0) puts +Y edge above 0
        #             for r>0)
        # Expected ball acceleration vector direction:
        ax_dir = math.copysign(1.0, pitch_deg) if pitch_deg != 0 else 0.0
        ay_dir = -math.copysign(1.0, roll_deg) if roll_deg != 0 else 0.0
        nrm = math.hypot(ax_dir, ay_dir) or 1.0
        ax_dir /= nrm
        ay_dir /= nrm
        # Project ball position + IMU rpy onto motion direction.
        tilt_samples = [s for s in samples if s.get('phase') == 'tilt']
        ts = np.array([s['t'] for s in tilt_samples])
        xs = np.array([s['x'] for s in tilt_samples])
        ys = np.array([s['y'] for s in tilt_samples])
        imu_p = np.array([s.get('imu_pitch_deg', 0.0)
                          for s in tilt_samples])
        imu_r = np.array([s.get('imu_roll_deg', 0.0)
                          for s in tilt_samples])
        if len(ts) < 5:
            return (0.0, 0.0, {'error': 'too few tilt-phase samples'})
        # Baseline is the mean position before STEP_ID_OPEN_LOOP_RAMP_S
        baseline_mask = ts < STEP_ID_OPEN_LOOP_RAMP_S
        if baseline_mask.sum() < 2:
            x0 = float(xs[0])
            y0 = float(ys[0])
        else:
            x0 = float(np.mean(xs[baseline_mask]))
            y0 = float(np.mean(ys[baseline_mask]))
        # Project displacement onto motion direction.
        d = (xs - x0) * ax_dir + (ys - y0) * ay_dir
        # Dead-time: first sample where |d| > 5 mm. This is also the
        # earliest plausible motion-onset time.
        td = 0.0
        td_idx = None
        for i, (ti, di) in enumerate(zip(ts, d)):
            if abs(di) > 5.0:
                td = float(ti)
                td_idx = i
                break
        # Motion onset for the fit window: first |d| > MOTION_ONSET_MM.
        motion_onset_t: Optional[float] = None
        for ti, di in zip(ts, d):
            if abs(di) > STEP_ID_OPEN_LOOP_MOTION_ONSET_MM:
                motion_onset_t = float(ti)
                break
        # Fit window: max(ramp_end, motion_onset) → end of tilt.
        # When stiction holds the ball, motion onset is the dominant
        # constraint; when motion is prompt, ramp-end wins.
        fit_start = max(STEP_ID_OPEN_LOOP_RAMP_S,
                        motion_onset_t
                        if motion_onset_t is not None else 0.0)
        fit_mask = ts >= fit_start
        if fit_mask.sum() < 5:
            # Fall back to the full post-ramp window if motion onset
            # didn't leave enough samples — gives the parabola
            # *something* to fit even if it ends up tiny, which is
            # diagnostic info for the failure path.
            fit_mask = ts >= STEP_ID_OPEN_LOOP_RAMP_S
            fit_start = STEP_ID_OPEN_LOOP_RAMP_S
            if fit_mask.sum() < 5:
                return (0.0, td,
                        {'error': 'too few fit-window samples',
                         'motion_onset_s': motion_onset_t})
        t_fit = ts[fit_mask]
        d_fit = d[fit_mask]
        # least-squares fit: d = c0 + c1·t + c2·t²
        try:
            coefs = np.polyfit(t_fit, d_fit, 2)
        except Exception as e:
            return (0.0, td, {'error': f'polyfit failed: {e!r}'})
        a_fit = float(coefs[0]) * 2.0   # because d = ½·a·t² → a = 2·c2
        # IMU-actual tilt magnitude during the fit window. Use the
        # dominant axis (the one we commanded). Mean abs over the fit
        # window — this is what the ball physically experienced as
        # gravity-along-surface, regardless of what we commanded.
        if abs(pitch_deg) >= abs(roll_deg):
            imu_proj = np.abs(imu_p)
        else:
            imu_proj = np.abs(imu_r)
        imu_in_fit = imu_proj[fit_mask]
        actual_theta_deg = (float(np.mean(imu_in_fit))
                            if len(imu_in_fit) else 0.0)
        commanded_theta_deg = math.hypot(roll_deg, pitch_deg)
        if commanded_theta_deg <= 1e-6:
            return (0.0, td, {'error': 'zero commanded tilt'})
        # Use IMU actual when it's plausible (≥ 30% of commanded —
        # below that, the platform almost certainly didn't tilt, and
        # dividing by tiny actual_theta would balloon G_eff). Fall
        # back to commanded with a note in the meta.
        if actual_theta_deg >= 0.3 * commanded_theta_deg:
            theta_for_geff = actual_theta_deg
            theta_source = 'imu_actual'
        else:
            theta_for_geff = commanded_theta_deg
            theta_source = 'commanded_fallback'
        g_eff = abs(a_fit) / theta_for_geff
        meta = {
            'a_fit_mm_s2': float(a_fit),
            'theta_deg': float(commanded_theta_deg),
            'theta_for_geff_deg': float(theta_for_geff),
            'theta_source': theta_source,
            'imu_actual_theta_deg_in_fit': float(actual_theta_deg),
            'motion_onset_s': (float(motion_onset_t)
                                if motion_onset_t is not None else None),
            'fit_window_s': [float(t_fit[0]), float(t_fit[-1])],
            'n_fit_samples': int(fit_mask.sum()),
            'baseline_xy': [x0, y0],
            'expected_motion_unit': [ax_dir, ay_dir],
            'sign_check_passed': bool(a_fit > 0),
        }
        return (g_eff, td, meta)

    def _fit_velocity_based_g_eff(self, samples: list[dict],
                                  roll_deg: float, pitch_deg: float
                                  ) -> tuple[float, dict]:
        """G_eff via linear regression of v(t) vs ∫θ(τ)dτ during the
        sustained-motion window. The slope of this line IS the plant
        gain (physics: a = G·θ integrates to v = G·∫θ + C).

        Why linear regression instead of peak_v / integrated_tilt?
        Per-trial G_eff using peak_v varied 5× across the 213307Z
        replicates ([104, 124, 214]) even with cleaned IMU and
        consistent trajectories — the noise was in the peak-velocity
        sample (single-frame vision glitch surviving the 95th-
        percentile filter) and in the choice of integration window
        (peak_v_t happens at variable times relative to the tilt-
        phase boundary). Linear regression uses ALL samples in the
        motion window via least-squares, so a few outlier frames
        don't dominate, AND the integration window is a fixed
        analytical construct (cumulative integral from t=0) rather
        than a noise-determined one.

        Implementation:
          1. Median-filter the ball-position arrays (kernel=5 frames)
             to absorb single-frame V0 dropouts that would otherwise
             produce velocity spikes.
          2. Compute v(t) along the expected motion direction.
          3. Compute cumulative integral of |θ_imu(t)| from t=0
             (always positive, magnitude in expected direction).
          4. Detect motion onset as first time smoothed velocity
             exceeds 30 mm/s for ≥3 consecutive frames — a stricter
             definition than "displacement > 5 mm" because it
             requires SUSTAINED motion not just a single frame
             jump.
          5. Linear-fit v(t) vs cumint(t) over [motion_onset, end-
             of-samples]. Slope = G_eff.
          6. Robust cleanup: drop samples whose fit residual is
             >3σ, refit. One iteration is enough for typical noise.

        Returns (g_eff_mm_s2_per_deg, meta_dict)."""
        if len(samples) < 10:
            return (0.0, {'velocity_fit_error': 'too few samples'})
        ax_dir = math.copysign(1.0, pitch_deg) if pitch_deg != 0 else 0.0
        ay_dir = -math.copysign(1.0, roll_deg) if roll_deg != 0 else 0.0
        nrm = math.hypot(ax_dir, ay_dir) or 1.0
        ax_dir /= nrm
        ay_dir /= nrm
        ts = np.array([s['t'] for s in samples])
        xs_raw = np.array([s['x'] for s in samples])
        ys_raw = np.array([s['y'] for s in samples])
        imu_p = np.array([s.get('imu_pitch_deg', 0.0) for s in samples])
        imu_r = np.array([s.get('imu_roll_deg', 0.0) for s in samples])
        # Median-filter the position arrays to absorb single-frame V0
        # dropouts. With kernel=5, a single bad frame's value gets
        # replaced by the median of itself + 4 neighbours — the
        # outlier never dominates. Implemented inline since scipy
        # isn't a hard dependency for this package.
        def _median_filter(a: np.ndarray, k: int = 5) -> np.ndarray:
            if len(a) < k:
                return a.copy()
            half = k // 2
            out = np.zeros_like(a)
            for i in range(len(a)):
                lo = max(0, i - half)
                hi = min(len(a), i + half + 1)
                out[i] = np.median(a[lo:hi])
            return out
        xs = _median_filter(xs_raw, 5)
        ys = _median_filter(ys_raw, 5)
        # Velocity from smoothed position (central differences).
        vx = np.gradient(xs, ts)
        vy = np.gradient(ys, ts)
        v_along = vx * ax_dir + vy * ay_dir
        # IMU magnitude in expected motion direction (always positive
        # because we tilt one way only; cumulative integral grows
        # monotonically).
        if abs(pitch_deg) >= abs(roll_deg):
            imu_dominant = np.abs(imu_p)
        else:
            imu_dominant = np.abs(imu_r)
        # Cumulative trapezoid integral of |θ_imu(t)| from t=0.
        # Using np.cumsum + dt approximation here; for precise
        # integration use np.trapz — but the speed difference at
        # 100 Hz × 2 s × 200 samples is negligible.
        dt = np.diff(ts, prepend=ts[0])
        cum_int_tilt = np.cumsum(0.5 * (imu_dominant
                                         + np.roll(imu_dominant, 1)) * dt)
        cum_int_tilt[0] = 0.0
        # Smoothed-velocity-based motion onset: first t where the
        # 3-frame moving average of v_along exceeds 30 mm/s.
        v_smooth = _median_filter(v_along, 3)
        motion_onset_idx = None
        for i in range(2, len(v_smooth)):
            if (v_smooth[i] > 30.0 and v_smooth[i-1] > 30.0
                    and v_smooth[i-2] > 30.0):
                motion_onset_idx = i - 2
                break
        if motion_onset_idx is None:
            return (0.0, {
                'velocity_fit_error': (
                    'no sustained motion detected (smoothed v < '
                    '30 mm/s for 3+ consecutive frames throughout '
                    'the trial)'),
            })
        # Linear regression v(t) = G_eff · cum_int(t) + offset over
        # [motion_onset, end_of_samples]. Slope = G_eff.
        fit_mask = np.arange(len(ts)) >= motion_onset_idx
        if fit_mask.sum() < 5:
            return (0.0, {
                'velocity_fit_error': 'too few post-onset samples',
            })
        x_fit = cum_int_tilt[fit_mask]
        y_fit = v_along[fit_mask]
        # Need x_fit to span enough range for a meaningful slope.
        if x_fit[-1] - x_fit[0] < 0.05:
            return (0.0, {
                'velocity_fit_error': 'integrated tilt range too small',
            })
        try:
            slope, intercept = np.polyfit(x_fit, y_fit, 1)
        except Exception as e:
            return (0.0, {
                'velocity_fit_error': f'polyfit failed: {e!r}',
            })
        # Robust cleanup: drop samples > 3σ off the line, refit once.
        residuals = y_fit - (slope * x_fit + intercept)
        std_resid = float(np.std(residuals))
        n_dropped = 0
        if std_resid > 1e-6:
            keep = np.abs(residuals) < 3.0 * std_resid
            if keep.sum() >= 5:
                n_dropped = int(np.sum(~keep))
                try:
                    slope, intercept = np.polyfit(
                        x_fit[keep], y_fit[keep], 1)
                except Exception:
                    pass
        # Slope is signed: positive = ball moving in expected
        # direction at the rate predicted by accumulated tilt. We
        # report magnitude.
        g_eff_v = float(abs(slope))
        return (g_eff_v, {
            'velocity_fit_method': 'lr_v_vs_cumint_theta',
            'lr_slope_signed': float(slope),
            'lr_intercept_mm_s': float(intercept),
            'lr_motion_onset_s': float(ts[motion_onset_idx]),
            'lr_n_fit_samples': int(fit_mask.sum()),
            'lr_n_outliers_dropped': n_dropped,
            'lr_residual_std_mm_s': std_resid,
            'lr_cum_int_range_deg_s':
                float(x_fit[-1] - x_fit[0]),
        })

    # ----- Closed-loop marker-pair trial -----------------------------------

    def _run_closed_loop_marker_trial(self, start_marker: int,
                                      target_marker: int,
                                      trial_idx: int,
                                      log_path: str) -> dict:
        """Closed-loop goto trial: publish BALL_TRACK_GOTO with target
        marker's centre as the goal. Settle when target marker is
        continuously occluded for STEP_ID_TARGET_OCCLUSION_S AND ball
        state agrees.

        Returns dict for the session JSON. Time series 't', 'x', 'y',
        plus 'commanded_at_t' (time the goto was published — t=0
        baseline for the digest's initial-acceleration G_eff fit).
        """
        out: dict = {
            'trial_idx': int(trial_idx),
            'start_marker': int(start_marker),
            'target_marker': int(target_marker),
            'aborted': False,
        }
        target_xy = self._marker_xy(target_marker)
        if target_xy is None:
            out['aborted'] = True
            out['abort_reason'] = f'unknown target marker {target_marker}'
            return out
        ball0 = self._latest_ball_xy()
        if ball0 is None:
            out['aborted'] = True
            out['abort_reason'] = 'no ball state at trial start'
            return out
        out['ball_start_xy'] = list(ball0)
        out['target_xy'] = list(target_xy)
        # Same snapshot as the open-loop trial — the verification
        # phase can run with a different backend than the open-loop
        # phase if the operator switched mid-session.
        out['vision_backend_at_start'] = (
            self._vision_backend_snapshot())
        # Status prompt: tell the operator the trial is starting and
        # what's happening. Without this the GUI sits silent for 25 s
        # and the operator can't tell when the trial actually started
        # (the BALL_TRACK_GOTO publish is invisible from the operator's
        # side until the ball moves).
        self._status['prompt'] = (
            f'Trial {trial_idx} RUNNING: marker {start_marker} → '
            f'marker {target_marker} '
            f'(target xy = {target_xy[0]:+.1f}, {target_xy[1]:+.1f}). '
            f'Settles when target marker is occluded for '
            f'{STEP_ID_TARGET_OCCLUSION_S:.0f} s.')
        self._publish_status()
        # Fire the goto. Note: ref_generator handles the platform→IMU
        # rotation, so we send the marker centre in platform frame and
        # the existing pipeline takes care of the rest.
        self._publish_mode('BALL_TRACK_GOTO',
                           extra={'x_mm': target_xy[0],
                                  'y_mm': target_xy[1]})
        out['commanded_at'] = time.strftime(
            '%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        t0 = time.monotonic()
        samples: list[dict] = []
        settled = False
        settled_t: Optional[float] = None
        target_occluded_since: Optional[float] = None
        last_progress_t = 0.0
        while not self._abort:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed > STEP_ID_GOTO_TIMEOUT_S:
                break
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                # Capture IMU + commanded-tilt + phase per sample so
                # the digest can plot commanded-vs-achieved tilt
                # (saturation visible) and a phase strip (when did
                # stiction relief fire?). Both come from latched
                # subscribers; if either is stale (e.g., diag arrived
                # >0.5 s ago), values fall back to 0 and a NaN-like
                # marker so the digest can mask them out.
                imu = self._step_id_imu_rpy or (0.0, 0.0)
                diag = self._step_id_ball_track_diag
                diag_age = (now - self._step_id_ball_track_diag_t
                            if self._step_id_ball_track_diag_t > 0
                            else 999.0)
                if diag is not None and diag_age < 0.5:
                    cmd_pitch = float(diag[2])
                    cmd_roll = float(diag[3])
                    phase_code = float(diag[1])
                else:
                    cmd_pitch = 0.0
                    cmd_roll = 0.0
                    phase_code = -2.0  # "no diag" sentinel
                samples.append({
                    't': elapsed,
                    'x': float(p.x), 'y': float(p.y),
                    'imu_pitch_deg': float(imu[1]),
                    'imu_roll_deg': float(imu[0]),
                    'cmd_pitch_deg': cmd_pitch,
                    'cmd_roll_deg': cmd_roll,
                    'phase_code': phase_code,
                })
            # Cross-sensor settle gate: target marker occluded AND
            # ball-position within tolerance.
            if self._is_ball_on_marker(target_marker):
                if target_occluded_since is None:
                    target_occluded_since = now
                elif (now - target_occluded_since
                      >= STEP_ID_TARGET_OCCLUSION_S):
                    settled = True
                    settled_t = elapsed
                    break
            else:
                target_occluded_since = None
            # Live progress: update the prompt every 1 s with elapsed
            # time + current ball-to-target distance so the operator
            # has visible signs of life.
            if now - last_progress_t > 1.0:
                last_progress_t = now
                if self._ball_state is not None:
                    p = self._ball_state.pose.position
                    err = math.hypot(float(p.x) - target_xy[0],
                                     float(p.y) - target_xy[1])
                    self._status['prompt'] = (
                        f'Trial {trial_idx} RUNNING — '
                        f't={elapsed:.1f}s, err={err:.0f} mm '
                        f'(marker {start_marker} → {target_marker})')
                    self._publish_status()
            time.sleep(0.02)
        # ---- End of trial: explicitly flatten the platform ----
        # mode:LEVEL_HOLD stops BALL_TRACK but does NOT zero
        # current_rpy. The level loop's setpoint becomes
        # level_ref + current_rpy, so if BALL_TRACK left current_rpy
        # at (1.5°, -2°), the platform stays tilted at that pose.
        # Operator places ball → ball rolls off due to residual tilt.
        # Fix: send LEVEL_HOLD AND a SetPose to (0,0,start_z)(0,0,0)
        # then sleep STEP_ID_INTER_TRIAL_SETTLE_S so the platform
        # actually reaches flat before the next trial's prompt.
        self._publish_mode('LEVEL_HOLD')
        if settled:
            self._status['prompt'] = (
                f'Trial {trial_idx} SETTLED at t={settled_t:.1f}s. '
                f'Leveling platform for {STEP_ID_INTER_TRIAL_SETTLE_S:.0f} '
                f's…')
        else:
            self._status['prompt'] = (
                f'Trial {trial_idx} did not settle within '
                f'{STEP_ID_GOTO_TIMEOUT_S:.0f} s. Leveling platform '
                f'for {STEP_ID_INTER_TRIAL_SETTLE_S:.0f} s…')
        self._publish_status()
        if self._set_pose_cli is not None:
            try:
                req = SetPose.Request()
                req.x = 0.0
                req.y = 0.0
                req.z = float(self._step_id_start_z_mm)
                req.roll = 0.0
                req.pitch = 0.0
                req.yaw = 0.0
                req.blocking = False
                self._set_pose_cli.call_async(req)
            except Exception:
                pass
        # Settle wall-clock so the platform physically reaches flat
        # before the operator is asked to place the next ball. Without
        # this sleep the operator sees the prompt change ~50 ms after
        # the trial ends, but the platform takes ~1 s to flatten,
        # during which the ball rolls off.
        self._sleep_with_abort(STEP_ID_INTER_TRIAL_SETTLE_S)
        out['settled'] = bool(settled)
        out['settled_at_s'] = (float(settled_t)
                               if settled_t is not None else None)
        out['duration_s'] = float(time.monotonic() - t0)
        out['n_samples'] = len(samples)
        out['samples'] = samples

        # Log to the trial topic for the GUI history.
        msg = String()
        msg.data = json.dumps({
            'phase': 'closed_loop_verification',
            'trial_idx': trial_idx,
            'start_marker': start_marker,
            'target_marker': target_marker,
            'settled': settled,
            'settled_at_s': out['settled_at_s'],
            'duration_s': out['duration_s'],
        })
        self.pub_trial.publish(msg)
        with open(log_path, 'a') as f:
            f.write(json.dumps({
                'trial': trial_idx, 'algo': 'step_id_closed_loop',
                'ts': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                **{k: v for k, v in out.items() if k != 'samples'},
            }) + '\n')
        return out

    # ----- Analytic gain calculation ---------------------------------------

    def _compute_recommended_gains(self, g_eff_mm_s2_per_deg: float,
                                   td_observed_s: float,
                                   current_gains: dict) -> dict:
        """Given the cascaded plant gain G_eff (mm/s² per degree of
        commanded tilt) and the observed dead-time, compute Kp and Kd
        that hit a target damping ratio ζ and an adaptive natural
        frequency ωn = min(0.5/Td, 5 rad/s).

        Closed-loop char eq with PD and plant ẍ = G·θ:
            s² + G·Kd·s + G·Kp = 0
        →  ωn² = G·Kp     → Kp = ωn² / G
           2ζωn = G·Kd    → Kd = 2ζωn / G

        Where G is in mm/s² per degree and Kp is in deg/mm, Kd in deg·s/mm.
        """
        if g_eff_mm_s2_per_deg <= 1.0:
            # G_eff is unphysically small — fit failed or ball didn't
            # move. Fall back to leaving gains alone with a warning.
            return {
                'kp': float(current_gains.get('kp', 0.015)),
                'kd': float(current_gains.get('kd', 0.03)),
                'ki': float(current_gains.get('ki', 0.001)),
                'max_tilt_deg':
                    float(current_gains.get('max_tilt_deg', 2.5)),
                'omega_n_rad_s': float(STEP_ID_FALLBACK_OMEGA_N),
                'zeta': float(STEP_ID_TARGET_ZETA),
                'g_eff_used': float(g_eff_mm_s2_per_deg),
                'note': ('G_eff implausibly small; recommendation '
                         'falls back to current gains'),
                'valid': False,
            }
        # ωn target = min(0.5/Td, OMEGA_N_CAP). Td is the
        # vision+control+IK cascade dead-time, NOT the motion-onset
        # time. Earlier code used motion-onset (which is dominated
        # by stiction breakthrough, ~500 ms on this hardware) and
        # produced ωn = 0.76 rad/s — settling time ~5 s, basically
        # useless. Using STEP_ID_LATENCY_TD_S (120 ms) gives
        # ωn = 4.17 rad/s, capped at OMEGA_N_CAP=5.
        omega_n = min(0.5 / STEP_ID_LATENCY_TD_S, STEP_ID_OMEGA_N_CAP)
        zeta = STEP_ID_TARGET_ZETA
        kp = (omega_n ** 2) / g_eff_mm_s2_per_deg
        kd = 2.0 * zeta * omega_n / g_eff_mm_s2_per_deg
        # Keep the existing Ki — analytic PD doesn't compute it (Ki is
        # for friction/stiction, not bandwidth shaping). Operator can
        # tune it separately if needed.
        ki = float(current_gains.get('ki', 0.001))
        # Recommended max_tilt: Kp × 60 mm gives the linear-range authority
        # for a 60 mm error (mid-range adjacent-marker step). Below this,
        # the controller saturates immediately and runs effectively bang-
        # bang. Capped at 8° for safety, with a 2.5° floor so we never
        # recommend less than current. Per operator observation
        # 2026-05-01: prior session's recommended Kp=0.077 saturated the
        # default max_tilt=2.5° at any error > 32 mm, producing orbital
        # ball motion in all 4 verification trials.
        max_tilt_deg_rec = max(2.5, min(8.0, kp * 60.0))
        return {
            'kp': float(kp),
            'kd': float(kd),
            'ki': ki,
            'max_tilt_deg': float(max_tilt_deg_rec),
            'omega_n_rad_s': float(omega_n),
            'zeta': float(zeta),
            'g_eff_used': float(g_eff_mm_s2_per_deg),
            'td_observed_s': float(td_observed_s),
            'td_used_for_omega_n_s': float(STEP_ID_LATENCY_TD_S),
            'note': (
                f'computed from G_eff={g_eff_mm_s2_per_deg:.1f} mm/s²/°, '
                f'Td_latency={STEP_ID_LATENCY_TD_S:.3f}s, '
                f'ωn={omega_n:.2f} rad/s, ζ={zeta:.2f}'
                f' (motion-onset Td={td_observed_s:.3f}s '
                f'reported separately; not used for ωn)'),
            'valid': True,
        }

    # ----- Restore-best service --------------------------------------------

    def _scan_history(self) -> list[dict]:
        """Walk every tuning_data/auto_tune_*/summary.json and return
        a list of {dir, n_trials, best_fitness, best_gains, mtime}.
        Sorted by mtime descending (newest first)."""
        root = os.path.expanduser('~/stable_bot_repo/tuning_data')
        out = []
        for sj in glob.glob(os.path.join(root, 'auto_tune_*', 'summary.json')):
            try:
                with open(sj) as f:
                    d = json.load(f)
                d['dir'] = os.path.dirname(sj)
                d['mtime'] = os.path.getmtime(sj)
                out.append(d)
            except Exception:
                continue
        out.sort(key=lambda r: r['mtime'], reverse=True)
        return out

    def _do_restore_best(self, req, res):
        history = self._scan_history()
        if not history:
            res.success = False
            res.message = 'no auto_tune sessions found in tuning_data/'
            return res
        best = max(history, key=lambda r: r.get('best_fitness', 0.0))
        gains = best.get('best_gains', {})
        if not gains:
            res.success = False
            res.message = (f'best session ({best["dir"]}) has no '
                           f'gains recorded')
            return res
        if not self._apply_gains(gains):
            res.success = False
            res.message = 'gain save failed'
            return res
        # Important: this restores the GAINS, not the fitness. The
        # next session's trial 0 re-evaluates these gains fresh —
        # so a one-off lucky run that produced a misleadingly high
        # fitness can't trap the next session's optimizer ("nothing
        # beats the historical 0.71"). The historical fitness is
        # surfaced here only as context.
        historical_f = best.get('best_fitness', 0.0)
        res.success = True
        res.message = (
            f'restored gains from {os.path.basename(best["dir"])} '
            f'(historical f={historical_f:.3f}, '
            f'n={best.get("n_trials", 0)}); '
            f'next session will re-evaluate fresh')
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    node = AutoTuneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
