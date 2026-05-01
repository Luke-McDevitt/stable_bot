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

# Open-loop tilt-step parameters. Conservative defaults: 1.0° tilt for
# 0.5 s gives ~30 mm of ball travel, well within marker-to-rim margin.
STEP_ID_OPEN_LOOP_TILT_DEG    = 1.0
STEP_ID_OPEN_LOOP_DURATION_S  = 0.5
STEP_ID_OPEN_LOOP_RAMP_S      = 0.15   # ramp-in/ramp-out so the
                                       # platform doesn't overshoot
                                       # the commanded tilt
STEP_ID_PRE_TILT_LEVEL_S      = 1.0    # hold flat for this long before
                                       # tilting, so the parabola fit
                                       # has a clean t=0 baseline
STEP_ID_POST_TILT_LEVEL_S     = 1.0    # hold flat after tilting so the
                                       # ball decelerates within the bag

# Closed-loop verification parameters
STEP_ID_GOTO_TIMEOUT_S        = 25.0   # per-trial goto timeout
STEP_ID_TARGET_OCCLUSION_S    = 1.0    # target marker continuously
                                       # occluded for this long → settled
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

            # --- Phase 2: open-loop characterization ---------------------
            tilt_mag = float(self._step_id_tilt_magnitude_deg)
            self._status.update({
                'state': 'step_id_open_loop',
                'prompt': (f'Characterizing plant — tilting platform '
                           f'{tilt_mag:.1f}° from marker {start_marker}.'),
            })
            self._publish_status()
            roll_cmd, pitch_cmd = self._safe_open_loop_direction(
                start_marker, magnitude_deg=tilt_mag)
            self.get_logger().info(
                f'step_id phase 2: open-loop tilt step '
                f'(roll={roll_cmd:+.2f}°, pitch={pitch_cmd:+.2f}°) '
                f'from marker {start_marker}')
            ol_trial = self._run_open_loop_trial(
                start_marker, roll_cmd, pitch_cmd, log_path)
            session['phases']['open_loop'] = ol_trial

            if ol_trial.get('aborted'):
                self._fail_step_id(
                    log_dir, summary_path, session,
                    f'open-loop trial failed: {ol_trial.get("abort_reason")}')
                return

            # --- Phase 3: compute recommended gains ----------------------
            g_eff = ol_trial.get('g_eff_mm_s2_per_deg', 0.0)
            # Sanity gate: physical floor is roughly 60 mm/s²/° (level-PI
            # losing half its setpoint) up to 200 mm/s²/° (slight
            # over-tilt). G_eff well below 20 means the BALL didn't
            # move much. Use the IMU achievement ratio to discriminate
            # the two physically-distinct failure modes:
            #   - achieved < 0.5 × commanded → platform didn't tilt
            #     (arming or level-loop issue)
            #   - achieved ≈ commanded but ball still didn't move
            #     → static friction held the ball. Bigger tilt needed.
            if g_eff < 20.0:
                ratio = ol_trial.get('imu_achievement_ratio', 0.0)
                achieved = ol_trial.get('imu_achieved_tilt_deg', 0.0)
                commanded = ol_trial.get('imu_commanded_tilt_deg',
                                         tilt_mag)
                if ratio < 0.5:
                    msg = (
                        f'open-loop trial: commanded {commanded:.2f}° '
                        f'tilt but IMU only reached {achieved:.2f}° '
                        f'({ratio*100:.0f}% of commanded). Platform is '
                        f'not physically tilting. Confirm the system '
                        f'is armed and the level loop is enabled, '
                        f'then restart.')
                else:
                    suggested = max(commanded + 1.5,
                                    min(commanded * 2.0, 6.0))
                    msg = (
                        f'open-loop trial: platform tilted to '
                        f'{achieved:.2f}° per IMU '
                        f'({ratio*100:.0f}% of commanded {commanded:.2f}°), '
                        f'but ball barely moved (G_eff='
                        f'{g_eff:.1f} mm/s²/°). Likely cause: static '
                        f'friction holding the ball. Increase the '
                        f'open-loop tilt magnitude — try '
                        f'{suggested:.1f}° via the GUI input — and '
                        f'restart.')
                self._fail_step_id(log_dir, summary_path, session, msg)
                return
            recommended = self._compute_recommended_gains(
                g_eff_mm_s2_per_deg=g_eff,
                td_observed_s=ol_trial.get('td_observed_s', 0.15),
                current_gains=current_gains)
            session['phases']['recommendation'] = recommended
            self.get_logger().info(
                f'step_id phase 3: recommended gains '
                f'(based on G_eff={g_eff:.1f} mm/s²/°): '
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
            self._status['state'] = 'step_id_verification'
            self._publish_status()
            verification_trials = []
            current_marker = start_marker
            for i, (off_start, off_target) in enumerate(
                    STEP_ID_MARKER_OFFSETS):
                # Adjust offsets so the first start matches the operator-
                # detected start marker (the table's first off_start is 0,
                # so this is a no-op for trial 0 and a delta for the rest).
                expected_start = (start_marker + off_start) % 8
                target = (start_marker + off_target) % 8
                self.get_logger().info(
                    f'step_id phase 4 trial {i+1}/4: '
                    f'marker {expected_start} → marker {target}')
                self._status.update({
                    'state': f'step_id_verification_trial_{i+1}',
                    'verification_trial_idx': i + 1,
                    'verification_trial_count': len(STEP_ID_MARKER_OFFSETS),
                    'expected_start_marker': int(expected_start),
                    'target_marker': int(target),
                    'prompt': (
                        f'Trial {i+1}/4: place ball on marker '
                        f'{expected_start}; goto marker {target}.'),
                })
                self._publish_status()
                # Hard pause: always wait for operator confirm before
                # starting each trial. No organic-arrival shortcut —
                # see _wait_for_ball_on_marker docstring for why.
                if not self._wait_for_ball_on_marker(expected_start):
                    self.get_logger().warn(
                        f'  operator never confirmed ball on marker '
                        f'{expected_start} — aborting verification phase')
                    break
                self._sleep_with_abort(STEP_ID_INTER_TRIAL_DWELL_S)
                if self._abort:
                    break
                trial_data = self._run_closed_loop_marker_trial(
                    expected_start, target, i + 1, log_path)
                verification_trials.append(trial_data)
                current_marker = target
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
            time.sleep(0.01)

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

        # Fit parabola to the constant-tilt window. Skip the first
        # STEP_ID_OPEN_LOOP_RAMP_S so the level-PI loop has time to
        # actually reach the commanded tilt; skip the last 0.05 s so
        # we're well inside the constant-tilt window.
        g_eff, td, fit_meta = self._fit_open_loop(
            samples, roll_deg, pitch_deg)
        out['g_eff_mm_s2_per_deg'] = float(g_eff)
        out['td_observed_s'] = float(td)
        out['fit'] = fit_meta

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
        """Fit ball trajectory during the constant-tilt window with a
        parabola: position(t) = x0 + v0·t + ½·a·t². The acceleration
        magnitude divided by sin(commanded_tilt) gives the cascaded
        plant gain G_eff in mm/s² per radian; convert to per-degree
        for human-readable output.

        Dead-time td is estimated as the time from tilt command (t=0)
        to when |ball position| diverges from baseline by > 5 mm.

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
        # If both axes commanded simultaneously (we don't, but the math
        # still works), normalise to a unit vector:
        nrm = math.hypot(ax_dir, ay_dir) or 1.0
        ax_dir /= nrm
        ay_dir /= nrm
        # Project ball position onto the expected motion direction so
        # we fit a 1-D scalar trajectory.
        ts = np.array([s['t'] for s in samples if s['phase'] == 'tilt'])
        xs = np.array([s['x'] for s in samples if s['phase'] == 'tilt'])
        ys = np.array([s['y'] for s in samples if s['phase'] == 'tilt'])
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
        # Dead-time: first sample where |d| > 5 mm.
        td = 0.0
        for ti, di in zip(ts, d):
            if abs(di) > 5.0:
                td = float(ti)
                break
        # Fit window: ramp_s end → end of tilt phase.
        fit_mask = ts >= STEP_ID_OPEN_LOOP_RAMP_S
        if fit_mask.sum() < 5:
            return (0.0, td, {'error': 'too few fit-window samples'})
        t_fit = ts[fit_mask]
        d_fit = d[fit_mask]
        # least-squares fit: d = c0 + c1·t + c2·t²
        try:
            coefs = np.polyfit(t_fit, d_fit, 2)
        except Exception as e:
            return (0.0, td, {'error': f'polyfit failed: {e!r}'})
        a_fit = float(coefs[0]) * 2.0   # because d = ½·a·t² → a = 2·c2
        # Commanded tilt magnitude (deg).
        theta_deg = math.hypot(roll_deg, pitch_deg)
        if theta_deg <= 1e-6:
            return (0.0, td, {'error': 'zero commanded tilt'})
        # G_eff in mm/s²/deg. The fit acceleration is along the
        # expected-motion direction; sign should be positive (ball
        # moved the way we expected). Negative a_fit → wrong sign,
        # which would indicate inverted IK; we report the magnitude
        # but flag it.
        g_eff = abs(a_fit) / theta_deg
        meta = {
            'a_fit_mm_s2': float(a_fit),
            'theta_deg': float(theta_deg),
            'fit_window_s': [float(t_fit[0]), float(t_fit[-1])],
            'n_fit_samples': int(fit_mask.sum()),
            'baseline_xy': [x0, y0],
            'expected_motion_unit': [ax_dir, ay_dir],
            'sign_check_passed': bool(a_fit > 0),
        }
        return (g_eff, td, meta)

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
        while not self._abort:
            now = time.monotonic()
            elapsed = now - t0
            if elapsed > STEP_ID_GOTO_TIMEOUT_S:
                break
            if self._ball_state is not None:
                p = self._ball_state.pose.position
                samples.append({'t': elapsed,
                                'x': float(p.x), 'y': float(p.y)})
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
            time.sleep(0.02)
        # Drop back to LEVEL_HOLD between trials so the platform doesn't
        # keep tilting while the ball is being placed.
        self._publish_mode('LEVEL_HOLD')
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
                'omega_n_rad_s': float(STEP_ID_FALLBACK_OMEGA_N),
                'zeta': float(STEP_ID_TARGET_ZETA),
                'g_eff_used': float(g_eff_mm_s2_per_deg),
                'note': ('G_eff implausibly small; recommendation '
                         'falls back to current gains'),
                'valid': False,
            }
        # Adaptive ωn: min(0.5/Td, OMEGA_N_CAP). 0.5/Td is the rule of
        # thumb that bandwidth shouldn't exceed half the inverse dead-
        # time (else Bode-style phase margin collapses).
        if td_observed_s > 0.05:
            omega_n = min(0.5 / td_observed_s, STEP_ID_OMEGA_N_CAP)
        else:
            omega_n = STEP_ID_OMEGA_N_CAP
        zeta = STEP_ID_TARGET_ZETA
        kp = (omega_n ** 2) / g_eff_mm_s2_per_deg
        kd = 2.0 * zeta * omega_n / g_eff_mm_s2_per_deg
        # Keep the existing Ki — analytic PD doesn't compute it (Ki is
        # for friction/stiction, not bandwidth shaping). Operator can
        # tune it separately if needed.
        ki = float(current_gains.get('ki', 0.001))
        return {
            'kp': float(kp),
            'kd': float(kd),
            'ki': ki,
            'omega_n_rad_s': float(omega_n),
            'zeta': float(zeta),
            'g_eff_used': float(g_eff_mm_s2_per_deg),
            'td_observed_s': float(td_observed_s),
            'note': (
                f'computed from G_eff={g_eff_mm_s2_per_deg:.1f} mm/s²/°, '
                f'Td={td_observed_s:.3f}s, ωn={omega_n:.2f} rad/s, '
                f'ζ={zeta:.2f}'),
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
