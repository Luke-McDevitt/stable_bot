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
from std_msgs.msg import String
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
HOLD_TOL_MM            = 25.0    # f_hold counts samples within this band
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

# Optimizer
NOISE_FLOOR            = 0.02
ANNEAL_TRIGGER         = 3       # consec non-improves → step *= ANNEAL_FACTOR
ANNEAL_FACTOR          = 0.7
SAFETY_BAD_TRIALS      = 5       # 5x f<0.10 → halt
SAFETY_BAD_THRESH      = 0.10
# Lucky-baseline fix: if N consecutive trials are rejected against
# the current best, re-evaluate the current best to get a fresh
# fitness sample. Update best_fitness with a weighted average so
# a one-shot lucky high regresses toward the truth instead of
# trapping the optimizer ("nothing beats trial 0").
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

# Fitness weights (sum should equal 1.0)
W_ERR    = 0.25
W_P95    = 0.15
W_SETTLE = 0.15
W_HOLD   = 0.25
W_CALM   = 0.20


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

        # ROS i/o
        self.create_subscription(
            PoseStamped, '/ball_state', self._on_ball_state, 10)
        self.create_subscription(
            String, '/control_result', self._on_control_result, 50)
        self.pub_cmd = self.create_publisher(String, '/control_cmd', 10)
        # Latched-style status: rosbridge clients pick up the latest
        # snapshot on connect (we'll re-publish it every status update).
        self.pub_status = self.create_publisher(
            String, '/auto_tune/status', 10)

        self.create_service(
            Trigger, '/auto_tune/start', self._srv_start)
        self.create_service(
            Trigger, '/auto_tune/stop', self._srv_stop)
        # Walks tuning_data/auto_tune_*/summary.json, picks the
        # session with the highest best_fitness, applies its gains.
        # Useful after a rough session: "go back to whatever worked
        # best in any past run."
        self.create_service(
            Trigger, '/auto_tune/restore_best', self._do_restore_best)

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

    def _srv_stop(self, req, res):
        if not self._running:
            res.success = False
            res.message = 'auto_tune not running'
            return res
        # Cooperative abort flag for the trial loop.
        self._abort = True
        # IMMEDIATELY return the platform to LEVEL_HOLD — don't wait for
        # the trial loop to notice the flag. Without this, the platform
        # keeps tilting until the current trial's collect-loop ticks
        # again (up to 20 ms), and during a runaway gain set that's
        # 20 ms too long.
        self._publish_mode('LEVEL_HOLD')
        # Belt and braces: also send a zero-tilt set_pose so even if
        # something is mis-mapping LEVEL_HOLD, we explicitly set the
        # platform flat at nominal Z.
        self._reset_pose_to_nominal(blocking=False)
        res.success = True
        res.message = 'auto_tune stopping — platform → LEVEL_HOLD'
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
            consec_no_improve = {k.name: 0 for k in KNOBS}

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

                # Pick a knob round-robin, skipping any sign-knob that
                # has been locked because flipping it lowered fitness
                # (strong evidence the current sign is correct).
                for _ in range(len(KNOBS)):
                    kn = KNOBS[knob_idx % len(KNOBS)]
                    knob_idx += 1
                    if kn.categorical and sign_locked.get(kn.name):
                        continue
                    break
                direction = random.choice([+1, -1])

                if kn.categorical:
                    test_gains = dict(best_gains)
                    test_gains[kn.name] = (
                        -1.0 if best_gains.get(kn.name, 1.0) > 0 else 1.0)
                    step_repr = f'{kn.name} flip → {test_gains[kn.name]:+.0f}'
                else:
                    delta = direction * steps[kn.name]
                    cur = float(best_gains.get(kn.name, 0.0))
                    new = max(kn.lo, min(kn.hi, cur + delta))
                    test_gains = dict(best_gains)
                    test_gains[kn.name] = new
                    step_repr = (f'{kn.name} {cur:.4f} → {new:.4f} '
                                 f'(Δ={delta:+.4f})')

                self.get_logger().info(
                    f'Trial {trial}/{max_trials}: {step_repr}')
                r = self._run_trial(test_gains)
                if r.aborted:
                    self.get_logger().warn(
                        f'  trial aborted: {r.abort_reason} — skipping')
                    continue
                f, c = self._compute_fitness(r)
                accepted = f > best_fitness + NOISE_FLOOR
                if accepted:
                    best_fitness = f
                    best_gains = dict(test_gains)
                    consec_no_improve[kn.name] = 0
                    consec_rejections = 0
                else:
                    consec_rejections += 1
                    consec_no_improve[kn.name] += 1
                    if (not kn.categorical
                            and consec_no_improve[kn.name] >= ANNEAL_TRIGGER):
                        steps[kn.name] *= ANNEAL_FACTOR
                        consec_no_improve[kn.name] = 0
                        self.get_logger().info(
                            f'  step annealed: {kn.name} step → '
                            f'{steps[kn.name]:.4f}')
                    # Sign-flip rejection counter — lock the sign once
                    # SIGN_FLIP_LOCK_AFTER_REJECTS hits. The flip
                    # showed fitness DROPS, so the current sign is
                    # the correct one; future trials don't waste
                    # budget retrying.
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
                    algo='hill_climb')

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
        f_hold   = on_target_frac
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
