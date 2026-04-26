#!/usr/bin/env python3
"""
stewart_control_node — long-lived ROS 2 node that owns the CAN bus and
exposes services for every Phase 5 GUI action.

Owns:
  - CAN bus (socketcan can0, 1 Mb/s)
  - ODriveFeeder thread: per-leg mode-aware Set_Input_Pos / Set_Input_Vel
    at 50 Hz; 'idle' legs are skipped (watchdog doesn't apply when in IDLE)
  - EncoderListener thread (passive + RTR for Get_Encoder_Estimates)
  - Optional PI level-hold loop (background) against the saved
    platform_level.yaml reference
  - Subscription to /platform/imu/data for the IMU readout topic

Services (all under the default /):
  /activate (ActivateOrDeactivate)      : arm or disarm all 6
  /e_stop (Trigger)                     : immediate disarm
  /jog_leg (JogLeg)                     : delta_turns on one leg
  /set_pose (SetPose)                   : xyz+rpy → IK → feeder
  /go_to_rest (Trigger)                 : pose=(0,0,0,0,0,0)
  /set_speed_cap (SetFloat)             : clamp commanded leg velocity
  /get_speed_cap (Trigger)              : returns current in message field
  /enable_level (ActivateOrDeactivate)  : toggle PI level loop
  /start_homing (StartHoming)           : subprocess stall_home.py
  /cancel_homing (Trigger)              : SIGINT subprocess
  /homing_stdin (via topic)             : /homing_stdin_in std_msgs/String

Topics:
  /leg_encoders (Float64MultiArray, 6 values, turns)     @ ~20 Hz
  /platform_rpy (Float32MultiArray, 3 values, degrees)   @ ~20 Hz
  /status (String JSON)                                   @ ~2 Hz
  /homing_output (String)                                 per subprocess line
"""
import datetime
import glob
import json
import math
import os
import re
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np
import yaml

try:
    import can
except ImportError:
    sys.exit("python-can not installed")

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32, Float64MultiArray, Float32MultiArray, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu

from jugglebot_interfaces.srv import (
    ActivateOrDeactivate, SetFloat, JogLeg, SetPose, StartHoming, ArmLeg,
)


# ---------------- Constants -------------------------------------------
def _find_stewart_bringup_dir():
    # Check for package.xml as the signature of a real package dir
    # (a phantom dir created by stall_home will only contain config/).
    for cand in ('~/ros2_ws/src/stewart_bringup',
                 '~/ros2_ws/src/stable_bot/stewart_bringup'):
        p = os.path.expanduser(cand)
        if os.path.isfile(os.path.join(p, 'package.xml')):
            return p
    return os.path.expanduser('~/ros2_ws/src/stewart_bringup')
_BRINGUP_DIR = _find_stewart_bringup_dir()
LEG_LIMITS_PATH = os.path.join(_BRINGUP_DIR, 'config/leg_limits.yaml')
GLOBAL_LIMITS_PATH = os.path.join(_BRINGUP_DIR, 'config/global_limits.yaml')
LEVEL_CAL_PATH = os.path.join(_BRINGUP_DIR, 'config/platform_level.yaml')
STALL_HOME_SCRIPT = os.path.expanduser(
    '~/Getting the robot working/Spin Motor Over CAN Test/stall_home.py')
ROUTINES_DIR = os.path.join(_BRINGUP_DIR, 'config/routines')
ROUTINE_LOGS_DIR = os.path.join(_BRINGUP_DIR, 'logs')
SAFETY_MARGIN_TURNS = 0.05

# IK geometry (match robot_geometry.py)
INITIAL_HEIGHT = 777.517
BASE_RADIUS = 410.0
PLAT_RADIUS = 200.0
BASE_SMALL_ANGLE = 20.0
PLAT_SMALL_ANGLE = 9.0
PLAT_X_AXIS_OFFSET = 154.3012223
MM_PER_REV = 71.047
MOTOR_EXTENSION_SIGN = -1

# CAN command IDs (ODrive Pro 0.6.x)
CMD_HEARTBEAT         = 0x001
CMD_GET_ERROR         = 0x003
CMD_SET_AXIS_STATE    = 0x007
CMD_GET_ENCODER       = 0x009
CMD_SET_CONTROLLER_MODE = 0x00B
CMD_SET_INPUT_POS     = 0x00C
CMD_SET_INPUT_VEL     = 0x00D
CMD_SET_LIMITS        = 0x00F
CMD_CLEAR_ERRORS      = 0x018
CMD_GET_IQ            = 0x014

# ODrive Pro 0.6.x active_errors bit flags (same as read_odrive_errors.py)
ODRIVE_ERROR_FLAGS = {
    0x00000001: "INITIALIZING",
    0x00000002: "SYSTEM_LEVEL",
    0x00000004: "TIMING_ERROR",
    0x00000008: "MISSING_ESTIMATE",
    0x00000010: "BAD_CONFIG",
    0x00000020: "DRV_FAULT",
    0x00000040: "MISSING_INPUT",
    0x00000100: "DC_BUS_OVER_VOLTAGE",
    0x00000200: "DC_BUS_UNDER_VOLTAGE",
    0x00000400: "DC_BUS_OVER_CURRENT",
    0x00000800: "DC_BUS_OVER_REGEN_CURRENT",
    0x00001000: "CURRENT_LIMIT_VIOLATION",
    0x00002000: "MOTOR_OVER_TEMP",
    0x00004000: "INVERTER_OVER_TEMP",
    0x00008000: "VELOCITY_LIMIT_VIOLATION",
    0x00010000: "POSITION_LIMIT_VIOLATION",
    0x01000000: "WATCHDOG_TIMER_EXPIRED",
    0x02000000: "ESTOP_REQUESTED",
    0x04000000: "SPINOUT_DETECTED",
    0x08000000: "BRAKE_RESISTOR_DISARMED",
    0x10000000: "THERMISTOR_DISCONNECTED",
    0x20000000: "CALIBRATION_ERROR",
}


def _decode_error_bits(bits):
    if bits == 0:
        return "OK"
    names = [name for mask, name in ODRIVE_ERROR_FLAGS.items() if bits & mask]
    leftover = bits
    for mask in ODRIVE_ERROR_FLAGS:
        leftover &= ~mask
    if leftover:
        names.append(f"UNKNOWN(0x{leftover:08X})")
    return " | ".join(names)
STATE_IDLE = 1
STATE_CLOSED_LOOP = 8
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP = 2

# PI level gains (same as platform_move)
LEVEL_KP = 0.7
LEVEL_KI = 0.4
LEVEL_FILTER_ALPHA = 0.3
LEVEL_RATE_LIMIT = 0.2
LEVEL_MAX_CORR = 5.0
LEVEL_DEADBAND = 0.05
CTRL_PERIOD_S = 0.02


# ---------------- Geometry + IK ---------------------------------------
def _build_platform():
    d2r = math.pi / 180
    gamma2 = BASE_SMALL_ANGLE
    gamma0 = 210 - gamma2 / 2
    gamma1 = 120 - gamma2
    lambda1 = PLAT_SMALL_ANGLE
    lambda2 = 120 - lambda1
    lambda0 = PLAT_X_AXIS_OFFSET
    base = np.zeros((6, 3))
    plat = np.zeros((6, 3))
    for i in range(6):
        first = i // 2
        second = (i + 1) // 2
        ba = gamma0 + gamma1 * first + gamma2 * second
        pa = lambda0 + lambda1 * first + lambda2 * second
        base[i] = [BASE_RADIUS * math.cos(ba * d2r),
                   BASE_RADIUS * math.sin(ba * d2r), 0]
        plat[i] = [PLAT_RADIUS * math.cos(pa * d2r),
                   PLAT_RADIUS * math.sin(pa * d2r), 0]
    start = np.array([[0], [0], [INITIAL_HEIGHT]])
    init_ll = np.linalg.norm(plat + start.T - base, axis=1)
    return base, plat, init_ll, start


def _rot_rpy(r, p, y):
    r, p, y = map(math.radians, (r, p, y))
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _leg_extensions_mm(xyz, rpy, geom):
    base, plat, init_ll, start = geom
    pos = np.array([[xyz[0]], [xyz[1]], [xyz[2]]])
    R = _rot_rpy(*rpy)
    new_pos = pos + start
    new_plat = (new_pos + R @ plat.T).T
    return np.linalg.norm(new_plat - base, axis=1) - init_ll


def _quat_to_rpy_deg(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _send_cmd(bus, node_id, cmd_id, data):
    arb = (node_id << 5) | cmd_id
    bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False),
             timeout=0.1)


def _send_pos(bus, n, target, vel_ff_tps=0.0, torque_ff_nm=0.0):
    """Set_Input_Pos with optional velocity + torque feed-forward.
    ODrive Pro CAN format: (pos_f32, vel_ff_i16, torque_ff_i16) where the
    i16 fields are scaled by 1000 — so sending 1500 means +1.5 turns/s."""
    vel_i16 = max(-32768, min(32767, int(round(vel_ff_tps * 1000.0))))
    trq_i16 = max(-32768, min(32767, int(round(torque_ff_nm * 1000.0))))
    _send_cmd(bus, n, CMD_SET_INPUT_POS,
              struct.pack('<fhh', float(target), vel_i16, trq_i16))


def _send_vel(bus, n, vel_turns_per_s, torque_ff=0.0):
    _send_cmd(bus, n, CMD_SET_INPUT_VEL,
              struct.pack('<ff', float(vel_turns_per_s), float(torque_ff)))


def _load_leg_limits():
    with open(LEG_LIMITS_PATH) as f:
        data = yaml.safe_load(f) or {}
    out = {}
    for n in range(6):
        ax = data.get(f'axis_{n}', {})
        mn, mx, rp = ax.get('min_pos_turns'), ax.get('max_pos_turns'), ax.get('rest_pos_turns')
        if mn is None or mx is None or rp is None:
            return None
        out[n] = {
            'rest': float(rp),
            'lo': min(float(mn), float(mx)) + SAFETY_MARGIN_TURNS,
            'hi': max(float(mn), float(mx)) - SAFETY_MARGIN_TURNS,
        }
    return out


def _load_global_limits():
    with open(GLOBAL_LIMITS_PATH) as f:
        return yaml.safe_load(f) or {}


def _load_level_cal():
    if not os.path.exists(LEVEL_CAL_PATH):
        return None
    with open(LEVEL_CAL_PATH) as f:
        doc = yaml.safe_load(f) or {}
    return float(doc.get('ref_roll_deg', 0.0)), float(doc.get('ref_pitch_deg', 0.0))


def _compute_motor_targets(xyz, rpy, geom, limits):
    ext = _leg_extensions_mm(xyz, rpy, geom)
    targets = np.zeros(6)
    any_clamped = False
    for i in range(6):
        ext_turns = MOTOR_EXTENSION_SIGN * ext[i] / MM_PER_REV
        raw = limits[i]['rest'] + ext_turns
        lo, hi = limits[i]['lo'], limits[i]['hi']
        if raw < lo:
            targets[i] = lo
            any_clamped = True
        elif raw > hi:
            targets[i] = hi
            any_clamped = True
        else:
            targets[i] = raw
    return targets, any_clamped


# ---------------- Background threads ----------------------------------
class EncoderListener:
    """Passively listens for both encoder (0x009) and error (0x003)
    broadcasts, and issues periodic RTRs for both so freshness is
    guaranteed regardless of ODrive cyclic-broadcast config."""
    def __init__(self, bus, rtr_period=0.1, error_rtr_period=0.5):
        self.bus = bus
        self.rtr_period = rtr_period
        self.error_rtr_period = error_rtr_period
        self.lock = threading.Lock()
        self.pos_by_node = {}     # node -> (pos_turns, rx_monotonic)
        self.err_by_node = {}     # node -> (active_errors, disarm_reason, rx)
        self.iq_by_node = {}      # node -> (iq_setpoint, iq_measured, rx)
        self.stop_flag = threading.Event()
        self._rx = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx = threading.Thread(target=self._tx_loop, daemon=True)

    def start(self):
        self._rx.start()
        self._tx.start()

    def stop(self):
        self.stop_flag.set()
        self._rx.join(timeout=2.0)
        self._tx.join(timeout=2.0)

    def _rx_loop(self):
        while not self.stop_flag.is_set():
            try:
                msg = self.bus.recv(timeout=0.1)
            except Exception:
                continue
            if msg is None or msg.is_remote_frame:
                continue
            cmd = msg.arbitration_id & 0x1F
            node = msg.arbitration_id >> 5
            if cmd == CMD_GET_ENCODER and len(msg.data) >= 4:
                pos = struct.unpack('<f', bytes(msg.data[:4]))[0]
                with self.lock:
                    self.pos_by_node[node] = (pos, time.monotonic())
            elif cmd == CMD_GET_ERROR and len(msg.data) >= 8:
                active = struct.unpack('<I', bytes(msg.data[0:4]))[0]
                disarm = struct.unpack('<I', bytes(msg.data[4:8]))[0]
                with self.lock:
                    self.err_by_node[node] = (active, disarm, time.monotonic())
            elif cmd == CMD_GET_IQ and len(msg.data) >= 8:
                iq_sp = struct.unpack('<f', bytes(msg.data[0:4]))[0]
                iq_m  = struct.unpack('<f', bytes(msg.data[4:8]))[0]
                with self.lock:
                    self.iq_by_node[node] = (iq_sp, iq_m, time.monotonic())

    def _tx_loop(self):
        last_err_rtr = 0.0
        while not self.stop_flag.is_set():
            # Always request encoder at 10 Hz
            for n in range(6):
                try:
                    arb = (n << 5) | CMD_GET_ENCODER
                    self.bus.send(can.Message(arbitration_id=arb,
                                              is_remote_frame=True,
                                              is_extended_id=False, dlc=8),
                                  timeout=0.1)
                except Exception:
                    pass
            now = time.monotonic()
            if now - last_err_rtr >= self.error_rtr_period:
                last_err_rtr = now
                for n in range(6):
                    try:
                        # Get_Error
                        self.bus.send(can.Message(
                            arbitration_id=(n << 5) | CMD_GET_ERROR,
                            is_remote_frame=True, is_extended_id=False,
                            dlc=8), timeout=0.1)
                        # Get_Iq (piggy-back on the error cadence)
                        self.bus.send(can.Message(
                            arbitration_id=(n << 5) | CMD_GET_IQ,
                            is_remote_frame=True, is_extended_id=False,
                            dlc=8), timeout=0.1)
                    except Exception:
                        pass
            time.sleep(self.rtr_period)

    def get_all(self, max_age_s=0.3):
        out = [None] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.pos_by_node.get(n)
                if v is not None and (now - v[1]) < max_age_s:
                    out[n] = v[0]
        return out

    def get_errors(self, max_age_s=2.0):
        """Returns list of 6 tuples (active_errors, disarm_reason) or None
        if no fresh reading. Includes decoded names as a third element."""
        out = [None] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.err_by_node.get(n)
                if v is not None and (now - v[2]) < max_age_s:
                    out[n] = (int(v[0]), int(v[1]))
        return out

    def get_iq(self, max_age_s=2.0):
        """Returns list of 6 iq_measured floats (A) or NaN if no fresh data."""
        out = [float('nan')] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.iq_by_node.get(n)
                if v is not None and (now - v[2]) < max_age_s:
                    out[n] = float(v[1])
        return out


class ODriveFeeder:
    """50 Hz background publisher. Per-leg mode is one of:
        'pos'  -> send Set_Input_Pos(pos_targets[n])
        'vel'  -> send Set_Input_Vel(shared vel_target, 0)
        'idle' -> send nothing (axis is in STATE_IDLE, watchdog not active)
    This unifies the pose-mode holding loop and the dead-man velocity jog."""

    def __init__(self, bus, initial_targets, period=0.02):
        self.bus = bus
        self.period = period
        self.lock = threading.Lock()
        self.pos_targets = np.array(initial_targets, dtype=float).copy()
        self.vel_target = 0.0
        self.modes = ['idle'] * 6
        self.stop_flag = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_flag.set()
        self.thread.join(timeout=2.0)

    def set_mode(self, n, mode):
        assert mode in ('pos', 'vel', 'idle')
        with self.lock:
            self.modes[n] = mode

    def set_all_modes(self, mode):
        assert mode in ('pos', 'vel', 'idle')
        with self.lock:
            self.modes = [mode] * 6

    def get_modes(self):
        with self.lock:
            return list(self.modes)

    def any_in_mode(self, mode):
        with self.lock:
            return any(m == mode for m in self.modes)

    def set_pos_targets(self, targets):
        with self.lock:
            self.pos_targets = np.array(targets, dtype=float).copy()

    def set_pos_target_one(self, n, target):
        with self.lock:
            self.pos_targets[n] = float(target)

    def get_pos_targets(self):
        with self.lock:
            return self.pos_targets.copy()

    def set_vel_target(self, v):
        with self.lock:
            self.vel_target = float(v)

    def get_vel_target(self):
        with self.lock:
            return self.vel_target

    def _run(self):
        # Backoff state: when many consecutive sends fail (usually means
        # CAN is bus-off because ODrives latched errors and stopped ACKing),
        # pause the feeder briefly instead of spamming the TX queue. That
        # lets the kernel drain and recovery commands through.
        consecutive_fails = 0
        backoff_until = 0.0
        # Velocity feed-forward: finite-difference the commanded position
        # targets so the ODrive's position controller gets both "where to
        # be" and "how fast to get there". This collapses dynamic tracking
        # error by ~10× on sinusoidal inputs — without vel_ff a PI with
        # modest gains inherently lags a moving reference.
        prev_pos_t = None
        prev_t = None
        VEL_FF_FILTER = 0.5  # exponential smoothing on FF to kill noise
        vel_ff_filt = np.zeros(6)
        while not self.stop_flag.is_set():
            t0 = time.monotonic()
            if t0 < backoff_until:
                time.sleep(min(0.1, backoff_until - t0))
                continue
            with self.lock:
                modes = list(self.modes)
                pos_t = self.pos_targets.copy()
                vel = self.vel_target
            # Compute finite-difference velocity (turns/s) of the target
            if prev_pos_t is not None and prev_t is not None:
                dt = t0 - prev_t
                if dt > 1e-4:
                    inst_vel_ff = (pos_t - prev_pos_t) / dt
                    # Clamp absurd spikes (e.g. first sample after a step
                    # change in target) to keep motors from jerking.
                    np.clip(inst_vel_ff, -10.0, 10.0, out=inst_vel_ff)
                    vel_ff_filt = (VEL_FF_FILTER * inst_vel_ff
                                   + (1 - VEL_FF_FILTER) * vel_ff_filt)
            any_attempted = False
            any_failed = False
            for n in range(6):
                m = modes[n]
                if m == 'idle':
                    continue
                any_attempted = True
                try:
                    if m == 'pos':
                        _send_pos(self.bus, n, float(pos_t[n]),
                                  vel_ff_tps=float(vel_ff_filt[n]))
                    elif m == 'vel':
                        _send_vel(self.bus, n, vel, 0.0)
                except Exception:
                    any_failed = True
            prev_pos_t = pos_t
            prev_t = t0
            if any_attempted:
                if any_failed:
                    consecutive_fails += 1
                    if consecutive_fails >= 20:
                        backoff_until = time.monotonic() + 0.25
                        consecutive_fails = 0
                else:
                    consecutive_fails = 0
            sl = self.period - (time.monotonic() - t0)
            if sl > 0:
                time.sleep(sl)


# ---------------- The node --------------------------------------------
class StewartControlNode(Node):
    def __init__(self):
        super().__init__('stewart_control_node')
        self.get_logger().info("starting stewart_control_node")

        self.geom = _build_platform()
        self.limits = _load_leg_limits()
        if self.limits is None:
            self.get_logger().error(
                f"{LEG_LIMITS_PATH} not homed — jog/pose services will fail "
                f"until /start_homing completes. Starting node anyway.")
        self.global_limits = _load_global_limits()
        self.hard_max_vel = float(self.global_limits.get(
            'hard_max_leg_vel_turns_per_sec', 2.0))
        self.soft_max_vel = float(self.global_limits.get(
            'default_soft_max_leg_vel_turns_per_sec', 1.0))

        # CAN bus — opened lazily so that homing subprocess can take it
        self.bus = None
        self.bus_lock = threading.Lock()
        self.feeder = None
        self.listener = None
        # captured rest positions (per-leg encoder snapshot from the GUI's
        # "Capture rest from current encoders" button). Used as the seed
        # for safe arming when leg_limits.yaml doesn't exist yet (e.g.
        # before homing has succeeded). None when no capture has been
        # made; otherwise a list of 6 floats (turns).
        self.captured_rest_positions = None

        # Manual endstop captures (alternative to automatic stall-homing).
        # Per-leg, so the user can capture in groups (e.g., the platform may
        # be unstable with all 6 legs at the top simultaneously, so they
        # capture top one or two legs at a time). Each slot is None before
        # capture, or a float (turns). Save merges with existing leg_limits.yaml
        # so previously-saved legs aren't clobbered.
        self.captured_endstops = {
            'bottom': [None] * 6,
            'top':    [None] * 6,
        }

        # per-leg armed state (True iff the leg is in CLOSED_LOOP, in either
        # pos or vel mode).
        self.leg_armed = [False] * 6
        # Track current-limit as-last-applied so the recording can correlate
        # motor behavior with the current cap. Default 6 A matches the
        # arm-time default in _arm_leg_internal.
        self.leg_current_a = 6.0

        # Current pose state (held by feeder)
        self.current_xyz = [0.0, 0.0, 0.0]
        self.current_rpy = [0.0, 0.0, 0.0]

        # Level-hold state
        self.level_enabled = False
        self.level_thread = None
        self.level_stop = threading.Event()
        self.level_ref_roll = 0.0
        self.level_ref_pitch = 0.0
        self.level_corr = [0.0, 0.0]

        # Latest IMU (rpy, last_rx)
        self.imu_lock = threading.Lock()
        self.imu_rpy = None
        self.imu_last_rx = 0.0
        self.imu_accel = None    # (ax, ay, az) m/s^2
        self.imu_gyro = None     # (gx, gy, gz) rad/s

        # Homing subprocess
        self.homing_proc = None
        self.homing_lock = threading.Lock()
        # CAN reset worker (runs in thread so we don't block rclpy)
        self.reset_can_thread = None

        # Routine recording / playback
        os.makedirs(ROUTINES_DIR, exist_ok=True)
        os.makedirs(ROUTINE_LOGS_DIR, exist_ok=True)
        self.recording_t0 = None          # monotonic() or None if not recording
        self.recording_keyframes = []     # [{t,x,y,z,roll,pitch,yaw}, ...]
        self.routine_player_thread = None
        self.routine_player_stop = threading.Event()
        self.playing_routine_name = None
        self.playing_routine_elapsed = 0.0
        self.playing_routine_duration = 0.0
        # Data logging during routine playback (the big one for offline
        # ball-on-disk simulation + tuning). Populated when play_routine
        # is called with record=true.
        self.rlog_dir = None
        self.rlog_csv = None             # open file handle
        self.rlog_can_proc = None        # candump subprocess
        self.rlog_timer = None           # rclpy timer handle
        self.rlog_t0 = 0.0

        # --- Pubs ---
        self.pub_encoders = self.create_publisher(
            Float64MultiArray, 'leg_encoders', 10)
        self.pub_currents = self.create_publisher(
            Float64MultiArray, 'leg_currents', 10)
        self.pub_rpy = self.create_publisher(
            Float32MultiArray, 'platform_rpy', 10)
        self.pub_status = self.create_publisher(String, 'status', 10)
        self.pub_homing_out = self.create_publisher(
            String, 'homing_output', 100)
        # Topic-based command bus. Primary path the GUI uses because
        # rosbridge + Fast DDS in WSL2 can't reliably see services from
        # other processes (topic discovery works fine).
        self.pub_control_result = self.create_publisher(
            String, 'control_result', 20)
        # Per-node decoded error state, published on request + ~1 Hz.
        self.pub_errors = self.create_publisher(String, 'odrive_errors', 10)

        # Dead-man jog watchdog: if /jog_vel_cmd stops arriving for > 0.5 s
        # while any leg is in vel mode, force vel target back to 0 so the
        # motor cannot run away if the browser disconnects mid-drag.
        self._last_jog_vel_rx = 0.0
        self._last_jog_vel_value = 0.0

        # --- Subs ---
        self.create_subscription(
            Imu, '/platform/imu/data', self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            String, 'homing_stdin_in', self._homing_stdin_cb, 10)

        # --- Services ---
        self.create_service(ActivateOrDeactivate, 'activate',
                            self.srv_activate)
        self.create_service(ArmLeg, 'arm_leg', self.srv_arm_leg)
        self.create_service(Trigger, 'e_stop', self.srv_e_stop)
        self.create_service(Trigger, 'go_to_rest', self.srv_go_to_rest)
        self.create_service(JogLeg, 'jog_leg', self.srv_jog_leg)
        self.create_service(SetPose, 'set_pose', self.srv_set_pose)
        self.create_service(SetFloat, 'set_speed_cap', self.srv_set_speed_cap)
        self.create_service(Trigger, 'get_speed_cap', self.srv_get_speed_cap)
        self.create_service(ActivateOrDeactivate, 'enable_level',
                            self.srv_enable_level)
        self.create_service(StartHoming, 'start_homing', self.srv_start_homing)
        self.create_service(Trigger, 'cancel_homing', self.srv_cancel_homing)

        # Dead-man velocity slider feeds into this topic. On slider release,
        # GUI publishes 0. On no message for > 0.5 s, we'd want to zero for
        # safety — that's a future enhancement; for now we trust the slider.
        self.create_subscription(
            Float32, 'jog_vel_cmd', self._jog_vel_cmd_cb, 10)
        # Topic-based alternative to every service (see _control_cmd_cb).
        self.create_subscription(
            String, 'control_cmd', self._control_cmd_cb, 10)

        # --- Timers ---
        self.create_timer(0.05, self._tick_state)      # 20 Hz encoders/rpy
        self.create_timer(0.5, self._tick_status)       # 2 Hz status

        self._open_bus_and_start_threads()
        self.get_logger().info("stewart_control_node ready.")

    # ---- convenience properties ----
    @property
    def armed(self):
        """True if any leg is currently in CLOSED_LOOP (pos or vel)."""
        return any(self.leg_armed)

    # ---- bus / thread lifecycle ----
    def _open_bus_and_start_threads(self):
        with self.bus_lock:
            if self.bus is not None:
                return True
            try:
                self.bus = can.Bus(interface='socketcan', channel='can0',
                                   bitrate=1_000_000,
                                   state=can.BusState.ACTIVE)
            except Exception as e:
                self.get_logger().error(f"can't open can0: {e}")
                self.bus = None
                return False
            if self.limits is not None:
                rest_targets, _ = _compute_motor_targets(
                    (0, 0, 0), (0, 0, 0), self.geom, self.limits)
            else:
                rest_targets = np.zeros(6)
            self.feeder = ODriveFeeder(self.bus, rest_targets)
            self.feeder.start()
            self.listener = EncoderListener(self.bus)
            self.listener.start()
            # Clear any latched errors so activate() won't fail.
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_CLEAR_ERRORS, b'\x00')
                except Exception:
                    pass
            return True

    def _close_bus_and_stop_threads(self):
        """Disarm + tear down everything so a subprocess (stall_home) can
        take the bus. Called before spawning homing."""
        if self.armed:
            self._disarm_internal()
        with self.bus_lock:
            if self.feeder is not None:
                self.feeder.stop()
                self.feeder = None
            if self.listener is not None:
                self.listener.stop()
                self.listener = None
            if self.bus is not None:
                try:
                    self.bus.shutdown()
                except Exception:
                    pass
                self.bus = None

    # ---- arm / disarm ----
    def _arm_leg_internal(self, n, mode, current=6.0, vel_limit=None,
                          force_no_limits=False):
        """Arm a single leg in the given mode ('pos' or 'vel'), or disarm
        (mode='idle'). Feeder modes are updated BEFORE the ODrive state
        transition so watchdog stays fed through the transition.

        force_no_limits=True bypasses the leg_limits.yaml check for pos
        mode. Only safe when the caller is going to hold the leg at its
        CURRENT encoder reading and not command any new positions —
        no soft-limit clamping happens, so any subsequent /set_pose or
        /jog_leg can still drive into mechanical endstops. Used by the
        'safe arm' path for pre-homing lock-in-place (spec: closed-loop
        ball demos discussion 2026-04-26)."""
        assert mode in ('pos', 'vel', 'idle')
        if self.bus is None:
            if not self._open_bus_and_start_threads():
                return False, "can't open can0"
        if mode == 'idle':
            with self.bus_lock:
                try:
                    _send_cmd(self.bus, n, CMD_SET_AXIS_STATE,
                              struct.pack('<I', STATE_IDLE))
                except Exception:
                    pass
            time.sleep(0.05)
            # Now stop feeding that leg in whatever mode it was
            if self.feeder is not None:
                self.feeder.set_mode(n, 'idle')
            self.leg_armed[n] = False
            return True, f"leg {n} disarmed"

        # mode in ('pos', 'vel')
        if mode == 'pos' and self.limits is None and not force_no_limits:
            return False, "pos mode needs leg_limits.yaml — run homing first"
        if vel_limit is None:
            vel_limit = self.soft_max_vel * 1.5

        # ----- CRITICAL SAFETY: seed the feeder target -----
        # The feeder streams Set_Input_Pos at 50 Hz with self.pos_targets[n].
        # If we don't seed this to the leg's current encoder reading BEFORE
        # the ODrive enters CLOSED_LOOP, the feeder will command whatever
        # was in pos_targets[n] previously — which could be a stale value
        # from an OLD leg_limits.yaml or from a previous arm cycle. The
        # ODrive will then dutifully drive the leg from its current
        # position to that stale target, which can be many turns away —
        # a slam. This was the bug reported on 2026-04-26 where L1 slammed
        # through the foam after a fresh save_limits.
        #
        # We REFUSE to arm if:
        #   - feeder/listener aren't running
        #   - the encoder reading is stale (> 1 s old)
        # rather than silently fall through with a stale target.
        if self.feeder is None or self.listener is None:
            return False, (
                f"leg {n}: arm refused — feeder or encoder listener not "
                f"running. Try Diagnostics → Hard reset stack.")
        if mode == 'pos':
            enc = self.listener.get_all(max_age_s=1.0)
            if enc[n] is None:
                return False, (
                    f"leg {n}: arm REFUSED — no fresh encoder reading "
                    f"(>1 s stale). Refusing to use a stale pos target "
                    f"as that can drive the leg to a previous position "
                    f"and slam an endstop. Verify the ODrive is powered "
                    f"and on the CAN bus, then retry.")
            seed = enc[n]
            if (force_no_limits
                    and self.captured_rest_positions is not None
                    and self.captured_rest_positions[n] is not None):
                seed = self.captured_rest_positions[n]
            self.feeder.set_pos_target_one(n, seed)
        # Start feeding in the target mode BEFORE state=8 so watchdog is
        # fed continuously through the transition.
        self.feeder.set_mode(n, mode)

        if mode == 'pos':
            ctrl_mode = CONTROL_MODE_POSITION
            input_mode = INPUT_MODE_PASSTHROUGH
        else:
            ctrl_mode = CONTROL_MODE_VELOCITY
            input_mode = INPUT_MODE_VEL_RAMP

        with self.bus_lock:
            try:
                _send_cmd(self.bus, n, CMD_CLEAR_ERRORS, b'\x00')
                time.sleep(0.02)
                _send_cmd(self.bus, n, CMD_SET_LIMITS,
                          struct.pack('<ff', vel_limit, current))
                time.sleep(0.02)
                _send_cmd(self.bus, n, CMD_SET_CONTROLLER_MODE,
                          struct.pack('<II', ctrl_mode, input_mode))
                time.sleep(0.02)
                _send_cmd(self.bus, n, CMD_SET_AXIS_STATE,
                          struct.pack('<I', STATE_CLOSED_LOOP))
            except Exception as e:
                return False, f"arm send failed: {e}"
        time.sleep(0.1)
        self.leg_armed[n] = True
        return True, f"leg {n} armed ({mode})"

    def _arm_all_in_pos_mode(self, current=6.0, vel_limit=None,
                             force_no_limits=False):
        # Acceptance gate: need EITHER homing-derived limits, OR a captured
        # rest snapshot (set via "Capture rest from current encoders" in
        # the GUI), OR an explicit force_no_limits override.
        have_limits = self.limits is not None
        have_rest = self.captured_rest_positions is not None
        if not have_limits and not force_no_limits and not have_rest:
            return False, (
                "no leg_limits.yaml and no captured rest positions. "
                "Either run homing, or press 'Capture rest from current "
                "encoders' in the Homing panel first.")
        # If we're falling back to captured rest (no limits but rest is set),
        # treat this as force_no_limits internally so _arm_leg_internal
        # uses the captured seed.
        use_force = force_no_limits or (not have_limits and have_rest)
        msgs = []
        for n in range(6):
            ok, m = self._arm_leg_internal(
                n, 'pos', current, vel_limit, force_no_limits=use_force)
            msgs.append(m)
            if not ok:
                return False, f"leg {n}: {m}"
        return True, ("all 6 armed (pos mode, holding " +
                      ("at captured rest positions" if use_force and have_rest
                       else "current position") + ")")

    def _capture_rest_positions(self):
        """Snapshot the current encoder readings into self.captured_rest_positions.
        Used as the seed for safe arming when leg_limits.yaml hasn't been
        produced yet (pre-homing flow).

        After capture: the regular Arm button (and srv_activate, and
        cmd:activate) will succeed even without limits, holding each leg
        at its captured position.

        Capture is non-destructive: it just records the current encoder
        readings into memory. To clear, press 'Capture rest' again with
        the legs in a different (also-safe) position, or call
        cmd:clear_rest. After homing succeeds, leg_limits.yaml takes
        precedence and captured_rest is no longer consulted.
        """
        if self.bus is None:
            if not self._open_bus_and_start_threads():
                return False, "can't open can0", None
        if self.listener is None:
            return False, "encoder listener not running", None
        enc = self.listener.get_all(max_age_s=1.0)
        missing = [n for n in range(6) if enc[n] is None]
        if missing:
            return False, (
                f"capture refused: legs {missing} have no fresh encoder "
                f"reading. Power-cycle the ODrive(s) and try again."), None
        self.captured_rest_positions = list(enc)
        formatted = ", ".join(
            f"L{n}={enc[n]:+.4f}" for n in range(6))
        return True, (
            f"captured rest positions: {formatted}. Now press the regular "
            f"Arm button to lock the legs at these positions."), enc

    def _capture_endstop(self, which, legs=None):
        """Snapshot current encoder readings as one of the two endstops.

        which in {'bottom', 'top'}.
        legs: list of leg indices (0–5) to capture, or None for all 6.

        User flow: physically position the SELECTED legs at the desired
        endstop (e.g., bottom = each leg resting on its foam block above
        the actual endstop), then call this. No motor movement. Captures
        are stored per-leg and merge — capturing legs [0, 1] then [2, 3]
        gives you bottom captures for all four; captures for legs not in
        `legs` are left untouched.
        """
        if which not in ('bottom', 'top'):
            return False, f"which must be 'bottom' or 'top'; got {which!r}", None
        if legs is None:
            legs = list(range(6))
        else:
            legs = list(legs)
        bad = [n for n in legs if not (0 <= n <= 5)]
        if bad:
            return False, f"invalid leg indices: {bad}", None
        if not legs:
            return False, "no legs selected", None
        if self.bus is None:
            if not self._open_bus_and_start_threads():
                return False, "can't open can0", None
        if self.listener is None:
            return False, "encoder listener not running", None
        enc = self.listener.get_all(max_age_s=1.0)
        missing = [n for n in legs if enc[n] is None]
        if missing:
            return False, (
                f"capture refused: legs {missing} have no fresh encoder "
                f"reading. Power-cycle the ODrive(s) and try again."), None
        for n in legs:
            self.captured_endstops[which][n] = float(enc[n])
        formatted = ", ".join(
            f"L{n}={enc[n]:+.4f}" for n in legs)
        return True, (
            f"captured {which.upper()} endstop for legs {legs}: {formatted}. "
            f"Capture more legs or press 'Save limits' to write the file."), enc

    def _save_limits_from_captures(self, rest_offset_turns=0.20,
                                   default_stroke_turns=3.0):
        """Write leg_limits.yaml from per-leg captures.

        For each leg:
          - If only BOTTOM captured: derive top = bottom - default_stroke_turns
            (assumes positive=down per the user's encoder convention; project
            memory: 'Per-leg motion sign conventions — Node 0: positive vel
            = downward'). The 'fixed_stroke_turns' approach is the documented
            homing strategy — see project_jugglebot_homing_strategy.md.
          - If only TOP captured: derive bottom = top + default_stroke_turns.
          - If both captured: use them directly.
          - If neither: skip — the leg's existing entry in leg_limits.yaml
            (if any) is preserved.

        rest_offset_turns:    rest = bottom + sign-toward-top * offset
        default_stroke_turns: |top - bottom| used when only one endstop is
                              captured for a given leg.

        After writing, self.limits is reloaded.
        """
        if rest_offset_turns <= 0:
            return False, "rest_offset_turns must be > 0"
        if default_stroke_turns <= 0:
            return False, "default_stroke_turns must be > 0"
        bot = self.captured_endstops['bottom']
        top = self.captured_endstops['top']

        # Load existing leg_limits so legs without new captures are preserved.
        existing = {}
        if os.path.isfile(LEG_LIMITS_PATH):
            try:
                with open(LEG_LIMITS_PATH) as f:
                    existing = yaml.safe_load(f) or {}
            except Exception:
                existing = {}

        out = dict(existing)  # start from existing, override per-leg as we go
        legs_written = []
        legs_skipped = []
        legs_derived = []  # legs where one endstop was derived from default_stroke
        ts = datetime.datetime.utcnow().isoformat(timespec='seconds') + '+00:00'
        for n in range(6):
            b, t = bot[n], top[n]
            if b is None and t is None:
                # Nothing captured for this leg — preserve any existing entry
                # (already there in `out` from the existing dict).
                if f'axis_{n}' in out:
                    legs_skipped.append(n)
                continue

            # Default sign convention from project memory: positive=down,
            # so up direction = negative. If both captured, use observed sign.
            if b is not None and t is not None:
                if abs(t - b) < 0.01:
                    return False, (
                        f"leg {n}: bottom and top capture are too close "
                        f"({t - b:+.4f} turns). Did you capture them at "
                        f"the same physical position?")
                up_dir = 1.0 if (t - b) > 0 else -1.0
            else:
                # Convention assumption: positive encoder = leg extended
                # downward → top is in the negative direction from bottom.
                up_dir = -1.0
                if b is not None:
                    t = b + up_dir * default_stroke_turns
                else:  # t is not None, b is None
                    b = t - up_dir * default_stroke_turns
                legs_derived.append(n)

            rest = b + up_dir * rest_offset_turns
            out[f'axis_{n}'] = {
                'min_pos_turns': float(b),
                'max_pos_turns': float(t),
                'rest_pos_turns': float(rest),
                'homed_at': ts,
                'source': 'manual_endstop_capture',
                'rest_offset_turns': float(rest_offset_turns),
                'derived_top': n in legs_derived if t is not None and bot[n] is not None and top[n] is None else False,
            }
            legs_written.append(n)

        if not legs_written:
            return False, (
                "no new captures — capture at least one leg's BOTTOM or "
                "TOP before saving.")

        try:
            with open(LEG_LIMITS_PATH, 'w') as f:
                yaml.safe_dump(out, f, sort_keys=False)
        except Exception as e:
            return False, f"failed to write {LEG_LIMITS_PATH}: {e}"
        # Reload limits so /set_pose, /jog_leg, /activate work immediately.
        try:
            self.limits = _load_leg_limits()
        except Exception as e:
            return False, f"wrote {LEG_LIMITS_PATH} but reload failed: {e}"
        if self.limits is None:
            return False, (
                f"wrote {LEG_LIMITS_PATH} but it failed validation on "
                f"reload — check the file (perhaps an axis is missing).")
        # Limits supersede captured_rest_positions.
        self.captured_rest_positions = None
        # Defense in depth: refresh the feeder's pos_targets to the new
        # neutral-pose rest values, so even if the per-leg seed-update
        # ever misses (encoder stale, etc.) the fallback is the CURRENT
        # leg_limits.yaml's rest, not whatever was loaded at startup
        # from the previous file. Pairs with the encoder-staleness arm
        # refusal in _arm_leg_internal.
        try:
            if self.feeder is not None and self.geom is not None:
                neutral_targets, _clamped = _compute_motor_targets(
                    (0, 0, 0), (0, 0, 0), self.geom, self.limits)
                self.feeder.set_pos_targets(neutral_targets)
        except Exception as e:
            self.get_logger().warn(
                f"feeder pos_targets refresh failed (non-fatal): {e}")
        msg_parts = [
            f"leg_limits.yaml written: {len(legs_written)} legs updated "
            f"({legs_written}); {len(legs_skipped)} preserved from existing "
            f"file; rest_offset = {rest_offset_turns:.3f} turns."
        ]
        if legs_derived:
            msg_parts.append(
                f"Top derived for legs {legs_derived} via default_stroke = "
                f"{default_stroke_turns:.2f} turns (positive=down convention).")
        msg_parts.append("Limits reloaded — pose / jog / arm now work normally.")
        return True, " ".join(msg_parts)

    def _safe_arm_in_place(self, current=6.0, vel_limit=None):
        """Arm all 6 legs at their current encoder positions, bypassing
        the leg_limits.yaml requirement. Use BEFORE homing to prevent
        legs from dropping under gravity during ODrive bench tests or
        partial homing attempts.

        Pre-flight checks added because this skips the limits guard:
          - bus must be open
          - all 6 legs must report a fresh encoder reading
          - listener must be running

        After this, /set_pose and /jog_leg will refuse (limits is None)
        — the legs hold rigidly at their current positions and can ONLY
        be released by Disarm or by running homing.
        """
        if self.bus is None:
            if not self._open_bus_and_start_threads():
                return False, "can't open can0"
        if self.listener is None:
            return False, "encoder listener not running"
        enc = self.listener.get_all(max_age_s=1.0)
        missing = [n for n in range(6) if enc[n] is None]
        if missing:
            return False, (
                f"safe arm refused: legs {missing} have no fresh encoder "
                f"reading. Power-cycle the ODrive(s) and try again.")
        ok, msg = self._arm_all_in_pos_mode(
            current=current, vel_limit=vel_limit, force_no_limits=True)
        if ok:
            return True, ("all 6 safe-armed in pos mode at current "
                          "positions. /set_pose and /jog_leg are blocked "
                          "until limits exist (run homing).")
        return ok, msg

    def _disarm_internal(self):
        """Send all legs to IDLE and restore user-default mode + limits."""
        if self.bus is None:
            self.leg_armed = [False] * 6
            return
        # IDLE first (kills watchdog), THEN reconfigure — this mirrors the
        # ordering that fixed the red-pulsing exit bug in platform_characterize.
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_SET_AXIS_STATE,
                              struct.pack('<I', STATE_IDLE))
                except Exception:
                    pass
        time.sleep(0.1)
        # Stop the feeder touching any leg
        if self.feeder is not None:
            self.feeder.set_all_modes('idle')
            self.feeder.set_vel_target(0.0)
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_SET_LIMITS,
                              struct.pack('<ff', self.soft_max_vel * 1.5, 6.0))
                    _send_cmd(self.bus, n, CMD_SET_CONTROLLER_MODE,
                              struct.pack('<II', CONTROL_MODE_VELOCITY,
                                          INPUT_MODE_VEL_RAMP))
                    _send_cmd(self.bus, n, CMD_SET_INPUT_VEL,
                              struct.pack('<ff', 0.0, 0.0))
                except Exception:
                    pass
        self.leg_armed = [False] * 6
        self._stop_level_loop()

    # ---- IMU ----
    def _imu_cb(self, msg):
        q = msg.orientation
        rpy = _quat_to_rpy_deg(q.w, q.x, q.y, q.z)
        with self.imu_lock:
            self.imu_rpy = rpy
            self.imu_last_rx = time.monotonic()
            self.imu_accel = (msg.linear_acceleration.x,
                              msg.linear_acceleration.y,
                              msg.linear_acceleration.z)
            self.imu_gyro = (msg.angular_velocity.x,
                             msg.angular_velocity.y,
                             msg.angular_velocity.z)

    # ---- periodic ticks ----
    def _tick_state(self):
        # encoders
        if self.listener is not None:
            enc = self.listener.get_all(max_age_s=0.5)
            m = Float64MultiArray()
            m.data = [float(v) if v is not None else float('nan') for v in enc]
            self.pub_encoders.publish(m)
            # currents (Iq measured, A)
            iq = self.listener.get_iq(max_age_s=2.0)
            mc = Float64MultiArray()
            mc.data = [float(v) for v in iq]
            self.pub_currents.publish(mc)
        # rpy
        with self.imu_lock:
            rpy = self.imu_rpy
            last_rx = self.imu_last_rx
        if rpy is not None:
            m = Float32MultiArray()
            m.data = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
            self.pub_rpy.publish(m)

    def _tick_status(self):
        # Dead-man watchdog: if /jog_vel_cmd hasn't arrived for > 0.5 s
        # AND any leg is in vel mode AND the last commanded value wasn't 0,
        # force zero. Catches browser crashes / lost network in the middle
        # of a drag.
        now_mono = time.monotonic()
        if self.feeder is not None:
            in_vel = self.feeder.any_in_mode('vel')
            stale = (now_mono - self._last_jog_vel_rx) > 0.5
            if in_vel and stale and self._last_jog_vel_value != 0.0:
                self.feeder.set_vel_target(0.0)
                self._last_jog_vel_value = 0.0
                self.get_logger().warn(
                    "jog_vel watchdog: stale input -> vel=0")

        # Publish errors snapshot each tick so the GUI panel auto-refreshes.
        self._publish_errors_snapshot()

        with self.imu_lock:
            imu_fresh = (self.imu_rpy is not None
                         and (time.monotonic() - self.imu_last_rx) < 0.5)
        homing_alive = (self.homing_proc is not None
                        and self.homing_proc.poll() is None)
        # Compute level errors (platform IMU vs saved platform_level.yaml ref)
        # regardless of whether the level loop is currently running — useful
        # for the stabilization demo UI to watch the error settle to zero.
        with self.imu_lock:
            cur_rpy = self.imu_rpy
        level_cal = _load_level_cal()
        if cur_rpy is not None and level_cal is not None:
            err_r = cur_rpy[0] - level_cal[0]
            err_p = cur_rpy[1] - level_cal[1]
        else:
            err_r = float('nan')
            err_p = float('nan')

        status = {
            'armed': bool(self.armed),
            'limits_loaded': self.limits is not None,
            'bus_open': self.bus is not None,
            'imu_fresh': bool(imu_fresh),
            'level_enabled': bool(self.level_enabled),
            'homing_running': bool(homing_alive),
            'soft_max_vel_turns_per_sec': float(self.soft_max_vel),
            'hard_max_vel_turns_per_sec': float(self.hard_max_vel),
            'current_xyz': self.current_xyz,
            'current_rpy': self.current_rpy,
            'level_err_roll_deg': float(err_r),
            'level_err_pitch_deg': float(err_p),
            'level_corr_roll_deg': float(self.level_corr[0]),
            'level_corr_pitch_deg': float(self.level_corr[1]),
            'imu_yaw_deg': float(cur_rpy[2]) if cur_rpy is not None else float('nan'),
            'recording': self.recording_t0 is not None,
            'recording_keyframe_count': len(self.recording_keyframes),
            'playing_routine': self.playing_routine_name,
            'playing_routine_elapsed_s': float(self.playing_routine_elapsed),
            'playing_routine_duration_s': float(self.playing_routine_duration),
        }
        m = String()
        m.data = json.dumps(status)
        self.pub_status.publish(m)

    # ---- service handlers ----
    def srv_activate(self, req, res):
        # ActivateOrDeactivate.srv: string command in {'activate','deactivate'}
        cmd = (req.command or '').strip().lower()
        if cmd in ('activate', 'arm', '1', 'true', 'on'):
            ok, msg = self._arm_all_in_pos_mode()
        elif cmd in ('deactivate', 'disarm', '0', 'false', 'off'):
            self._disarm_internal()
            ok, msg = True, "all 6 disarmed"
        else:
            ok, msg = False, f"unknown command '{req.command}'"
        res.success = ok
        res.message = msg
        return res

    def srv_arm_leg(self, req, res):
        n = int(req.leg)
        if not 0 <= n <= 5:
            res.success = False
            res.message = f"leg {n} out of range"
            return res
        mode = (req.mode or '').strip().lower()
        if mode not in ('pos', 'vel', 'idle'):
            res.success = False
            res.message = f"mode must be 'pos'|'vel'|'idle', got '{req.mode}'"
            return res
        ok, msg = self._arm_leg_internal(n, mode)
        res.success = ok
        res.message = msg
        return res

    def _jog_vel_cmd_cb(self, msg):
        """Dead-man velocity input. The GUI publishes this continuously while
        the slider is being dragged, and sends 0 on release."""
        if self.feeder is None:
            return
        v = float(msg.data)
        cap = self.soft_max_vel
        v = max(-cap, min(cap, v))
        self._last_jog_vel_rx = time.monotonic()
        self._last_jog_vel_value = v
        self.feeder.set_vel_target(v)

    def _control_cmd_cb(self, msg):
        """Topic-based alternative to every service. Needed because
        rosbridge in WSL2 + ROS 2 Kilted can't reliably discover services
        across processes (topics discover fine). Payload is JSON string
        with a 'cmd' key and per-cmd args. Results come back via the
        /status topic and /control_result topic."""
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"control_cmd: bad JSON: {e}")
            return
        cmd = d.get('cmd', '')
        ok, reply_msg = False, f"unknown cmd '{cmd}'"
        extra = {}  # extra fields to include in the response JSON
        try:
            if cmd == 'activate':
                ok, reply_msg = self._arm_all_in_pos_mode()
            elif cmd == 'capture_endstop':
                # Manual endstop capture for leg_limits.yaml without
                # automatic stall-homing. Payload:
                #   {"which": "bottom"|"top", "legs": [0,1,...]}
                # legs is optional; omit / null = all 6 legs.
                which = (d.get('which') or '').strip().lower()
                legs_arg = d.get('legs')
                if legs_arg is not None:
                    try:
                        legs_arg = [int(x) for x in legs_arg]
                    except Exception:
                        legs_arg = None
                ok, reply_msg, encs = self._capture_endstop(which, legs=legs_arg)
                if encs is not None:
                    extra['positions'] = list(encs)
                extra['captured'] = {
                    k: list(v) for k, v in self.captured_endstops.items()
                }
            elif cmd == 'save_limits':
                # Compute rest = bottom + offset, write leg_limits.yaml,
                # reload self.limits. Payload:
                #   {"rest_offset_turns": <float>,
                #    "default_stroke_turns": <float>}
                # If only bottom captured, top = bottom - default_stroke_turns
                # (positive=down convention per project memory).
                offset = float(d.get('rest_offset_turns', 0.20))
                stroke = float(d.get('default_stroke_turns', 3.0))
                ok, reply_msg = self._save_limits_from_captures(
                    rest_offset_turns=offset,
                    default_stroke_turns=stroke)
                # Echo the captured state so the GUI can update its
                # status indicators after save.
                extra['captured'] = {
                    k: list(v) for k, v in self.captured_endstops.items()
                }
            elif cmd == 'clear_endstops':
                # Clear all captures, OR a specific endstop, OR specific legs.
                # Payload (all optional):
                #   {"which": "bottom"|"top",  // default: both
                #    "legs": [0,1,...]}        // default: all
                which_clear = (d.get('which') or '').strip().lower()
                whichs = [which_clear] if which_clear in ('bottom', 'top') \
                    else ['bottom', 'top']
                legs_clear = d.get('legs')
                if legs_clear is not None:
                    try:
                        legs_clear = [int(x) for x in legs_clear]
                    except Exception:
                        legs_clear = list(range(6))
                else:
                    legs_clear = list(range(6))
                for w in whichs:
                    for n in legs_clear:
                        if 0 <= n <= 5:
                            self.captured_endstops[w][n] = None
                ok, reply_msg = True, (
                    f"cleared {whichs} captures for legs {legs_clear}")
                extra['captured'] = {
                    k: list(v) for k, v in self.captured_endstops.items()
                }
            elif cmd == 'capture_rest':
                # Snapshot current encoder readings into captured_rest_positions.
                # Doesn't arm — user presses regular Arm afterward, which
                # will use the captured positions if no leg_limits.yaml.
                ok, reply_msg, encs = self._capture_rest_positions()
                if encs is not None:
                    extra['positions'] = list(encs)
            elif cmd == 'clear_rest':
                self.captured_rest_positions = None
                ok, reply_msg = True, "captured rest positions cleared."
            elif cmd == 'safe_arm':
                # Legacy: capture + arm in one click. Kept for backward
                # compatibility with the v9.2-deployed GUI; new GUI uses
                # the two-button capture_rest + activate flow.
                ok, reply_msg = self._safe_arm_in_place()
            elif cmd == 'deactivate' or cmd == 'e_stop':
                self._disarm_internal()
                ok, reply_msg = True, "all 6 disarmed"
            elif cmd == 'arm_leg':
                leg = int(d.get('leg', -1))
                mode = str(d.get('mode', '')).strip().lower()
                if not 0 <= leg <= 5:
                    ok, reply_msg = False, f"leg {leg} out of range"
                elif mode not in ('pos', 'vel', 'idle'):
                    ok, reply_msg = False, f"bad mode '{mode}'"
                else:
                    ok, reply_msg = self._arm_leg_internal(leg, mode)
            elif cmd == 'jog_leg':
                ok, reply_msg = self._do_jog_leg(
                    int(d.get('leg', 0)), float(d.get('delta_turns', 0.0)))
            elif cmd == 'set_pose':
                ok, reply_msg = self._do_set_pose(
                    float(d.get('x', 0)), float(d.get('y', 0)),
                    float(d.get('z', 0)), float(d.get('roll', 0)),
                    float(d.get('pitch', 0)), float(d.get('yaw', 0)))
            elif cmd == 'go_to_rest':
                # Always stop the level loop first so it can't keep re-
                # adding tilt correction while we're trying to park.
                if self.level_enabled:
                    self._stop_level_loop()
                if not self.armed:
                    ok, reply_msg = False, "not armed"
                elif self.limits is None:
                    ok, reply_msg = False, "no leg_limits.yaml"
                else:
                    targets, _ = _compute_motor_targets(
                        (0, 0, 0), (0, 0, 0), self.geom, self.limits)
                    self.feeder.set_pos_targets(targets)
                    self.current_xyz = [0.0, 0.0, 0.0]
                    self.current_rpy = [0.0, 0.0, 0.0]
                    ok, reply_msg = True, "rest pose (level disabled)"
            elif cmd == 'set_speed_cap':
                v = max(0.1, min(float(d.get('value', 1.0)), self.hard_max_vel))
                self.soft_max_vel = v
                ok, reply_msg = True, f"soft_max_vel = {v:.3f}"
            elif cmd == 'enable_level':
                want = bool(d.get('enable', True))
                ok, reply_msg = self._do_enable_level(want)
            elif cmd == 'start_homing':
                ok, reply_msg = self._do_start_homing(d)
            elif cmd == 'cancel_homing':
                ok, reply_msg = self._do_cancel_homing()
            elif cmd == 'read_errors':
                ok, reply_msg = self._do_read_errors()
            elif cmd == 'clear_errors':
                ok, reply_msg = self._do_clear_errors()
            elif cmd == 'set_leg_current':
                ok, reply_msg = self._do_set_leg_current(
                    float(d.get('value', 6.0)))
            elif cmd == 'list_routines':
                names = self._list_routine_names()
                ok, reply_msg = True, f"{len(names)} routines"
                extra['routines'] = names
            elif cmd == 'start_recording':
                self.recording_t0 = time.monotonic()
                self.recording_keyframes = []
                ok, reply_msg = True, "recording started (t0 reset)"
                extra['keyframes'] = []
            elif cmd == 'record_keyframe':
                ok, reply_msg, kfs = self._do_record_keyframe()
                extra['keyframes'] = kfs
            elif cmd == 'clear_recording':
                self.recording_keyframes = []
                # Keep t0 so the next add starts counting from where we were
                ok, reply_msg = True, "recording cleared"
                extra['keyframes'] = []
            elif cmd == 'save_recording':
                ok, reply_msg = self._do_save_recording(
                    str(d.get('name', '')).strip(),
                    bool(d.get('loop', False)))
                extra['routines'] = self._list_routine_names()
            elif cmd == 'play_routine':
                ok, reply_msg = self._do_play_routine(
                    str(d.get('name', '')).strip(),
                    bool(d.get('loop', False)),
                    bool(d.get('record', False)))
            elif cmd == 'stop_routine':
                ok, reply_msg = self._do_stop_routine()
            elif cmd == 'delete_routine':
                ok, reply_msg = self._do_delete_routine(
                    str(d.get('name', '')).strip())
                extra['routines'] = self._list_routine_names()
            elif cmd == 'reset_can':
                ok, reply_msg = self._do_reset_can()
            elif cmd == 'restart_ros2_daemon':
                ok, reply_msg = self._do_restart_ros2_daemon()
            elif cmd == 'probe_graph':
                # Enumerate topics + nodes via rclpy (same info the ros2
                # CLI would show) so the GUI doesn't have to call the
                # /rosapi/topics service (which is unreliable here).
                tnt = self.get_topic_names_and_types()
                extra['topics'] = [
                    {'name': n, 'types': list(t)} for n, t in tnt]
                nlist = []
                for nname, ns in self.get_node_names_and_namespaces():
                    if ns == '/' or ns == '':
                        nlist.append('/' + nname)
                    else:
                        nlist.append(ns.rstrip('/') + '/' + nname)
                extra['nodes'] = sorted(nlist)
                ok, reply_msg = True, (
                    f"{len(tnt)} topics, {len(nlist)} nodes")
            elif cmd == 'generate_rolling_ball_routine':
                ok, reply_msg = self._do_generate_rolling_ball(
                    radius_m=float(d.get('radius_m', 0.18)),
                    period_s=float(d.get('period_s', 3.0)),
                    ramp_up_s=float(d.get('ramp_up_s', 2.0)),
                    hold_cycles=float(d.get('hold_cycles', 3)),
                    ramp_down_s=float(d.get('ramp_down_s', 2.0)),
                    center_hold_s=float(d.get('center_hold_s', 2.0)),
                    center_z_mm=float(d.get('center_z_mm', 30.0)),
                    max_tilt_deg=float(d.get('max_tilt_deg', 0.0)),
                    min_tilt_deg=float(d.get('min_tilt_deg', 0.0)),
                    tilt_multiplier=float(d.get('tilt_multiplier', 1.0)))
                extra['routines'] = self._list_routine_names()
            else:
                ok, reply_msg = False, f"unknown cmd '{cmd}'"
        except Exception as e:
            ok, reply_msg = False, f"exception: {e}"
        # Publish result
        resp = {'cmd': cmd, 'success': bool(ok), 'message': reply_msg, **extra}
        # carry id through for GUI correlation, if present
        if 'id' in d:
            resp['id'] = d['id']
        rm = String()
        rm.data = json.dumps(resp)
        self.pub_control_result.publish(rm)

    # ---- command helpers (shared by service handlers and /control_cmd) ----
    def _do_jog_leg(self, n, delta_turns):
        """Position-mode jog on one leg. Auto-switches the leg into POS
        mode if it isn't already (from IDLE or VEL) — the target is
        seeded from the current encoder reading so there's no lurch on
        mode switch. That makes per-leg jog work regardless of whether
        the user last used the velocity slider or arm-all."""
        if self.limits is None:
            return False, "no leg_limits.yaml"
        if not 0 <= n <= 5:
            return False, f"leg {n} out of range"
        if self.feeder is None:
            return False, "feeder not running"
        modes = self.feeder.get_modes()
        if modes[n] != 'pos':
            ok, msg = self._arm_leg_internal(n, 'pos')
            if not ok:
                return False, f"auto-arm POS failed: {msg}"
        cur = self.feeder.get_pos_targets()
        new_target = float(cur[n]) + float(delta_turns)
        lo, hi = self.limits[n]['lo'], self.limits[n]['hi']
        clamped = False
        if new_target < lo: new_target = lo; clamped = True
        elif new_target > hi: new_target = hi; clamped = True
        cur[n] = new_target
        self.feeder.set_pos_targets(cur)
        return True, (f"clamped at {new_target:+.4f}" if clamped
                      else f"L{n} → {new_target:+.4f}")

    def _do_set_pose(self, x, y, z, r, p, yw):
        if not self.armed:
            return False, "not armed"
        if self.limits is None:
            return False, "no leg_limits.yaml"
        self.current_xyz = [x, y, z]
        self.current_rpy = [r, p, yw]
        if self.level_enabled:
            return True, "pose set (level loop will track)"
        targets, any_clamped = _compute_motor_targets(
            (x, y, z), (r, p, yw), self.geom, self.limits)
        self.feeder.set_pos_targets(targets)
        return True, ("pose sent (CLAMPED)" if any_clamped else "pose sent")

    def _do_read_errors(self):
        """Force a fresh RTR round and return a JSON-serialised snapshot."""
        if self.bus is None:
            return False, "bus not open"
        # Send RTR for GET_ERROR to all 6, wait briefly for responses
        with self.bus_lock:
            for n in range(6):
                try:
                    arb = (n << 5) | CMD_GET_ERROR
                    self.bus.send(can.Message(arbitration_id=arb,
                                              is_remote_frame=True,
                                              is_extended_id=False, dlc=8),
                                  timeout=0.1)
                except Exception:
                    pass
        # Give the listener a moment to capture replies
        time.sleep(0.25)
        self._publish_errors_snapshot()
        return True, "errors snapshot published"

    # ---- Demo trajectory generators ----
    def _do_generate_rolling_ball(self, radius_m, period_s, ramp_up_s,
                                  hold_cycles, ramp_down_s, center_hold_s,
                                  center_z_mm=30.0, max_tilt_deg=0.0,
                                  min_tilt_deg=0.0, tilt_multiplier=1.0,
                                  sample_hz=50.0):
        """Generate a rolling-ball trajectory with full feed-forward.

        For a ball spiraling at r(t)=R(t) with constant angular velocity ω,
        required ball acceleration in polar is:
            radial  : R̈ - R·ω²   (positive = outward)
            tangential : 2·Ṙ·ω   (in direction of motion)
        For a solid sphere rolling without slipping, the platform tilt must
        provide force of (5/7)·g·sin(θ) along the downhill direction:

            sin(θ) = (7/5)/g · sqrt( (R·ω² - R̈)² + (2·Ṙ·ω)² )

        and the tilt direction is shifted by atan2(2Ṙω, R·ω² - R̈) from
        the pure-centripetal pattern. This lead phase is what imparts
        tangential velocity to the ball during the spin-up ramp.

        R(t) uses a 5th-order smoothstep (s(0)=s(1)=0 for s, s', s'') so
        θ starts and ends at zero with zero derivatives → no impulsive
        motor commands at phase boundaries. During ramp_up R climbs
        0 → R_final; ramp_down mirrors; center_hold sets tilt to 0 so
        the ball settles at the center.
        """
        if period_s <= 0 or radius_m <= 0 or hold_cycles < 0:
            return False, "period_s, radius_m must be > 0; cycles >= 0"
        if ramp_up_s < 0 or ramp_down_s < 0:
            return False, "ramp_up_s and ramp_down_s must be >= 0"
        if tilt_multiplier <= 0:
            return False, "tilt_multiplier must be > 0"
        # Note: ramp_up_s=0 produces an instant jump to steady-state tilt
        # at t=0 (motors will slew hard and PI will catch up). Same for
        # ramp_down_s=0 at the end. Useful for static-friction breaks.
        g = 9.81
        omega = 2.0 * math.pi / period_s
        a_c = omega * omega * radius_m
        arg = (7.0 / 5.0) * a_c / g
        if arg >= 1.0:
            return False, (
                f"physically infeasible: (7/5)·ω²·R/g = {arg:.3f} must be "
                f"< 1. Slower period or smaller radius needed.")
        theta_physics_deg = math.degrees(math.asin(arg))
        theta_target_deg = theta_physics_deg
        clamped_tilt = False
        effective_radius_m = radius_m
        if max_tilt_deg > 0 and theta_physics_deg > max_tilt_deg:
            theta_target_deg = max_tilt_deg
            clamped_tilt = True
            effective_radius_m = (
                math.sin(math.radians(max_tilt_deg)) * g
                / ((7.0/5.0) * omega * omega))
        peak_tilt_rate = theta_target_deg * omega
        cap = float(self.global_limits.get(
            'hard_max_tilt_rate_deg_per_sec', 15.0))
        if peak_tilt_rate > cap:
            try:
                omega_min = (cap * g / ((7.0/5.0) * radius_m)) ** (1.0/3.0)
                t_min = 2.0 * math.pi / omega_min
            except Exception:
                t_min = period_s * 1.5
            return False, (
                f"peak tilt rate {peak_tilt_rate:.1f}°/s exceeds hard cap "
                f"{cap:.1f}°/s. Try period >= ~{t_min:.1f}s, or raise "
                f"`hard_max_tilt_rate_deg_per_sec` in global_limits.yaml.")

        # If max_tilt_deg < physics θ, we're aiming for a smaller effective
        # radius. Drive the kinematics with R_eff so the feed-forward is
        # self-consistent (the ball's orbit will actually match this R).
        R_drive = effective_radius_m

        # Smoothstep helpers (5th order). s(0)=0, s(1)=1, derivatives zero
        # at endpoints.
        def _ss(u):
            if u <= 0.0: return 0.0
            if u >= 1.0: return 1.0
            return 10*u**3 - 15*u**4 + 6*u**5

        def _ss_d1(u):
            if u <= 0.0 or u >= 1.0: return 0.0
            return 30*u**2 - 60*u**3 + 30*u**4

        def _ss_d2(u):
            if u <= 0.0 or u >= 1.0: return 0.0
            return 60*u - 180*u**2 + 120*u**3

        # Build keyframes at sample_hz (50 Hz default now — feed-forward
        # needs finer time resolution so Ṙ and R̈ look smooth to the motors).
        dt = 1.0 / sample_hz
        hold_dur = hold_cycles * period_s
        total = ramp_up_s + hold_dur + ramp_down_s + center_hold_s
        kfs = []
        t = 0.0
        floor_deg = max(0.0, float(min_tilt_deg))

        while t <= total + 1e-6:
            if t < ramp_up_s:
                # Ramp up: R climbs 0 → R_drive via smoothstep
                u = t / ramp_up_s
                R = R_drive * _ss(u)
                R_d = R_drive * _ss_d1(u) / ramp_up_s
                R_dd = R_drive * _ss_d2(u) / (ramp_up_s ** 2)
                phase = omega * t
            elif t < ramp_up_s + hold_dur:
                # Hold: constant R, steady-state feed-forward (R_d=R_dd=0)
                R = R_drive
                R_d = 0.0
                R_dd = 0.0
                phase = omega * t
            elif t < ramp_up_s + hold_dur + ramp_down_s:
                # Ramp down: R shrinks R_drive → 0 via smoothstep (reverse)
                tau = (t - ramp_up_s - hold_dur) / ramp_down_s
                u = 1.0 - tau
                R = R_drive * _ss(u)
                # du/dt = -1/ramp_down_s
                R_d = -R_drive * _ss_d1(u) / ramp_down_s
                R_dd = R_drive * _ss_d2(u) / (ramp_down_s ** 2)
                phase = omega * t
            else:
                # Center hold: flat, no motion
                R = 0.0
                R_d = 0.0
                R_dd = 0.0
                phase = omega * t

            # Required acceleration magnitude along downhill direction:
            #   (5/7)·g·sin(θ) = sqrt( (R·ω² - R̈)² + (2·Ṙ·ω)² )
            radial = R * omega * omega - R_dd
            tangential = 2.0 * R_d * omega
            acc_mag = math.sqrt(radial * radial + tangential * tangential)
            sin_theta = (7.0/5.0) * acc_mag / g
            # Clamp for safety (shouldn't hit unless user cranks params)
            sin_theta = max(-0.95, min(0.95, sin_theta))
            theta_rad = math.asin(sin_theta)
            theta_deg = math.degrees(theta_rad)

            # Apply tilt_multiplier BEFORE caps so max_tilt_deg still acts
            # as an absolute safety ceiling. Multiplier > 1 pushes beyond
            # the physics-ideal (ball will spiral inward relative to R),
            # multiplier < 1 pulls back (ball spirals outward). Use > 1
            # when static friction is eating your motion.
            theta_deg *= tilt_multiplier

            # Enforce max/min tilt caps (applied AFTER multiplier)
            if max_tilt_deg > 0 and theta_deg > max_tilt_deg:
                theta_deg = max_tilt_deg
            if floor_deg > 0 and 0 < t < ramp_up_s + hold_dur + ramp_down_s:
                # Apply min only while the demo is actively running; leave
                # center_hold at zero so the ball can settle.
                if theta_deg < floor_deg:
                    theta_deg = floor_deg

            # Phase lead: tangential term rotates the tilt direction ahead
            # of the pure-centripetal phase by atan2(2Ṙω, Rω² - R̈).
            # Use atan2 with zero guards so hold-phase (radial=Rω², tang=0)
            # returns exactly 0.
            if abs(tangential) > 1e-12 or abs(radial) > 1e-12:
                phase_lead = math.atan2(tangential, radial)
            else:
                phase_lead = 0.0

            total_phase = phase + phase_lead
            kfs.append({
                't': round(t, 3),
                'x': 0.0, 'y': 0.0, 'z': float(center_z_mm),
                'roll':  round(theta_deg * math.sin(total_phase), 3),
                'pitch': round(theta_deg * math.cos(total_phase), 3),
                'yaw':   0.0,
            })
            t += dt

        name = (f"rolling_ball_T{period_s:.1f}s_R{int(round(radius_m*1000))}mm"
                f"_Z{int(round(center_z_mm))}mm")
        if clamped_tilt:
            name += f"_maxTilt{max_tilt_deg:.1f}deg"
        if abs(tilt_multiplier - 1.0) > 1e-3:
            name += f"_x{tilt_multiplier:.1f}"
        if ramp_up_s == 0 or ramp_down_s == 0:
            name += "_noramp"
        name = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
        doc = {
            'name': name,
            'loop': False,
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'keyframe_count': len(kfs),
            'generator': 'rolling_ball',
            'params': {
                'radius_m': radius_m, 'period_s': period_s,
                'ramp_up_s': ramp_up_s, 'hold_cycles': hold_cycles,
                'ramp_down_s': ramp_down_s, 'center_hold_s': center_hold_s,
                'center_z_mm': center_z_mm,
                'max_tilt_deg': max_tilt_deg,
                'min_tilt_deg': min_tilt_deg,
                'tilt_multiplier': tilt_multiplier,
                'theta_physics_deg': theta_physics_deg,
                'theta_target_deg': theta_target_deg,
                'theta_clamped': clamped_tilt,
                'effective_radius_m': effective_radius_m,
                'peak_tilt_rate_deg_per_sec': peak_tilt_rate,
                'total_duration_s': total,
            },
            'keyframes': kfs,
        }
        path = os.path.join(ROUTINES_DIR, name + '.json')
        with open(path, 'w') as f:
            json.dump(doc, f, indent=2)
        msg = (f"'{name}' ({len(kfs)} kf, {total:.1f}s, "
               f"θ={theta_target_deg:.2f}°, peak rate={peak_tilt_rate:.1f}°/s)")
        if clamped_tilt:
            msg += (f"  [physics θ={theta_physics_deg:.2f}° clamped to "
                    f"{max_tilt_deg:.1f}°; ball will orbit at ~"
                    f"{effective_radius_m*1000:.0f} mm, not {radius_m*1000:.0f}]")
        return True, msg

    # ---- CAN bus recovery ----
    def _do_reset_can(self):
        """Soft-reset can0. Runs the ip link cycle in a background worker
        so rclpy stays responsive (the subprocess + thread joins can take
        up to ~15 s). Progress is reported via `reset_can_progress` msgs
        on /control_result; the final `reset_can` msg carries the overall
        outcome. Needs NOPASSWD for `ip link set can0 *`."""
        if self.reset_can_thread is not None and self.reset_can_thread.is_alive():
            return False, "a CAN reset is already running"
        self.reset_can_thread = threading.Thread(
            target=self._reset_can_worker, daemon=True)
        self.reset_can_thread.start()
        return True, "soft reset started — watch progress messages"

    def _publish_progress(self, cmd, msg, success=True):
        m = String()
        m.data = json.dumps({'cmd': cmd, 'message': msg, 'success': success})
        try:
            self.pub_control_result.publish(m)
        except Exception:
            pass

    def _do_restart_ros2_daemon(self):
        """Kick the ros2 CLI daemon if it's wedged (common in WSL2 —
        makes `ros2 topic/node list` hang). No sudo needed; runs in a
        thread to avoid blocking rclpy. Doesn't affect already-running
        nodes or rosbridge, only the CLI discovery cache."""
        t = threading.Thread(target=self._daemon_restart_worker, daemon=True)
        t.start()
        return True, "restarting ros2 daemon in background"

    def _daemon_restart_worker(self):
        progress = lambda s: self._publish_progress(
            'restart_ros2_daemon_progress', s)
        final = lambda ok, s: self._publish_progress(
            'restart_ros2_daemon', s, ok)
        progress("ros2 daemon stop")
        try:
            r = subprocess.run(['ros2', 'daemon', 'stop'],
                               capture_output=True, text=True, timeout=5)
            if r.returncode != 0 and r.stderr.strip():
                progress(f"  stop stderr: {r.stderr.strip()}")
        except subprocess.TimeoutExpired:
            progress("  stop timed out — force-killing daemon process")
            try:
                subprocess.run(['pkill', '-9', '-f', '_ros2_daemon'],
                               capture_output=True, timeout=3)
            except Exception:
                pass
        except Exception as e:
            progress(f"  stop error: {e}")
        progress("ros2 daemon start")
        try:
            r = subprocess.run(['ros2', 'daemon', 'start'],
                               capture_output=True, text=True, timeout=10)
            if r.returncode != 0:
                err = (r.stderr or r.stdout).strip() or f"rc={r.returncode}"
                final(False, f"daemon start failed: {err}")
                return
        except subprocess.TimeoutExpired:
            final(False, "daemon start timed out (>10 s)")
            return
        except Exception as e:
            final(False, f"daemon start error: {e}")
            return
        final(True, "ros2 daemon restarted OK")

    def _reset_can_worker(self):
        progress = lambda s: self._publish_progress('reset_can_progress', s)
        final    = lambda ok, s: self._publish_progress('reset_can', s, ok)
        # Publish IMMEDIATELY so the user knows the worker started, even if
        # the subsequent thread-join step hangs.
        progress("worker started")
        was_armed = self.armed

        # _close_bus_and_stop_threads() can hang if the feeder/listener
        # threads are stuck in a blocked can.send() because the adapter is
        # driver-level wedged. Run it in its own watchdog thread so this
        # worker keeps going even if the close thread never completes.
        close_done = threading.Event()
        def _close_worker():
            try:
                self._close_bus_and_stop_threads()
            except Exception as e:
                progress(f"close error: {e}")
            finally:
                close_done.set()
        close_thr = threading.Thread(target=_close_worker, daemon=True)
        close_thr.start()
        progress("closing bus + stopping threads (max 5 s wait)")
        if not close_done.wait(timeout=5.0):
            progress("  close thread is stuck — proceeding anyway. "
                     "Some feeder/listener state may be orphaned.")
            # Drop our references; daemon thread will die with the process
            self.feeder = None
            self.listener = None
            self.bus = None
        steps = [
            ['sudo', '-n', '/usr/sbin/ip', 'link', 'set', 'can0', 'down'],
            ['sudo', '-n', '/usr/sbin/ip', 'link', 'set', 'can0', 'up',
             'type', 'can', 'bitrate', '1000000'],
            ['sudo', '-n', '/usr/sbin/ip', 'link', 'set', 'can0',
             'txqueuelen', '1000'],
        ]
        for s in steps:
            progress(f"$ {' '.join(s[1:])}")
            try:
                r = subprocess.run(s, capture_output=True, text=True, timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    self._open_bus_and_start_threads()
                except Exception:
                    pass
                final(False, (
                    f"`{' '.join(s[1:])}` timed out after 4 s. If NOPASSWD "
                    "is configured, the gs_usb adapter is almost certainly "
                    "stuck — physically unplug + replug the USB cable "
                    "(then usbipd attach from Windows) and try again."))
                return
            except Exception as e:
                try:
                    self._open_bus_and_start_threads()
                except Exception:
                    pass
                final(False, f"subprocess error: {e}")
                return
            if r.returncode != 0:
                err = (r.stderr or r.stdout).strip() or f'rc={r.returncode}'
                progress(f"  failed: {err}")
                try:
                    self._open_bus_and_start_threads()
                except Exception:
                    pass
                if 'Timer expired' in err:
                    final(False, (
                        "`ip link set can0 up` returned 'Timer expired' — "
                        "the USB-CAN adapter is hung at the hardware level. "
                        "Physical replug (or usbipd detach+attach) is the "
                        "only recovery."))
                else:
                    final(False, f"ip link step failed: {err}")
                return
            progress("  ok")
        time.sleep(0.2)
        progress("reopening socket + helper threads")
        if not self._open_bus_and_start_threads():
            final(False, "ip link OK but socket reopen failed — try again")
            return
        progress("sending CLEAR_ERRORS to all 6 nodes")
        time.sleep(0.1)
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_CLEAR_ERRORS, b'\x00')
                except Exception:
                    pass
        msg = "CAN soft-reset OK. ODrive errors cleared."
        if was_armed:
            msg += " Re-arm required."
        final(True, msg)

    # ---- routine record / playback ----
    def _list_routine_names(self):
        files = glob.glob(os.path.join(ROUTINES_DIR, '*.json'))
        return sorted(os.path.splitext(os.path.basename(f))[0] for f in files)

    def _do_record_keyframe(self):
        if self.recording_t0 is None:
            return False, "not recording — call start_recording first", []
        t = time.monotonic() - self.recording_t0
        kf = {
            't': round(float(t), 3),
            'x':     float(self.current_xyz[0]),
            'y':     float(self.current_xyz[1]),
            'z':     float(self.current_xyz[2]),
            'roll':  float(self.current_rpy[0]),
            'pitch': float(self.current_rpy[1]),
            'yaw':   float(self.current_rpy[2]),
        }
        self.recording_keyframes.append(kf)
        return True, (f"kf #{len(self.recording_keyframes)} @ t={t:.2f}s "
                      f"xyz=({kf['x']:+.1f},{kf['y']:+.1f},{kf['z']:+.1f}) "
                      f"rpy=({kf['roll']:+.1f},{kf['pitch']:+.1f},{kf['yaw']:+.1f})"
                      ), list(self.recording_keyframes)

    def _do_save_recording(self, name, loop):
        if not name:
            return False, "empty name"
        safe = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
        if not safe:
            return False, f"invalid name '{name}'"
        if not self.recording_keyframes:
            return False, "no keyframes recorded"
        doc = {
            'name': safe,
            'loop': bool(loop),
            'saved_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'keyframe_count': len(self.recording_keyframes),
            'keyframes': self.recording_keyframes,
        }
        path = os.path.join(ROUTINES_DIR, safe + '.json')
        with open(path, 'w') as f:
            json.dump(doc, f, indent=2)
        return True, f"saved '{safe}' ({len(self.recording_keyframes)} kf)"

    def _do_delete_routine(self, name):
        if not name:
            return False, "empty name"
        path = os.path.join(ROUTINES_DIR, name + '.json')
        if not os.path.exists(path):
            return False, f"no routine '{name}'"
        os.remove(path)
        return True, f"deleted '{name}'"

    def _do_play_routine(self, name, loop, record=False):
        if (self.routine_player_thread is not None
                and self.routine_player_thread.is_alive()):
            return False, "a routine is already playing — stop it first"
        if not self.armed:
            return False, "arm first (routines command pose targets)"
        if self.limits is None:
            return False, "no leg_limits.yaml"
        path = os.path.join(ROUTINES_DIR, name + '.json')
        if not os.path.exists(path):
            return False, f"no routine '{name}'"
        try:
            with open(path) as f:
                doc = json.load(f)
        except Exception as e:
            return False, f"failed to load: {e}"
        kfs = doc.get('keyframes', [])
        if not kfs:
            return False, "routine has no keyframes"
        effective_loop = loop if loop else bool(doc.get('loop', False))

        log_msg = ""
        if record:
            rc, rmsg = self._start_recording_log(name, doc)
            if rc:
                log_msg = f" [REC → {rmsg}]"
            else:
                log_msg = f" [REC FAILED: {rmsg}]"

        self.routine_player_stop.clear()
        self.playing_routine_name = name
        self.routine_player_thread = threading.Thread(
            target=self._routine_player_run,
            args=(kfs, effective_loop),
            daemon=True,
        )
        self.routine_player_thread.start()
        return True, (f"playing '{name}' ({len(kfs)} kf, "
                      f"loop={effective_loop}){log_msg}")

    # ---- routine data logging ----
    # Writes routine.json + metadata.json + telemetry.csv + can.log into
    # ~/ros2_ws/src/stewart_bringup/logs/<timestamp>_<name>/ so an offline
    # script can reconstruct commanded vs actual motion, compute ball
    # dynamics, and correlate CAN events.
    RLOG_HEADERS = [
        't_s',
        'cmd_x', 'cmd_y', 'cmd_z',
        'cmd_roll', 'cmd_pitch', 'cmd_yaw',
        'imu_roll', 'imu_pitch', 'imu_yaw',
        'imu_ax', 'imu_ay', 'imu_az',
        'gyro_x', 'gyro_y', 'gyro_z',
        'enc_0', 'enc_1', 'enc_2', 'enc_3', 'enc_4', 'enc_5',
        'iq_0', 'iq_1', 'iq_2', 'iq_3', 'iq_4', 'iq_5',
        'tilt_corr_r', 'tilt_corr_p',
        'level_enabled',
        # Motor limits (can change mid-run via the sliders)
        'soft_max_vel_tps', 'leg_current_a',
        # Per-leg armed/mode state: 0=idle, 1=pos, 2=vel
        'mode_0', 'mode_1', 'mode_2', 'mode_3', 'mode_4', 'mode_5',
        # Routine playback progress
        'routine_elapsed_s', 'routine_duration_s',
    ]

    def _start_recording_log(self, routine_name, routine_doc):
        if self.rlog_dir is not None:
            return False, "already recording"
        stamp = time.strftime('%Y-%m-%d_%H%M%S', time.localtime())
        safe = re.sub(r'[^A-Za-z0-9_\-]', '_', routine_name or 'routine')
        log_dir = os.path.join(ROUTINE_LOGS_DIR, f"{stamp}_{safe}")
        try:
            os.makedirs(log_dir, exist_ok=True)
        except Exception as e:
            return False, f"mkdir failed: {e}"

        # Copy of the routine as played
        try:
            with open(os.path.join(log_dir, 'routine.json'), 'w') as f:
                json.dump(routine_doc, f, indent=2)
        except Exception as e:
            return False, f"routine dump failed: {e}"

        # Run-time context snapshot — capture as much as we can so the
        # offline analyzer can reproduce exactly what the run looked like.
        cal = _load_level_cal()
        leg_limits_dump = None
        try:
            if os.path.exists(LEG_LIMITS_PATH):
                with open(LEG_LIMITS_PATH) as f:
                    leg_limits_dump = yaml.safe_load(f)
        except Exception:
            pass
        meta = {
            'started_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'routine_name': routine_name,
            'armed_at_start': bool(self.armed),
            'leg_armed_at_start': list(self.leg_armed),
            'feeder_modes_at_start': (self.feeder.get_modes()
                                       if self.feeder is not None else None),
            'level_enabled_at_start': bool(self.level_enabled),
            'level_cal': {'ref_roll_deg': cal[0], 'ref_pitch_deg': cal[1]}
                         if cal else None,
            'soft_max_vel_turns_per_sec': self.soft_max_vel,
            'hard_max_vel_turns_per_sec': self.hard_max_vel,
            'leg_current_a_at_start': self.leg_current_a,
            'global_limits': dict(self.global_limits),
            'leg_limits_path': LEG_LIMITS_PATH,
            'leg_limits_snapshot': leg_limits_dump,
            'imu_topic': '/platform/imu/data',
            'control_period_s': CTRL_PERIOD_S,
            'level_gains': {
                'kp': LEVEL_KP, 'ki': LEVEL_KI,
                'filter_alpha': LEVEL_FILTER_ALPHA,
                'rate_limit_deg_per_iter': LEVEL_RATE_LIMIT,
                'max_corr_deg': LEVEL_MAX_CORR,
                'deadband_deg': LEVEL_DEADBAND,
            },
            # Version/feature flags so the offline analyzer can group runs
            # by control-algorithm version in summary.csv.
            'features': {
                'vel_feedforward': True,
                'dynamic_level_ref': True,
                'odrive_set_input_pos_vel_ff': True,
            },
        }
        try:
            with open(os.path.join(log_dir, 'metadata.json'), 'w') as f:
                json.dump(meta, f, indent=2, default=str)
        except Exception as e:
            return False, f"metadata dump failed: {e}"

        # Open telemetry CSV
        try:
            self.rlog_csv = open(os.path.join(log_dir, 'telemetry.csv'), 'w')
            self.rlog_csv.write(','.join(self.RLOG_HEADERS) + '\n')
            self.rlog_csv.flush()
        except Exception as e:
            return False, f"csv open failed: {e}"

        # Start candump (best-effort; continue if it fails)
        can_log_path = os.path.join(log_dir, 'can.log')
        try:
            self.rlog_can_proc = subprocess.Popen(
                ['candump', '-L', 'can0'],
                stdout=open(can_log_path, 'w'),
                stderr=subprocess.DEVNULL,
            )
        except FileNotFoundError:
            self.get_logger().warn(
                "candump not found — install can-utils for CAN capture")
            self.rlog_can_proc = None
        except Exception as e:
            self.get_logger().warn(f"candump start failed: {e}")
            self.rlog_can_proc = None

        self.rlog_dir = log_dir
        self.rlog_t0 = time.monotonic()
        # 50 Hz sample rate. ROS timer is the simplest way to schedule
        # without a dedicated thread.
        self.rlog_timer = self.create_timer(0.02, self._recording_sample)
        self.get_logger().info(f"recording to {log_dir}")
        return True, log_dir

    MODE_CODE = {'idle': 0, 'pos': 1, 'vel': 2}

    def _recording_sample(self):
        if self.rlog_csv is None:
            return
        t = time.monotonic() - self.rlog_t0
        with self.imu_lock:
            rpy = self.imu_rpy
            accel = self.imu_accel
            gyro = self.imu_gyro
        if rpy is None:   rpy = (float('nan'),) * 3
        if accel is None: accel = (float('nan'),) * 3
        if gyro is None:  gyro = (float('nan'),) * 3
        enc = (self.listener.get_all(max_age_s=0.5)
               if self.listener else [None] * 6)
        iq = (self.listener.get_iq(max_age_s=2.0)
              if self.listener else [float('nan')] * 6)
        modes = (self.feeder.get_modes() if self.feeder is not None
                 else ['idle'] * 6)
        mode_codes = [self.MODE_CODE.get(m, 0) for m in modes]
        row = [
            t,
            *self.current_xyz,
            *self.current_rpy,
            *rpy,
            *accel,
            *gyro,
            *[e if e is not None else float('nan') for e in enc],
            *iq,
            *self.level_corr,
            1 if self.level_enabled else 0,
            float(self.soft_max_vel),
            float(self.leg_current_a),
            *mode_codes,
            float(self.playing_routine_elapsed),
            float(self.playing_routine_duration),
        ]
        try:
            self.rlog_csv.write(','.join(
                f'{v:.6f}' if isinstance(v, float) else str(v)
                for v in row) + '\n')
        except Exception as e:
            self.get_logger().warn(f"rlog write failed: {e}")

    def _stop_recording_log(self):
        if self.rlog_timer is not None:
            try:
                self.rlog_timer.cancel()
                self.destroy_timer(self.rlog_timer)
            except Exception:
                pass
            self.rlog_timer = None
        if self.rlog_csv is not None:
            try:
                self.rlog_csv.flush()
                self.rlog_csv.close()
            except Exception:
                pass
            self.rlog_csv = None
        if self.rlog_can_proc is not None:
            try:
                self.rlog_can_proc.send_signal(signal.SIGINT)
                try:
                    self.rlog_can_proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.rlog_can_proc.kill()
            except Exception:
                pass
            self.rlog_can_proc = None
        if self.rlog_dir is not None:
            end = {
                'ended_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                'duration_s': time.monotonic() - self.rlog_t0,
            }
            try:
                with open(os.path.join(self.rlog_dir, 'end.json'), 'w') as f:
                    json.dump(end, f, indent=2)
            except Exception:
                pass
            self.get_logger().info(f"recording complete: {self.rlog_dir}")
            self.rlog_dir = None

    def _do_stop_routine(self):
        self.routine_player_stop.set()
        t = self.routine_player_thread
        if t is not None:
            t.join(timeout=2.0)
        self.routine_player_thread = None
        if self.playing_routine_name:
            msg = f"stopped '{self.playing_routine_name}'"
        else:
            msg = "no routine was playing"
        self.playing_routine_name = None
        self.playing_routine_elapsed = 0.0
        self.playing_routine_duration = 0.0
        # Stop recording if it was running
        if self.rlog_dir is not None:
            self._stop_recording_log()
        return True, msg

    def _routine_player_run(self, keyframes, loop):
        """Linear-interpolate between keyframes at 100 Hz and feed the pose
        through the same _do_set_pose path the GUI sliders use — so the
        level loop, soft-limit clamps, and feeder all behave identically
        to live pose commands."""
        # Sort defensively by time
        kfs = sorted(keyframes, key=lambda k: float(
            k.get('t', k.get('time_seconds', 0.0))))
        times = np.array([float(k.get('t', k.get('time_seconds', 0.0)))
                          for k in kfs])
        axes = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
        vals = {a: np.array([float(k.get(a, 0.0)) for k in kfs])
                for a in axes}
        t_min = float(times[0]) if len(times) else 0.0
        t_max = float(times[-1]) if len(times) else 0.0
        dur = t_max - t_min
        self.playing_routine_duration = dur
        self.playing_routine_elapsed = 0.0

        period = 0.01  # 100 Hz
        t_start = time.monotonic()
        self.get_logger().info(
            f"routine player: {len(kfs)} keyframes, duration={dur:.2f}s, "
            f"loop={loop}")
        while not self.routine_player_stop.is_set():
            loop_t0 = time.monotonic()
            elapsed = loop_t0 - t_start
            self.playing_routine_elapsed = elapsed
            if dur > 0:
                if elapsed > dur:
                    if loop:
                        t_start = loop_t0
                        elapsed = 0.0
                    else:
                        break
                t_eval = t_min + elapsed
                pose = {a: float(np.interp(t_eval, times, vals[a]))
                        for a in axes}
            else:
                # Single-keyframe routine — hold indefinitely
                pose = {a: float(vals[a][0]) for a in axes}
            # Apply via the same path as GUI sliders
            self._do_set_pose(pose['x'], pose['y'], pose['z'],
                              pose['roll'], pose['pitch'], pose['yaw'])
            sl = period - (time.monotonic() - loop_t0)
            if sl > 0:
                time.sleep(sl)
        self.get_logger().info("routine player exited")
        # Stop recording (if active). Safe to call even if nothing is running.
        if self.rlog_dir is not None:
            self._stop_recording_log()

    def _do_set_leg_current(self, current_a):
        """Update SET_LIMITS on every leg to the new current cap. Keeps
        the current vel_limit unchanged. Takes effect immediately on the
        ODrive regardless of axis state."""
        current_a = max(1.0, min(float(current_a), 15.0))
        if self.bus is None:
            return False, "bus not open"
        vl = self.soft_max_vel * 1.5
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_SET_LIMITS,
                              struct.pack('<ff', vl, current_a))
                except Exception:
                    pass
        self.leg_current_a = current_a
        return True, f"current = {current_a:.1f} A on all 6 legs"

    def _do_clear_errors(self):
        if self.bus is None:
            return False, "bus not open"
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_CLEAR_ERRORS, b'\x00')
                except Exception:
                    pass
        # And immediately follow up with a fresh RTR round so the UI sees
        # the cleared state.
        time.sleep(0.1)
        self._do_read_errors()
        return True, "clear_errors sent to all 6"

    def _publish_errors_snapshot(self):
        if self.listener is None:
            return
        errs = self.listener.get_errors(max_age_s=3.0)
        nodes = []
        for n in range(6):
            e = errs[n]
            if e is None:
                nodes.append({'node': n, 'available': False})
            else:
                active, disarm = e
                nodes.append({
                    'node': n,
                    'available': True,
                    'active_errors': active,
                    'disarm_reason': disarm,
                    'active_decoded': _decode_error_bits(active),
                    'disarm_decoded': _decode_error_bits(disarm),
                })
        m = String()
        m.data = json.dumps({'nodes': nodes, 'ts': time.time()})
        self.pub_errors.publish(m)

    def _do_enable_level(self, want):
        if want:
            if not self.armed:
                return False, "arm first"
            cal = _load_level_cal()
            if cal is None:
                return False, f"no {LEVEL_CAL_PATH} — capture level first"
            self.level_ref_roll, self.level_ref_pitch = cal
            self._start_level_loop()
            return True, (f"level enabled "
                          f"(ref roll={self.level_ref_roll:+.3f} "
                          f"pitch={self.level_ref_pitch:+.3f})")
        self._stop_level_loop()
        return True, "level disabled"

    def srv_e_stop(self, req, res):
        self._disarm_internal()
        res.success = True
        res.message = "e-stop: all axes IDLE"
        return res

    def srv_go_to_rest(self, req, res):
        if not self.armed:
            res.success = False
            res.message = "not armed"
            return res
        if self.limits is None:
            res.success = False
            res.message = "no leg_limits.yaml"
            return res
        targets, _ = _compute_motor_targets(
            (0, 0, 0), (0, 0, 0), self.geom, self.limits)
        self.feeder.set_pos_targets(targets)
        self.current_xyz = [0.0, 0.0, 0.0]
        self.current_rpy = [0.0, 0.0, 0.0]
        res.success = True
        res.message = "sent rest pose"
        return res

    def srv_jog_leg(self, req, res):
        if not self.armed:
            res.success = False
            res.message = "not armed"
            res.final_pos_turns = 0.0
            return res
        if self.limits is None:
            res.success = False
            res.message = "no leg_limits.yaml"
            res.final_pos_turns = 0.0
            return res
        n = int(req.leg)
        if not 0 <= n <= 5:
            res.success = False
            res.message = f"leg {n} out of range"
            res.final_pos_turns = 0.0
            return res
        cur = self.feeder.get_pos_targets()
        new_target = float(cur[n]) + float(req.delta_turns)
        lo, hi = self.limits[n]['lo'], self.limits[n]['hi']
        clamped = False
        if new_target < lo:
            new_target = lo
            clamped = True
        elif new_target > hi:
            new_target = hi
            clamped = True
        cur[n] = new_target
        self.feeder.set_pos_targets(cur)
        res.success = True
        res.message = ("clamped to soft limit" if clamped
                       else f"jogged leg {n} by {req.delta_turns:+.3f}")
        res.final_pos_turns = float(new_target)
        return res

    def srv_set_pose(self, req, res):
        if not self.armed:
            res.success = False
            res.message = "not armed"
            res.any_clamped = False
            return res
        if self.limits is None:
            res.success = False
            res.message = "no leg_limits.yaml"
            res.any_clamped = False
            return res
        xyz = (float(req.x), float(req.y), float(req.z))
        rpy = (float(req.roll), float(req.pitch), float(req.yaw))
        # If PI level is on, its thread drives the pose each iter — here we
        # only update the commanded xyz and let the level loop add tilt.
        self.current_xyz = list(xyz)
        self.current_rpy = list(rpy)
        if self.level_enabled:
            # level thread will pick up new xyz on next iter
            res.success = True
            res.message = "pose set (level loop will track)"
            res.any_clamped = False
            return res
        targets, any_clamped = _compute_motor_targets(
            xyz, rpy, self.geom, self.limits)
        self.feeder.set_pos_targets(targets)
        res.success = True
        res.message = "pose sent"
        res.any_clamped = bool(any_clamped)
        return res

    def srv_set_speed_cap(self, req, res):
        v = float(req.data)
        v = max(0.1, min(v, self.hard_max_vel))
        self.soft_max_vel = v
        res.success = True
        res.message = f"soft_max_vel = {v:.3f} turns/s"
        return res

    def srv_get_speed_cap(self, req, res):
        res.success = True
        res.message = json.dumps({
            'soft_max_leg_vel_turns_per_sec': self.soft_max_vel,
            'hard_max_leg_vel_turns_per_sec': self.hard_max_vel,
            'default_soft_max_tilt_rate_deg_per_sec': float(
                self.global_limits.get('default_soft_max_tilt_rate_deg_per_sec',
                                       8.0)),
            'hard_max_tilt_rate_deg_per_sec': float(
                self.global_limits.get('hard_max_tilt_rate_deg_per_sec',
                                       15.0)),
        })
        return res

    def srv_enable_level(self, req, res):
        cmd = (req.command or '').strip().lower()
        want = cmd in ('activate', 'on', 'enable', 'true', '1')
        if want:
            if not self.armed:
                res.success = False
                res.message = "arm first"
                return res
            cal = _load_level_cal()
            if cal is None:
                res.success = False
                res.message = f"no {LEVEL_CAL_PATH} — capture level first"
                return res
            self.level_ref_roll, self.level_ref_pitch = cal
            self._start_level_loop()
            res.success = True
            res.message = (f"level enabled (ref roll={self.level_ref_roll:+.3f}"
                           f" pitch={self.level_ref_pitch:+.3f})")
        else:
            self._stop_level_loop()
            res.success = True
            res.message = "level disabled"
        return res

    # ---- level loop ----
    def _start_level_loop(self):
        self._stop_level_loop()
        self.level_corr = [0.0, 0.0]
        self.level_stop.clear()
        self.level_enabled = True
        self.level_thread = threading.Thread(
            target=self._level_run, daemon=True)
        self.level_thread.start()

    def _stop_level_loop(self):
        self.level_enabled = False
        self.level_stop.set()
        t = self.level_thread
        if t is not None:
            t.join(timeout=1.0)
        self.level_thread = None

    def _level_run(self):
        err_r_f = 0.0
        err_p_f = 0.0
        integ_r = 0.0
        integ_p = 0.0
        tilt_r_corr = 0.0
        tilt_p_corr = 0.0
        while not self.level_stop.is_set():
            t0 = time.monotonic()
            with self.imu_lock:
                rpy = self.imu_rpy
                last_rx = self.imu_last_rx
            if rpy is None or (t0 - last_rx) > 0.3:
                time.sleep(CTRL_PERIOD_S)
                continue
            # DYNAMIC reference: target IMU reading follows the commanded
            # pose (current_rpy). That way a routine commanding roll=+2°
            # produces an actual world-frame roll of +2° rather than being
            # cancelled by the PI. When current_rpy == 0 (the "hold level"
            # case) this collapses back to static ref = level_ref.
            target_r = self.level_ref_roll  + self.current_rpy[0]
            target_p = self.level_ref_pitch + self.current_rpy[1]
            err_r = rpy[0] - target_r
            err_p = rpy[1] - target_p
            err_r_f = LEVEL_FILTER_ALPHA * err_r + (1 - LEVEL_FILTER_ALPHA) * err_r_f
            err_p_f = LEVEL_FILTER_ALPHA * err_p + (1 - LEVEL_FILTER_ALPHA) * err_p_f
            if abs(err_r_f) > LEVEL_DEADBAND:
                integ_r += -err_r_f * LEVEL_KI * CTRL_PERIOD_S
            if abs(err_p_f) > LEVEL_DEADBAND:
                integ_p += -err_p_f * LEVEL_KI * CTRL_PERIOD_S
            integ_r = max(-LEVEL_MAX_CORR, min(LEVEL_MAX_CORR, integ_r))
            integ_p = max(-LEVEL_MAX_CORR, min(LEVEL_MAX_CORR, integ_p))
            target_r = -err_r_f * LEVEL_KP + integ_r
            target_p = -err_p_f * LEVEL_KP + integ_p
            d_r = max(-LEVEL_RATE_LIMIT, min(LEVEL_RATE_LIMIT, target_r - tilt_r_corr))
            d_p = max(-LEVEL_RATE_LIMIT, min(LEVEL_RATE_LIMIT, target_p - tilt_p_corr))
            tilt_r_corr = max(-LEVEL_MAX_CORR, min(LEVEL_MAX_CORR, tilt_r_corr + d_r))
            tilt_p_corr = max(-LEVEL_MAX_CORR, min(LEVEL_MAX_CORR, tilt_p_corr + d_p))
            self.level_corr = [tilt_r_corr, tilt_p_corr]
            xyz = self.current_xyz
            rpy_cmd = [self.current_rpy[0] + tilt_r_corr,
                       self.current_rpy[1] + tilt_p_corr,
                       self.current_rpy[2]]
            if self.limits is not None and self.feeder is not None:
                targets, _ = _compute_motor_targets(
                    tuple(xyz), tuple(rpy_cmd), self.geom, self.limits)
                self.feeder.set_pos_targets(targets)
            sl = CTRL_PERIOD_S - (time.monotonic() - t0)
            if sl > 0:
                time.sleep(sl)

    # ---- homing subprocess ----
    def _do_start_homing(self, d):
        """d is a dict-like with the StartHoming fields. Returns (ok, msg)."""
        with self.homing_lock:
            if self.homing_proc is not None and self.homing_proc.poll() is None:
                return False, "homing already running"

        def getv(k, default=None):
            v = d.get(k, default) if hasattr(d, 'get') else getattr(d, k, default)
            return v

        # -u makes Python's stdout/stderr unbuffered so input() prompts
        # actually reach the pipe before the user responds. Needed so the
        # GUI homing log can show "[y]=save+park / [r]=retry / ..." in
        # real time rather than batched at process exit.
        argv = [sys.executable, '-u', STALL_HOME_SCRIPT]
        node = getv('node', -1)
        nodes = getv('nodes', '')
        if node is not None and isinstance(node, (int, float)) and node >= 0:
            argv += ['--node', str(int(node))]
        elif nodes:
            argv += ['--nodes', str(nodes)]
        else:
            return False, "must specify either node (>=0) or nodes (non-empty)"

        argv += ['--vel', f"{float(getv('vel', 0.0))}"]
        argv += ['--current', f"{float(getv('current', 0.0))}"]
        vl = getv('vel_limit', 0.0)
        if vl and vl > 0:    argv += ['--vel-limit', f"{float(vl)}"]
        tc = getv('travel_cap', 0.0)
        if tc and tc > 0:    argv += ['--travel-cap', f"{float(tc)}"]
        svf = getv('stall_vel_frac', 0.0)
        if svf and svf > 0:  argv += ['--stall-vel-frac', f"{float(svf)}"]
        fmf = getv('free_motion_frac', 0.0)
        if fmf and fmf > 0:  argv += ['--free-motion-frac', f"{float(fmf)}"]
        sd = getv('stall_dur', 0.0)
        if sd and sd > 0:    argv += ['--stall-dur', f"{float(sd)}"]
        pr = getv('pre_roll', 0.0)
        if pr and pr > 0:    argv += ['--pre-roll', f"{float(pr)}"]
        bl = getv('baseline', 0.0)
        if bl and bl > 0:    argv += ['--baseline', f"{float(bl)}"]
        rt = getv('retract', 0.0)
        if rt and not (isinstance(rt, float) and math.isnan(rt)) and rt != 0:
            argv += ['--retract', f"{float(rt)}"]
        rm = getv('retract_mode', '')
        if rm:               argv += ['--retract-mode', str(rm)]
        rv = getv('retract_vel', 0.0)
        if rv and rv > 0:    argv += ['--retract-vel', f"{float(rv)}"]
        rc = getv('retract_current', 0.0)
        if rc and rc > 0:    argv += ['--retract-current', f"{float(rc)}"]
        ds = getv('distance', 0.0)
        if ds and ds > 0:    argv += ['--distance', f"{float(ds)}"]
        if getv('dry_run', False):         argv += ['--dry-run']
        if getv('collect', False):         argv += ['--collect']
        if getv('no_stall', False):        argv += ['--no-stall']
        if getv('go_to_rest', False):      argv += ['--go-to-rest']
        if getv('no_park', False):         argv += ['--no-park']
        if getv('measure_stroke', False):  argv += ['--measure-stroke']
        if getv('apply_to_all_legs', False): argv += ['--apply-to-all-legs']
        if getv('manual_capture', False):  argv += ['--manual-capture']

        if not os.path.exists(STALL_HOME_SCRIPT):
            return False, f"stall_home.py not found at {STALL_HOME_SCRIPT}"

        self.get_logger().info(
            f"releasing CAN and spawning stall_home: {' '.join(argv)}")
        self._close_bus_and_stop_threads()

        try:
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'  # belt-and-suspenders with -u
            self.homing_proc = subprocess.Popen(
                argv,
                stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                # Binary mode so the pump can do non-blocking os.read();
                # decoding happens in the pump.
                text=False, bufsize=0,
                preexec_fn=os.setsid,
                env=env,
            )
        except Exception as e:
            self._open_bus_and_start_threads()
            return False, f"failed to spawn: {e}"

        threading.Thread(target=self._homing_output_pump, daemon=True).start()
        return True, f"homing started (pid={self.homing_proc.pid})"

    def _do_cancel_homing(self):
        p = self.homing_proc
        if p is None or p.poll() is not None:
            return False, "no homing process running"
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
            return True, "SIGINT sent"
        except Exception as e:
            return False, f"cancel failed: {e}"

    def srv_start_homing(self, req, res):
        ok, msg = self._do_start_homing(req)
        res.accepted = ok
        res.message = msg
        return res

        res.accepted = True
        res.message = f"homing started (pid={self.homing_proc.pid})"
        return res

    def _homing_output_pump(self):
        """Stream stall_home.py stdout (merged with stderr) onto the
        /homing_output topic. Handles interactive prompts (text with no
        trailing newline) by flushing whatever's buffered after a short
        idle window, so the GUI can show '[y]=save+park / [r]=retry /
        [s]=skip / [q]=quit:' before the user responds."""
        import fcntl
        import select as _select

        proc = self.homing_proc
        if proc is None:
            return
        fd = proc.stdout.fileno()
        # Non-blocking reads so select drives the pace.
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        buf = b''
        last_data = time.monotonic()
        IDLE_FLUSH_S = 0.15
        last_partial = None  # last partial-line publish, to suppress duplicates

        def _publish(text, partial=False):
            if not text:
                return
            nonlocal last_partial
            if partial and text == last_partial:
                return  # skip duplicate idle-flush of same prompt
            last_partial = text if partial else None
            m = String()
            m.data = text
            self.pub_homing_out.publish(m)

        try:
            while True:
                alive = proc.poll() is None
                r, _, _ = _select.select([fd], [], [], 0.1)
                if fd in r:
                    try:
                        chunk = os.read(fd, 4096)
                    except BlockingIOError:
                        chunk = b''
                    if chunk == b'':
                        # EOF from the read side
                        if not alive:
                            break
                    else:
                        buf += chunk
                        last_data = time.monotonic()
                        # Publish every complete line as we see it
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            _publish(line.decode('utf-8', errors='replace'))
                        continue
                # No new data this tick. If there's leftover buffered
                # content that's been idle a moment, it's probably a
                # prompt waiting for input → flush it so the GUI shows it.
                if buf and (time.monotonic() - last_data) > IDLE_FLUSH_S:
                    _publish(buf.decode('utf-8', errors='replace'), partial=True)
                    buf = b''
                if not alive:
                    break
        except Exception as e:
            _publish(f"[pump error] {e}")
        finally:
            # Drain any final bytes after child exit
            try:
                while True:
                    chunk = os.read(fd, 65536)
                    if not chunk:
                        break
                    buf += chunk
            except Exception:
                pass
            if buf:
                _publish(buf.decode('utf-8', errors='replace'))

        try:
            proc.wait(timeout=1.0)
        except Exception:
            pass
        _publish(f"[homing exited with code {proc.returncode}]")
        # Reload leg_limits.yaml (stall_home may have rewritten it), reopen bus
        self.limits = _load_leg_limits()
        self._open_bus_and_start_threads()
        self.homing_proc = None

    def srv_cancel_homing(self, req, res):
        ok, msg = self._do_cancel_homing()
        res.success = ok
        res.message = msg
        return res

    def _homing_stdin_cb(self, msg):
        p = self.homing_proc
        if p is None or p.poll() is not None:
            return
        try:
            # subprocess is binary-mode now; encode text + newline
            p.stdin.write((msg.data + '\n').encode('utf-8'))
            p.stdin.flush()
        except Exception as e:
            self.get_logger().warn(f"homing_stdin write failed: {e}")

    # ---- shutdown ----
    def shutdown(self):
        self.get_logger().info("shutting down stewart_control_node")
        if self.rlog_dir is not None:
            self._stop_recording_log()
        self._stop_level_loop()
        if self.homing_proc is not None and self.homing_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.homing_proc.pid), signal.SIGTERM)
            except Exception:
                pass
        self._close_bus_and_stop_threads()


def main(args=None):
    rclpy.init(args=args)
    node = StewartControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
