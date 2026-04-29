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
from jugglebot_interfaces.msg import LevelDiag


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
LEVEL_GAINS_PATH = os.path.join(_BRINGUP_DIR, 'config/level_gains.yaml')
BAGS_DIR = os.path.expanduser('~/stable_bot_bags')
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
CMD_RX_SDO            = 0x004   # outbound SDO request to a drive
CMD_TX_SDO            = 0x005   # response from a drive
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
LEVEL_KI = 0.2
LEVEL_FILTER_ALPHA = 0.3
LEVEL_RATE_LIMIT = 0.2
LEVEL_MAX_CORR = 5.0
LEVEL_DEADBAND = 0.05
# Reference rate at which the level gains were originally tuned.
# `filter_alpha` and `rate_limit_deg_per_iter` are interpreted PHYSICALLY
# (cutoff Hz, slew °/s respectively) and scaled at use-time to the actual
# loop rate, so a YAML tuned at 50 Hz remains valid at 200 Hz.
LEVEL_REF_HZ = 50.0
# Default loop rate. Override via the `--level-loop-hz <Hz>` CLI flag.
# 200 Hz puts the outer loop well above the platform's mechanical bandwidth
# while staying ~42% bus-headroom on a 1 Mbps CAN: ~574 kbps total
# (Set_Input_Pos × 6 @ 200 Hz + encoder broadcasts × 6 @ 500 Hz + heartbeats
# + RTRs).
LEVEL_DEFAULT_HZ = 200.0


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


def _load_level_gains():
    """Returns dict with kp, ki, deadband_deg, rate_limit_deg_per_iter,
    max_corr_deg, filter_alpha, plus the inner-deadband integrator-decay
    knob. Falls back to historical hardcoded values if the YAML is
    missing so the node still starts."""
    defaults = {
        'kp': 0.7, 'ki': 0.2, 'deadband_deg': 0.05,
        'rate_limit_deg_per_iter': 0.2, 'max_corr_deg': 5.0,
        'filter_alpha': 0.3,
        # If True AND empirical_ik is loaded, the level loop uses the
        # measured platform Jacobian's pinv to map (corr_r, corr_p, 0)
        # to leg deltas instead of the geometric IK. The geometric IK
        # is still used for the translation part (xyz). Lets us A/B
        # the empirical replacement vs the original IK by toggling
        # the gain via the GUI.
        'use_empirical_ik': True,
        # Integrator-decay band:
        #   |err_filt| > integ_decay_outer_deg → full integration, no decay
        #   integ_decay_outer_deg ≥ |err_filt| ≥ deadband_deg → linear blend
        #   |err_filt| < deadband_deg → no integration, full decay
        #
        # The blend range is what addresses the "loop keeps correcting
        # at small but non-zero error" pattern observed 2026-04-29 —
        # decay engages well before the deadband so wind-up bleeds off
        # while error is still in the 0.02-0.10° range that the loop
        # spends most of its time at. Set outer == deadband to disable
        # the blend zone (binary in/out of deadband, original behavior).
        'integ_decay_outer_deg': 0.10,
        # Decay strength PER TICK at the reference rate (50 Hz). Scaled
        # automatically to whatever loop rate is actually running.
        # 0.99 ⇒ ~1 second time constant. 1.0 disables decay entirely.
        'integ_decay_per_tick_at_50hz': 0.99,
    }
    if not os.path.exists(LEVEL_GAINS_PATH):
        return defaults
    try:
        with open(LEVEL_GAINS_PATH) as f:
            doc = yaml.safe_load(f) or {}
    except Exception:
        return defaults
    out = dict(defaults)
    for k in defaults:
        if k in doc:
            out[k] = float(doc[k])
    return out


def _compute_motor_targets(xyz, rpy, geom, limits, empirical_ik=None):
    """Compute the 6 leg-position targets (turns) for a desired
    platform pose. By default this is the GEOMETRIC IK (the original
    behavior). When `empirical_ik` is provided, the rotational part
    of the pose (rpy) is mapped via the measured platform Jacobian
    instead of the geometric IK — translational part (xyz) is still
    handled geometrically because we never measured x/y perturbation
    response. Hybrid IK in other words: geometric translation,
    empirical rotation.
    """
    if empirical_ik is None:
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
    # Hybrid path. Base targets at zero rpy, geometric.
    base_ext = _leg_extensions_mm(xyz, (0.0, 0.0, 0.0), geom)
    targets = np.zeros(6)
    for i in range(6):
        ext_turns = MOTOR_EXTENSION_SIGN * base_ext[i] / MM_PER_REV
        targets[i] = limits[i]['rest'] + ext_turns
    # Empirical rotation: pinv(J_at_z) @ rpy (in degrees) → leg deltas (turns).
    leg_deltas = empirical_ik.rpy_to_leg_deltas(xyz[2], rpy)
    targets = targets + np.asarray(leg_deltas)
    # Re-clamp to soft limits.
    any_clamped = False
    for i in range(6):
        lo, hi = limits[i]['lo'], limits[i]['hi']
        if targets[i] < lo:
            targets[i] = lo; any_clamped = True
        elif targets[i] > hi:
            targets[i] = hi; any_clamped = True
    return targets, any_clamped


class EmpiricalIK:
    """Loads a jacobian.json (from system-id) and provides a
    Z-interpolated rpy-to-leg-deltas mapping. Used by the level loop
    as a drop-in replacement for the rotational part of the geometric
    IK, accounting for real per-leg gain mismatches.

    The mapping is `leg_deltas (turns) = pinv(J_z) @ rpy (degrees)`
    where J_z is the empirical Jacobian at Z, averaged across the
    "clean" deltas (≥0.05 turns) to skip the stiction-noise-dominated
    small-δ measurements.
    """

    # Skip δ smaller than this when averaging the empirical Jacobian.
    # The 2026-04-29 sysid run showed the small-δ cells are
    # disproportionately affected by stiction (leg 4's roll response
    # doubles between δ=0.025 and δ=0.10), so they shouldn't dominate
    # the IK fit.
    MIN_DELTA_FOR_FIT = 0.05

    def __init__(self, jacobian_path):
        self.path = jacobian_path
        self.z_list = []
        self.pinv_per_z = []   # list of np.ndarray (6, 3)
        self.J_per_z = []      # list of np.ndarray (3, 6) — raw J for diagnostics
        self.loaded_at = None
        self._load()

    def is_loaded(self):
        return bool(self.z_list)

    def _load(self):
        try:
            with open(self.path) as f:
                doc = json.load(f)
        except Exception as e:
            self.load_error = f"open: {e}"
            return
        emp = doc.get('empirical_jacobian') or {}
        rows = []
        for z_str, per_d in emp.items():
            try:
                z = float(z_str)
            except ValueError:
                continue
            jacs = []
            for d_str, jac in per_d.items():
                try:
                    d = float(d_str)
                except ValueError:
                    continue
                if d < self.MIN_DELTA_FOR_FIT:
                    continue
                arr = np.array([
                    [float(v) if v is not None else np.nan for v in row]
                    for row in jac
                ], dtype=float)
                jacs.append(arr)
            if not jacs:
                continue
            mean_J = np.nanmean(np.stack(jacs), axis=0)
            mean_J = np.nan_to_num(mean_J, nan=0.0)
            pinv_J = np.linalg.pinv(mean_J)
            rows.append((z, mean_J, pinv_J))
        rows.sort(key=lambda r: r[0])
        self.z_list = [r[0] for r in rows]
        self.J_per_z = [r[1] for r in rows]
        self.pinv_per_z = [r[2] for r in rows]
        self.loaded_at = doc.get('started_utc')

    def _interp_pinv(self, z_mm):
        """Linear interpolation of pinv(J) on Z. Clamps outside the
        measured range to the nearest endpoint — safer than
        extrapolating the empirical fit."""
        if not self.z_list:
            return None
        z = float(z_mm)
        if z <= self.z_list[0]:
            return self.pinv_per_z[0]
        if z >= self.z_list[-1]:
            return self.pinv_per_z[-1]
        for i in range(len(self.z_list) - 1):
            z_lo, z_hi = self.z_list[i], self.z_list[i + 1]
            if z_lo <= z < z_hi:
                a = (z - z_lo) / (z_hi - z_lo)
                return (1.0 - a) * self.pinv_per_z[i] + a * self.pinv_per_z[i + 1]
        return self.pinv_per_z[-1]

    def rpy_to_leg_deltas(self, z_mm, rpy_deg):
        """Returns 6-vector of leg-position deltas (turns) that
        achieve the desired (roll, pitch, yaw) at Z=z_mm. Yaw is
        included even though the level loop usually commands yaw=0;
        the model captures real yaw response from individual legs and
        passing 0 just means the inversion finds the leg combo that
        produces the desired roll/pitch with minimal yaw side-effect."""
        pinv = self._interp_pinv(z_mm)
        if pinv is None:
            return np.zeros(6)
        rpy_arr = np.array([float(rpy_deg[0]), float(rpy_deg[1]),
                            float(rpy_deg[2]) if len(rpy_deg) > 2 else 0.0],
                           dtype=float)
        return pinv @ rpy_arr


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
        self.vel_by_node = {}     # node -> (vel_turns_per_sec, rx)  — same CAN frame as pos
        self.err_by_node = {}     # node -> (active_errors, disarm_reason, rx)
        self.iq_by_node = {}      # node -> (iq_setpoint, iq_measured, rx)
        self.state_by_node = {}   # node -> (axis_state_uint8, rx)  — from Heartbeat (cmd 0x001)
        # Pending SDO read replies keyed by (node_id, endpoint_id). Filled
        # by _rx_loop when a Tx_SDO frame arrives; consumed by read_sdo.
        # We keep the bus_lock + send out of the way of the listener
        # thread by sending requests via the parent's bus directly.
        self.sdo_replies = {}
        self.sdo_lock = threading.Lock()
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
                # ODrive's Get_Encoder_Estimates frame is 8 bytes:
                # bytes [0:4] = pos (turns float), [4:8] = vel (turns/s float).
                # Older firmware used to send only 4 bytes; keep that path for safety.
                pos = struct.unpack('<f', bytes(msg.data[:4]))[0]
                vel = (struct.unpack('<f', bytes(msg.data[4:8]))[0]
                       if len(msg.data) >= 8 else float('nan'))
                with self.lock:
                    now = time.monotonic()
                    self.pos_by_node[node] = (pos, now)
                    if not (vel != vel):   # not NaN
                        self.vel_by_node[node] = (vel, now)
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
            elif cmd == CMD_TX_SDO and len(msg.data) >= 4:
                # SDO read response. Prologue is <BHB> (opcode, ep_id,
                # reserved); the remaining bytes are the typed value.
                _opcode, ep_id, _res = struct.unpack(
                    '<BHB', bytes(msg.data[:4]))
                payload = bytes(msg.data[4:])
                with self.sdo_lock:
                    self.sdo_replies[(node, ep_id)] = (
                        payload, time.monotonic())
            elif cmd == CMD_HEARTBEAT and len(msg.data) >= 5:
                # ODrive 0.6.x heartbeat layout: [0:4]=axis_error (legacy,
                # often 0), [4]=axis_state, [5]=procedure_result, etc. We
                # only care about state for diagnostics — broadcast at
                # 10 Hz natively, no RTR needed.
                state = int(msg.data[4])
                with self.lock:
                    self.state_by_node[node] = (state, time.monotonic())

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

    def get_states(self, max_age_s=2.0):
        """Returns list of 6 axis_state uint8 values. 0 = no fresh
        heartbeat (sentinel for "missing/stale" — real ODrive states
        are 1, 3, 4, 6, 7, 8, 11; 0 is undefined and we co-opt it)."""
        out = [0] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.state_by_node.get(n)
                if v is not None and (now - v[1]) < max_age_s:
                    out[n] = int(v[0])
        return out

    def get_iq_setpoint(self, max_age_s=2.0):
        """Returns list of 6 iq_setpoint floats (A) — what the ODrive
        controller is commanding the motor to draw — or NaN if stale.
        The gap between iq_setpoint and iq_measured tells you about
        the motor's electrical/torque tracking."""
        out = [float('nan')] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.iq_by_node.get(n)
                if v is not None and (now - v[2]) < max_age_s:
                    out[n] = float(v[0])
        return out

    def get_vel(self, max_age_s=0.5):
        """Returns list of 6 leg velocities (turns/s) or NaN if no fresh
        reading. Comes from the same Get_Encoder_Estimates frame as pos."""
        out = [float('nan')] * 6
        with self.lock:
            now = time.monotonic()
            for n in range(6):
                v = self.vel_by_node.get(n)
                if v is not None and (now - v[1]) < max_age_s:
                    out[n] = float(v[0])
        return out

    def write_sdo(self, bus_lock, node_id, endpoint_id, value, kind):
        """Fire-and-forget SDO write. ODrive doesn't echo writes, so
        there's no response to poll — we just send the frame and move
        on. Use this for runtime self-heal of params that don't have a
        dedicated CAN command (e.g. wL_FF_enable, vel_integrator_gain).
        Returns True on send success, False if the frame failed."""
        arb = (node_id << 5) | CMD_RX_SDO
        prologue = struct.pack('<BHB', 0x01, int(endpoint_id), 0)
        # Pack the value bytes. Mirrors the configurator's _pack_value.
        if kind == 'bool':
            payload = struct.pack('<B', 1 if value else 0)
        elif kind in ('uint8', 'int8'):
            payload = struct.pack('<B' if kind.startswith('u') else '<b',
                                  int(value))
        elif kind in ('uint16',):
            payload = struct.pack('<H', int(value))
        elif kind in ('uint32',):
            payload = struct.pack('<I', int(value))
        elif kind in ('int16',):
            payload = struct.pack('<h', int(value))
        elif kind in ('int32',):
            payload = struct.pack('<i', int(value))
        elif kind in ('float32', 'float'):
            payload = struct.pack('<f', float(value))
        elif kind == 'float64':
            payload = struct.pack('<d', float(value))
        else:
            return False
        data = prologue + payload
        if len(data) > 8:
            return False
        if len(data) < 8:
            data = data + b'\x00' * (8 - len(data))
        with bus_lock:
            try:
                self.bus.send(can.Message(
                    arbitration_id=arb, data=data,
                    is_extended_id=False), timeout=0.2)
                return True
            except Exception:
                return False

    def read_sdo(self, bus_lock, node_id, endpoint_id, kind, timeout=1.0):
        """Single-frame SDO read. The listener handles dispatch in its
        own thread; this just sends the request and polls sdo_replies.

        Caller must pass `bus_lock` (the parent node's lock) so the send
        doesn't race with the feeder. `kind` is one of float32/float64/
        bool/uint8/uint16/uint32/int8/int16/int32 — same vocabulary as
        the configurator's _unpack_value."""
        # Clear any stale reply for this (node, endpoint) so we don't
        # match against a leftover.
        key = (node_id, int(endpoint_id))
        with self.sdo_lock:
            self.sdo_replies.pop(key, None)
        # Send Rx_SDO read request: <BHB> = (opcode=0, endpoint_id, 0)
        arb = (node_id << 5) | CMD_RX_SDO
        prologue = struct.pack('<BHB', 0x00, int(endpoint_id), 0)
        # CAN classic: pad payload to 8 bytes; some adapters dislike short DLC.
        data = prologue + b'\x00' * (8 - len(prologue))
        with bus_lock:
            try:
                self.bus.send(can.Message(
                    arbitration_id=arb, data=data,
                    is_extended_id=False), timeout=0.2)
            except Exception:
                return None
        # Poll for the reply.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.sdo_lock:
                entry = self.sdo_replies.pop(key, None)
            if entry is not None:
                payload, _t_rx = entry
                return _decode_sdo_value(payload, kind)
            time.sleep(0.001)
        return None


def _decode_sdo_value(data, kind):
    """Decode an SDO-read payload (bytes after the 4-byte prologue).
    Mirrors the configurator's _unpack_value so the value semantics
    are identical between the two code paths."""
    if not data:
        return None
    if kind in ('float32', 'float'):
        return struct.unpack('<f', data[:4])[0] if len(data) >= 4 else None
    if kind == 'float64':
        return struct.unpack('<d', data[:8])[0] if len(data) >= 8 else None
    if kind == 'bool':
        return bool(data[0])
    if kind in ('uint8', 'int8'):
        return int(data[0])
    if kind == 'uint16':
        return struct.unpack('<H', data[:2])[0] if len(data) >= 2 else None
    if kind == 'uint32':
        return struct.unpack('<I', data[:4])[0] if len(data) >= 4 else None
    if kind == 'int16':
        return struct.unpack('<h', data[:2])[0] if len(data) >= 2 else None
    if kind == 'int32':
        return struct.unpack('<i', data[:4])[0] if len(data) >= 4 else None
    return None


class ODriveFeeder:
    """50 Hz background publisher. Per-leg mode is one of:
        'pos'  -> send Set_Input_Pos(pos_targets[n])
        'vel'  -> send Set_Input_Vel(shared vel_target, 0)
        'idle' -> send nothing (axis is in STATE_IDLE, watchdog not active)
    This unifies the pose-mode holding loop and the dead-man velocity jog."""

    def __init__(self, bus, initial_targets, period=0.02, safety_check=None):
        self.bus = bus
        self.period = period
        self.lock = threading.Lock()
        self.pos_targets = np.array(initial_targets, dtype=float).copy()
        self.vel_target = 0.0
        self.modes = ['idle'] * 6
        self.stop_flag = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        # Final-line-of-defense safety hook. Called every tick before any
        # CAN frame is sent. Receives (modes, vel, pos_targets) and returns
        # (safe_vel, safe_pos_targets). Lets the Node enforce position
        # clamps + position-aware velocity safety against current encoder
        # readings — even if a malformed /control_cmd or buggy code path
        # ever sets pos_targets/vel directly. None = pass-through.
        self.safety_check = safety_check

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
            # SAFETY GATE: every tick, give the Node a chance to clamp
            # pos_t per-leg to the current soft limits and zero vel if any
            # vel-mode leg is about to drive past an endstop. This is the
            # last line of defense — any code path that bypasses the
            # higher-level checks still gets caught here before the CAN
            # frame is sent.
            if self.safety_check is not None:
                try:
                    safe_vel, safe_pos = self.safety_check(modes, vel, pos_t)
                    vel = float(safe_vel)
                    pos_t = np.asarray(safe_pos, dtype=float).copy()
                except Exception:
                    # Safety check is best-effort; never crash the feeder
                    # if the Node hands us a bad lambda. Silent fall-through
                    # to unclamped command — but the Node's higher-level
                    # checks should still cover it.
                    pass
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
    def __init__(self, level_loop_hz=None):
        super().__init__('stewart_control_node')
        self.get_logger().info("starting stewart_control_node")
        # Outer-loop sample period. Driven by --level-loop-hz; defaults to
        # LEVEL_DEFAULT_HZ. ki, alpha, and rate_limit are scaled at use
        # time so the gains in level_gains.yaml retain their physical
        # meaning regardless of the loop rate.
        rate_hz = float(level_loop_hz) if level_loop_hz else LEVEL_DEFAULT_HZ
        self.ctrl_period_s = 1.0 / max(1.0, rate_hz)
        self.level_loop_hz = rate_hz
        self.get_logger().info(
            f"level loop rate: {rate_hz:.1f} Hz "
            f"(period {self.ctrl_period_s*1000:.2f} ms)")

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

        # Level-PI tuning support — see docs/level_pi_tuning_plan.md.
        self.level_gains = _load_level_gains()
        # Empirical IK from system-id. Loaded if a jacobian.json exists
        # under tuning_data/system_id_*/. Picks the most recent run.
        # If no jacobian available, level loop falls back to geometric IK.
        self.empirical_ik = self._load_empirical_ik()
        if self.empirical_ik is not None and self.empirical_ik.is_loaded():
            self.get_logger().info(
                f"empirical IK loaded from {self.empirical_ik.path} "
                f"({len(self.empirical_ik.z_list)} Z heights, "
                f"started {self.empirical_ik.loaded_at})")
        else:
            self.get_logger().info(
                "empirical IK not loaded — using geometric IK only")
        # Step-injection offsets read by _level_run on each tick. Driven
        # by the step-test routine and the auto-sweep step battery.
        self.level_step_offset_roll = 0.0
        self.level_step_offset_pitch = 0.0
        # External integrator-reset flag. _level_run picks it up at the
        # top of the next tick and zeros integ_r/integ_p without any
        # mechanical transient (cleaner than disarm-and-snap).
        self._level_zero_integ_request = threading.Event()
        # `ros2 bag record` subprocess for single recordings (not the
        # auto-sweep — that manages its own per-Z bag children). Guarded
        # by a lock; one bag at a time.
        self._bag_lock = threading.Lock()
        self._bag_proc = None
        self._bag_dir = None
        self._bag_t0 = 0.0
        # Auto-sweep thread state.
        self._sweep_lock = threading.Lock()
        self._sweep_thread = None
        self._sweep_stop = threading.Event()
        self._sweep_state = {'state': 'idle'}   # current snapshot for /level_record_state
        self._sweep_state_lock = threading.Lock()

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
        # Level-PI tuning publishers. /level_diag carries the per-tick
        # controller state (typed, recorded into bags). /level_record_state
        # carries the bag/sweep status for the GUI (JSON String, 5 Hz heartbeat).
        self.pub_level_diag = self.create_publisher(LevelDiag, 'level_diag', 50)
        self.pub_level_record_state = self.create_publisher(
            String, 'level_record_state', 10)

        # Dead-man jog watchdog: if /jog_vel_cmd stops arriving for > 0.5 s
        # while any leg is in vel mode, force vel target back to 0 so the
        # motor cannot run away if the browser disconnects mid-drag.
        self._last_jog_vel_rx = 0.0
        self._last_jog_vel_value = 0.0
        # CAN bus utilization (rolling delta vs previous _tick_status sample)
        self._last_can_stats = None

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
        self.create_timer(0.2, self._tick_level_record_state)  # 5 Hz heartbeat

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
            self.feeder = ODriveFeeder(
                self.bus, rest_targets,
                safety_check=self._feeder_safety_check)
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
    def _arm_leg_internal(self, n, mode, current=None, vel_limit=None,
                          force_no_limits=False):
        """Arm a single leg in the given mode ('pos' or 'vel'), or disarm
        (mode='idle'). Feeder modes are updated BEFORE the ODrive state
        transition so watchdog stays fed through the transition.

        `current` defaults to self.leg_current_a (the value last set
        via the GUI's set_leg_current slider) when not specified. Pre-
        2026-04-29 the default was hardcoded 6.0 A — every arm wiped
        the runtime current_soft_max regardless of what the slider had
        been moved to, so a slider set to 11 A had no effect after the
        next arm click. Confirmed via the inner_loop_config snapshot
        showing 6.0 A on all 6 drives despite the user reporting the
        slider was at 11.

        force_no_limits=True bypasses the leg_limits.yaml check for pos
        mode. Only safe when the caller is going to hold the leg at its
        CURRENT encoder reading and not command any new positions —
        no soft-limit clamping happens, so any subsequent /set_pose or
        /jog_leg can still drive into mechanical endstops. Used by the
        'safe arm' path for pre-homing lock-in-place (spec: closed-loop
        ball demos discussion 2026-04-26)."""
        assert mode in ('pos', 'vel', 'idle')
        if current is None:
            current = float(self.leg_current_a)
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

        # Capture the freshest encoder reading just before sending the
        # CAN sequence — used to explicitly seed input_pos right before
        # CLOSED_LOOP_CONTROL so ODrive doesn't briefly use a stale or
        # default (often zero) input_pos register.
        seed_for_arm = None
        if mode == 'pos':
            enc_now = self.listener.get_all(max_age_s=0.5)
            if enc_now[n] is not None:
                seed_for_arm = float(enc_now[n])
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
                # CRITICAL: explicitly write input_pos = current encoder
                # IMMEDIATELY before STATE=CLOSED_LOOP. The feeder thread
                # also sends Set_Input_Pos at 50 Hz, but if the ODrive
                # resets input_pos when CONTROLLER_MODE switches (or if
                # the feeder hasn't ticked yet), there's a window where
                # input_pos = 0. Encoder is typically nonzero, so the
                # controller drives the leg toward 0 — which is the
                # 2026-04-26 slam pattern. Belt-and-suspenders against
                # this: write the seed directly here, no time.sleep.
                if mode == 'pos' and seed_for_arm is not None:
                    _send_pos(self.bus, n, seed_for_arm,
                              vel_ff_tps=0.0, torque_ff_nm=0.0)
                elif mode == 'vel':
                    # For vel mode, explicitly zero the input_vel before
                    # CLOSED_LOOP so the leg doesn't take off.
                    _send_vel(self.bus, n, 0.0, 0.0)
                _send_cmd(self.bus, n, CMD_SET_AXIS_STATE,
                          struct.pack('<I', STATE_CLOSED_LOOP))
            except Exception as e:
                return False, f"arm send failed: {e}"
        time.sleep(0.1)
        self.leg_armed[n] = True
        return True, f"leg {n} armed ({mode})"

    def _arm_all_in_pos_mode(self, current=None, vel_limit=None,
                             force_no_limits=False):
        # `current=None` means "use the slider value" (self.leg_current_a).
        # Pass an explicit value only when the caller is doing a special
        # arm path (e.g. _safe_arm_in_place pre-homing).
        if current is None:
            current = float(self.leg_current_a)
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
        legs_derived = []  # legs where one endstop was derived
        ts = datetime.datetime.utcnow().isoformat(timespec='seconds') + '+00:00'
        for n in range(6):
            b, t = bot[n], top[n]
            if b is None and t is None:
                # Nothing captured for this leg — preserve any existing entry.
                if f'axis_{n}' in out:
                    legs_skipped.append(n)
                continue

            # SAFETY: determine the per-leg sign convention (signed stroke
            # from min toward max) WITHOUT guessing.
            #
            # 1. If both endstops captured this round: use observed.
            # 2. If only one captured + this leg has prior data in
            #    leg_limits.yaml: use the prior signed stroke. This handles
            #    encoder re-zeroing while preserving the per-leg direction
            #    that was empirically correct before.
            # 3. Otherwise: REFUSE. Guessing a default direction is what
            #    caused the slam on 2026-04-26 — never again.
            signed_stroke = None
            if b is not None and t is not None:
                signed_stroke = t - b
                if abs(signed_stroke) < 0.01:
                    return False, (
                        f"leg {n}: bottom and top capture are too close "
                        f"({signed_stroke:+.4f} turns). Did you capture "
                        f"them at the same physical position?")
            else:
                # Single-endstop case. Look up prior data for this leg.
                ax_existing = existing.get(f'axis_{n}', {}) or {}
                ex_min = ax_existing.get('min_pos_turns')
                ex_max = ax_existing.get('max_pos_turns')
                if (ex_min is not None and ex_max is not None
                        and abs(float(ex_max) - float(ex_min)) > 0.01):
                    # Use the magnitude clamp: prefer caller-supplied
                    # default_stroke_turns scaled by the observed sign,
                    # so an explicit user override still applies.
                    sign = 1.0 if (float(ex_max) - float(ex_min)) > 0 else -1.0
                    signed_stroke = sign * default_stroke_turns
                else:
                    return False, (
                        f"leg {n}: only one endstop captured AND no prior "
                        f"leg_limits.yaml entry to infer the per-leg sign "
                        f"convention. REFUSING to guess — guessing was the "
                        f"slam bug on 2026-04-26. Capture BOTH endstops for "
                        f"leg {n} (or first run a working calibration of any "
                        f"leg as a sign reference) before saving.")

            # Derive whichever endstop wasn't captured.
            if b is None:
                b = t - signed_stroke
                legs_derived.append(n)
            elif t is None:
                t = b + signed_stroke
                legs_derived.append(n)

            # Rest is rest_offset_turns "above" bottom, where "above" = the
            # signed direction toward top.
            up_dir = 1.0 if signed_stroke > 0 else -1.0
            rest = b + up_dir * rest_offset_turns
            out[f'axis_{n}'] = {
                'min_pos_turns': float(b),
                'max_pos_turns': float(t),
                'rest_pos_turns': float(rest),
                'homed_at': ts,
                'source': 'manual_endstop_capture',
                'rest_offset_turns': float(rest_offset_turns),
                'derived_endstop': n in legs_derived,
                'signed_stroke_turns': float(signed_stroke),
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

    def _safe_arm_in_place(self, current=None, vel_limit=None):
        """Arm all 6 legs at their current encoder positions, bypassing
        the leg_limits.yaml requirement. Use BEFORE homing to prevent
        legs from dropping under gravity during ODrive bench tests or
        partial homing attempts.

        `current=None` falls back to self.leg_current_a (the GUI
        slider's value). Pass an explicit value only if you have a
        specific reason to override.

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

    # ---- Feeder safety hook (last line of defense) ----
    def _feeder_safety_check(self, modes, vel, pos_targets):
        """Called by ODriveFeeder every 20 ms (50 Hz) before any CAN frame
        is sent. Returns (safe_vel, safe_pos_targets) with pos targets
        clamped to per-leg [lo, hi] from leg_limits.yaml.
        """
        if self.limits is None:
            return vel, pos_targets
        safe_pos = list(pos_targets)
        for n in range(6):
            if modes[n] != 'pos':
                continue
            lo = self.limits[n]['lo']
            hi = self.limits[n]['hi']
            if safe_pos[n] < lo:
                safe_pos[n] = lo
            elif safe_pos[n] > hi:
                safe_pos[n] = hi
        return vel, safe_pos

    # ---- CAN bus utilization ----
    # Read kernel-side packet/byte counters from /sys/class/net/can0/
    # statistics/. Cumulative since interface up; we sample twice (one
    # tick apart) and divide to get rates.
    CAN_IFACE = 'can0'
    CAN_BITRATE_BPS = 1_000_000  # classic CAN @ 1 Mbps (matches our `ip link set` cmd)
    # Average per-frame overhead for classic CAN, 11-bit IDs, with ~5%
    # bit-stuffing on the variable bits. Used to estimate wire bits/frame
    # from kernel byte/packet counters.
    _CAN_OVERHEAD_BITS = 47
    _CAN_STUFF_FACTOR = 1.05

    def _read_can_iface_stats(self):
        """Returns {'rx_packets', 'tx_packets', 'rx_bytes', 'tx_bytes',
        't_mono'} or None if can0 isn't up. We don't open the file
        descriptors persistently — on the Pi these reads are <100us.
        """
        base = f'/sys/class/net/{self.CAN_IFACE}/statistics'
        try:
            return {
                'rx_packets': int(open(f'{base}/rx_packets').read().strip()),
                'tx_packets': int(open(f'{base}/tx_packets').read().strip()),
                'rx_bytes':   int(open(f'{base}/rx_bytes').read().strip()),
                'tx_bytes':   int(open(f'{base}/tx_bytes').read().strip()),
                't_mono':     time.monotonic(),
            }
        except Exception:
            return None

    def _compute_can_rates(self, prev, cur):
        """Given two stats samples, compute per-second rates and an
        estimated bus utilization. The byte counters in /sys are DATA
        payload bytes (not wire bits), so we add per-frame overhead
        and a bit-stuffing factor to approximate actual bus bits."""
        if prev is None or cur is None:
            return None
        dt = cur['t_mono'] - prev['t_mono']
        if dt <= 0:
            return None
        d_rxp = cur['rx_packets'] - prev['rx_packets']
        d_txp = cur['tx_packets'] - prev['tx_packets']
        d_rxb = cur['rx_bytes']   - prev['rx_bytes']
        d_txb = cur['tx_bytes']   - prev['tx_bytes']
        rx_fps = d_rxp / dt
        tx_fps = d_txp / dt
        total_fps = rx_fps + tx_fps
        # Wire-bit estimate: data bytes × 8 × stuff_factor + per-frame overhead
        wire_bits = (
            (d_rxb + d_txb) * 8 * self._CAN_STUFF_FACTOR
            + (d_rxp + d_txp) * self._CAN_OVERHEAD_BITS
        )
        bus_bps = wire_bits / dt
        return {
            'can_rx_fps': float(rx_fps),
            'can_tx_fps': float(tx_fps),
            'can_total_fps': float(total_fps),
            'can_bus_kbps': float(bus_bps / 1000.0),
            'can_bus_utilization_pct': float(
                bus_bps / self.CAN_BITRATE_BPS * 100.0),
        }

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

        # CAN bus utilization — sample now, diff against previous tick.
        # Tick is at 5 Hz, so each datapoint covers ~200 ms; well below
        # the 1-sec window we'd want for human-readable rates but smooth
        # enough on a steady traffic pattern.
        can_now = self._read_can_iface_stats()
        can_rates = self._compute_can_rates(self._last_can_stats, can_now)
        self._last_can_stats = can_now

        status = {
            'armed': bool(self.armed),
            'limits_loaded': self.limits is not None,
            'bus_open': self.bus is not None,
            'imu_fresh': bool(imu_fresh),
            'level_enabled': bool(self.level_enabled),
            'homing_running': bool(homing_alive),
            'soft_max_vel_turns_per_sec': float(self.soft_max_vel),
            'hard_max_vel_turns_per_sec': float(self.hard_max_vel),
            'leg_current_a': float(self.leg_current_a),
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
        if can_rates is not None:
            status.update(can_rates)
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
        /status topic and /control_result topic.

        Other nodes (e.g. ref_generator_node in stewart_vision) use this
        same /control_cmd topic with a different `key:value` protocol
        (`mode:LEVEL_HOLD`, `ball_config:{...}`). Those aren't JSON for
        us — silently ignore rather than spamming the journal at 5 Hz.
        """
        text = (msg.data or '').strip()
        if not text or not text.startswith('{'):
            # Not our protocol; let the other nodes handle it.
            return
        try:
            d = json.loads(text)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"control_cmd: bad JSON: {e}")
            return
        cmd = d.get('cmd', '')
        ok, reply_msg = False, f"unknown cmd '{cmd}'"
        extra = {}  # extra fields to include in the response JSON
        try:
            if cmd == 'activate':
                ok, reply_msg = self._arm_all_in_pos_mode()
            elif cmd == 'get_limits':
                # Return whatever's currently in self.limits PLUS the file
                # path + mtime, so the test GUI can verify the controller
                # is using the latest leg_limits.yaml (and not some stale
                # cached version).
                ok, reply_msg = True, "limits dumped"
                extra['limits_in_memory'] = (
                    {f'axis_{n}': {
                        'lo': float(self.limits[n]['lo']),
                        'hi': float(self.limits[n]['hi']),
                        'rest': float(self.limits[n]['rest']),
                    } for n in range(6)}
                    if self.limits is not None else None)
                extra['limits_file_path'] = LEG_LIMITS_PATH
                try:
                    st = os.stat(LEG_LIMITS_PATH)
                    extra['limits_file_size'] = st.st_size
                    extra['limits_file_mtime_unix'] = st.st_mtime
                except OSError:
                    extra['limits_file_size'] = None
                    extra['limits_file_mtime_unix'] = None
                # Also dump the raw file contents so the GUI can show
                # the file vs in-memory side by side.
                try:
                    with open(LEG_LIMITS_PATH, 'r') as f:
                        extra['limits_file_text'] = f.read()
                except OSError as e:
                    extra['limits_file_text'] = f"(read failed: {e})"
            elif cmd == 'reload_limits':
                # Force re-read of leg_limits.yaml from disk and refresh
                # the feeder's pos_targets to the new neutral-pose rests.
                # User's hypothesis 2026-04-26: an old cached limits is
                # being used somewhere — this gives them a way to KNOW
                # the file on disk is what's in memory.
                try:
                    self.limits = _load_leg_limits()
                except Exception as e:
                    ok, reply_msg = False, f"reload failed: {e}"
                else:
                    if self.limits is None:
                        ok, reply_msg = False, (
                            "leg_limits.yaml exists but failed validation "
                            "(missing axis or bad values).")
                    else:
                        # Refresh feeder targets to new neutral rest.
                        try:
                            if self.feeder is not None and self.geom is not None:
                                neutral_targets, _c = _compute_motor_targets(
                                    (0, 0, 0), (0, 0, 0),
                                    self.geom, self.limits)
                                self.feeder.set_pos_targets(neutral_targets)
                        except Exception as e:
                            self.get_logger().warn(
                                f"feeder refresh after reload failed: {e}")
                        # Clear captured rest (limits supersedes).
                        self.captured_rest_positions = None
                        ok, reply_msg = True, (
                            f"reloaded leg_limits.yaml from "
                            f"{LEG_LIMITS_PATH}")
            elif cmd == 'set_leg_target':
                # Drive a single leg to an ABSOLUTE position (turns) using
                # the existing per-leg jog flow internally. Soft limits +
                # the feeder-level safety hook still apply.
                # Payload: {"leg": N, "target_turns": X}
                try:
                    n = int(d.get('leg', -1))
                    target = float(d.get('target_turns'))
                except Exception:
                    ok, reply_msg = False, "bad set_leg_target payload"
                else:
                    if not 0 <= n <= 5:
                        ok, reply_msg = False, f"leg {n} out of range"
                    elif self.listener is None:
                        ok, reply_msg = False, "listener not running"
                    else:
                        enc = self.listener.get_all(max_age_s=1.0)
                        if enc[n] is None:
                            ok, reply_msg = False, (
                                f"leg {n}: no fresh encoder reading")
                        else:
                            delta = target - float(enc[n])
                            ok, reply_msg = self._do_jog_leg(n, delta)
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
            elif cmd == 'level_record_start':
                ok, reply_msg, ext = self._lr_start_bag(
                    name=str(d.get('name') or 'untitled'),
                    notes=str(d.get('notes') or ''))
                extra.update(ext)
            elif cmd == 'level_record_stop':
                ok, reply_msg, ext = self._lr_stop_bag()
                extra.update(ext)
            elif cmd == 'level_zero_integrator':
                self._level_zero_integ_request.set()
                ok, reply_msg = True, "integrator zero requested"
            elif cmd == 'level_step_test':
                ok, reply_msg = self._lr_start_step_test_async(
                    amp_deg=float(d.get('amp_deg', 1.0)),
                    hold_s=float(d.get('hold_s', 3.0)),
                    count=int(d.get('count', 1)))
            elif cmd == 'level_auto_sweep_start':
                ok, reply_msg, ext = self._lr_start_sweep(d)
                extra.update(ext)
            elif cmd == 'level_auto_sweep_abort':
                ok, reply_msg = self._lr_abort_sweep()
            elif cmd == 'level_reload_gains':
                self.level_gains = _load_level_gains()
                extra['gains'] = dict(self.level_gains)
                ok, reply_msg = True, f"reloaded {LEVEL_GAINS_PATH}"
            elif cmd == 'level_get_gains':
                extra['gains'] = dict(self.level_gains)
                extra['gains_file_path'] = LEVEL_GAINS_PATH
                ok, reply_msg = True, "gains dumped"
            elif cmd == 'sysid_start':
                ok, reply_msg = self._do_sysid_start(d)
            elif cmd == 'sysid_abort':
                ok, reply_msg = self._do_sysid_abort()
            elif cmd == 'reload_empirical_ik':
                ok, reply_msg = self._do_reload_empirical_ik()
                if self.empirical_ik is not None and \
                        self.empirical_ik.is_loaded():
                    extra['empirical_ik_path'] = self.empirical_ik.path
                    extra['empirical_ik_z_list'] = list(
                        self.empirical_ik.z_list)
            elif cmd == 'level_save_gains':
                # Edit gains directly from the GUI: validate, write the
                # YAML, reload in-memory. Body is `gains: {<key>: <val>, ...}`
                # with any subset of the known knobs. Unknown keys are
                # rejected so a typo can't silently drop a gain.
                ok, reply_msg = self._do_level_save_gains(d.get('gains', {}))
                extra['gains'] = dict(self.level_gains)
                extra['gains_file_path'] = LEVEL_GAINS_PATH
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
    # Per-step jog cap — defense against absurdly large delta values
    # (typos, the input field accidentally getting set to 100, etc).
    # Soft limits in leg_limits.yaml already clamp the actual final
    # position, so a slightly oversize jog is harmless — it just
    # gets clipped at lo/hi. 5.0 turns ≈ 355 mm, which is well past
    # the entire stroke of any reasonable Stewart leg, so anything
    # larger really is a misuse.
    MAX_JOG_DELTA_TURNS = 5.0

    def _do_jog_leg(self, n, delta_turns):
        """Position-mode jog on one leg.

        Seeds the new target from the leg's CURRENT encoder reading
        (not the stale pos_target) plus the requested delta. This is
        safer than the previous behavior, which added delta to whatever
        was last commanded — if a previous command was uncompleted, that
        baseline could be far from where the leg actually is.

        Auto-switches the leg into POS mode if it isn't already.
        """
        if self.limits is None:
            return False, "no leg_limits.yaml"
        if not 0 <= n <= 5:
            return False, f"leg {n} out of range"
        if self.feeder is None:
            return False, "feeder not running"
        if self.listener is None:
            return False, "encoder listener not running"
        delta = float(delta_turns)
        # Per-step magnitude cap.
        if abs(delta) > self.MAX_JOG_DELTA_TURNS:
            return False, (
                f"jog delta {delta:+.3f} turns exceeds per-step cap "
                f"{self.MAX_JOG_DELTA_TURNS:.2f} turns "
                f"(~{self.MAX_JOG_DELTA_TURNS * MM_PER_REV:.0f} mm). "
                f"Click the smaller-step button or jog repeatedly.")
        # Use current encoder as the baseline, not pos_target.
        enc = self.listener.get_all(max_age_s=1.0)
        if enc[n] is None:
            return False, (
                f"jog refused: leg {n} has no fresh encoder reading. "
                f"Power-cycle the ODrive(s) and retry.")
        modes = self.feeder.get_modes()
        if modes[n] != 'pos':
            ok, msg = self._arm_leg_internal(n, 'pos')
            if not ok:
                return False, f"auto-arm POS failed: {msg}"
        new_target = float(enc[n]) + delta
        lo, hi = self.limits[n]['lo'], self.limits[n]['hi']
        clamped = False
        if new_target < lo:
            new_target = lo
            clamped = True
        elif new_target > hi:
            new_target = hi
            clamped = True
        cur = self.feeder.get_pos_targets()
        cur[n] = new_target
        self.feeder.set_pos_targets(cur)
        return True, (f"clamped at {new_target:+.4f}" if clamped
                      else f"L{n} → {new_target:+.4f}")

    def _load_empirical_ik(self):
        """Find the most recent system_id_*/jacobian.json under
        tuning_data/ and load it. Returns None if none found.
        Failure to parse a file logs a warning but doesn't crash —
        the node falls back to geometric IK."""
        repo_tuning = os.path.join(_BRINGUP_DIR, '..', 'tuning_data')
        repo_tuning = os.path.abspath(repo_tuning)
        if not os.path.isdir(repo_tuning):
            return None
        candidates = []
        for name in os.listdir(repo_tuning):
            if not name.startswith('system_id_'):
                continue
            jp = os.path.join(repo_tuning, name, 'jacobian.json')
            if os.path.isfile(jp):
                candidates.append((name, jp))
        if not candidates:
            return None
        # Newest by name (timestamps are lexicographically sortable).
        candidates.sort(key=lambda c: c[0])
        _name, latest = candidates[-1]
        try:
            return EmpiricalIK(latest)
        except Exception as e:
            self.get_logger().warn(f"empirical IK load failed: {e}")
            return None

    def _do_reload_empirical_ik(self):
        """Re-scan tuning_data/ and reload the empirical IK. Useful
        after committing a new system-id run without restarting the
        node."""
        prev = self.empirical_ik
        self.empirical_ik = self._load_empirical_ik()
        if self.empirical_ik is None or not self.empirical_ik.is_loaded():
            return False, "no jacobian.json found under tuning_data/"
        msg = (f"loaded {self.empirical_ik.path} "
               f"({len(self.empirical_ik.z_list)} Z heights)")
        if prev is None:
            return True, "empirical IK enabled: " + msg
        return True, "empirical IK reloaded: " + msg

    def _do_set_pose(self, x, y, z, r, p, yw, allow_large=False):
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
        # _compute_motor_targets already clamps to per-leg [lo, hi] from
        # leg_limits.yaml. Trust the soft limits — no per-step delta cap.
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
            'control_period_s': self.ctrl_period_s,
            'level_loop_hz': self.level_loop_hz,
            'level_gains_ref_hz': LEVEL_REF_HZ,
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

    # Range-validation table for level gains. Each entry is
    # (lower_bound, upper_bound). Values outside the range are clamped
    # *and* a warning goes to the reply message. Bounds are deliberately
    # generous — the GUI's input fields enforce sensible ranges; this
    # is the last-line defense against catastrophic typos
    # (e.g. KP=10000 from a slipped decimal).
    _LEVEL_GAIN_BOUNDS = {
        'kp':                            (0.0, 5.0),
        'ki':                            (0.0, 2.0),
        'deadband_deg':                  (0.0, 1.0),
        'rate_limit_deg_per_iter':       (0.001, 5.0),
        'max_corr_deg':                  (0.1, 15.0),
        'filter_alpha':                  (0.0, 1.0),
        'integ_decay_outer_deg':         (0.0, 5.0),
        'integ_decay_per_tick_at_50hz':  (0.0, 1.0),
        # 0 = use geometric IK (legacy). 1 = use empirical IK if loaded.
        'use_empirical_ik':              (0.0, 1.0),
    }

    def _do_level_save_gains(self, gains_dict):
        """Save a partial gains dict into level_gains.yaml + reload.
        Body shape: {'kp': 1.0, 'ki': 0.3, ...}. Only known keys
        accepted; unknown keys → error (typo guard). Validated against
        _LEVEL_GAIN_BOUNDS before write."""
        if not isinstance(gains_dict, dict) or not gains_dict:
            return False, "gains: empty or not a dict"
        unknown = [k for k in gains_dict if k not in self._LEVEL_GAIN_BOUNDS]
        if unknown:
            return False, f"unknown gain keys: {unknown}. Known: {list(self._LEVEL_GAIN_BOUNDS.keys())}"
        clamped_msgs = []
        clean = {}
        for k, v in gains_dict.items():
            try:
                v = float(v)
            except (TypeError, ValueError):
                return False, f"gain '{k}' is not a number: {v!r}"
            lo, hi = self._LEVEL_GAIN_BOUNDS[k]
            if v < lo or v > hi:
                v_clamped = max(lo, min(v, hi))
                clamped_msgs.append(f"{k}={v} clamped to {v_clamped} (range [{lo}, {hi}])")
                v = v_clamped
            clean[k] = v
        # Merge into existing on-disk gains so we preserve any keys not
        # being edited (e.g. if a future field is added but the GUI
        # doesn't know about it yet).
        try:
            on_disk = {}
            if os.path.isfile(LEVEL_GAINS_PATH):
                with open(LEVEL_GAINS_PATH) as f:
                    on_disk = yaml.safe_load(f) or {}
            on_disk.update(clean)
            os.makedirs(os.path.dirname(LEVEL_GAINS_PATH), exist_ok=True)
            tmp = LEVEL_GAINS_PATH + '.tmp'
            with open(tmp, 'w') as f:
                yaml.safe_dump(on_disk, f, sort_keys=True,
                               default_flow_style=False)
            os.replace(tmp, LEVEL_GAINS_PATH)
        except Exception as e:
            return False, f"failed to write {LEVEL_GAINS_PATH}: {e}"
        # Reload in-memory so the level loop picks up the new values
        # within one tick.
        try:
            self.level_gains = _load_level_gains()
        except Exception as e:
            return True, (f"wrote YAML but reload failed: {e}; "
                          f"call level_reload_gains manually")
        msg = f"saved {len(clean)} gain(s) to {os.path.basename(LEVEL_GAINS_PATH)}"
        if clamped_msgs:
            msg += "; " + "; ".join(clamped_msgs)
        return True, msg

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

    def _prepare_for_level(self):
        # Defensive runtime self-heal before the level loop starts streaming
        # Set_Input_Pos. We've been bitten twice by the drives' runtime
        # control_mode being VELOCITY at this point — once because flash had
        # been left at VELOCITY by the WebGUI, and once when a leg was armed
        # in vel mode by mistake. Either way, Set_Input_Pos gets silently
        # ignored, the watchdog times out, the leg disarms.
        #
        # The standard ODrive CAN command 0x00B writes runtime control_mode
        # and input_mode immediately, no save / reboot required. Belt-and-
        # suspenders: also force the feeder into 'pos' mode and re-seed
        # pos_targets from the latest encoder reading so we don't slam
        # toward a stale target on the first tick.
        if self.bus is None or self.feeder is None or self.listener is None:
            return False, "bus/feeder/listener not running"
        enc = self.listener.get_all(max_age_s=1.0)
        stale = [i for i, e in enumerate(enc) if e is None]
        if stale:
            return False, f"stale encoder reading on legs {stale} — refusing to seed"
        # Phase 1: standard CAN cmd 0x00B (control_mode + input_mode)
        # under bus_lock. Mode/input change is runtime-only — flash
        # unaffected. Also seed feeder's per-leg target from the latest
        # encoder so we don't slam toward a stale target on first tick.
        with self.bus_lock:
            for n in range(6):
                try:
                    _send_cmd(self.bus, n, CMD_SET_CONTROLLER_MODE,
                              struct.pack('<II', CONTROL_MODE_POSITION,
                                          INPUT_MODE_PASSTHROUGH))
                except Exception as e:
                    return False, f"set_controller_mode leg {n} failed: {e}"
                self.feeder.set_pos_target_one(n, float(enc[n]))
                self.feeder.set_mode(n, 'pos')
        # Phase 2: SDO write of wL_FF_enable=True. Belt-and-suspenders
        # against the recurring "FF lost on flash reset / WebGUI reconfig"
        # issue. Endpoint id verified against fw 0.6.11-1's
        # flat_endpoints.json (see odrive_endpoints_reference.md). Each
        # write_sdo acquires bus_lock internally — must NOT be inside
        # the lock above or it deadlocks.
        wlff_failures = []
        for n in range(6):
            if not self.listener.write_sdo(
                    self.bus_lock, n, 305, True, 'bool'):
                wlff_failures.append(n)
        time.sleep(0.05)
        msg = "all 6 legs prepped (pos+passthrough+wL_FF=True, seeded from encoders)"
        if wlff_failures:
            msg += f"  WARN wL_FF write failed on legs {wlff_failures}"
        return True, msg

    def _do_enable_level(self, want):
        if want:
            if not self.armed:
                return False, "arm first"
            cal = _load_level_cal()
            if cal is None:
                return False, f"no {LEVEL_CAL_PATH} — capture level first"
            ok, prep_msg = self._prepare_for_level()
            if not ok:
                return False, f"level prep failed: {prep_msg}"
            self.level_ref_roll, self.level_ref_pitch = cal
            self._start_level_loop()
            return True, (f"level enabled "
                          f"(ref roll={self.level_ref_roll:+.3f} "
                          f"pitch={self.level_ref_pitch:+.3f}; "
                          f"{prep_msg})")
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
            ok, prep_msg = self._prepare_for_level()
            if not ok:
                res.success = False
                res.message = f"level prep failed: {prep_msg}"
                return res
            self.level_ref_roll, self.level_ref_pitch = cal
            self._start_level_loop()
            res.success = True
            res.message = (f"level enabled (ref roll={self.level_ref_roll:+.3f}"
                           f" pitch={self.level_ref_pitch:+.3f}; {prep_msg})")
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
        # Level-loop body. Gains are read from self.level_gains at the
        # top of every tick so /control_cmd level_reload_gains takes
        # effect within one period without restarting the loop. Step
        # offsets (self.level_step_offset_*) and the zero-integrator
        # request flag are also read fresh per tick to keep the
        # auto-sweep / step-test routines decoupled from this loop.
        # See docs/level_pi_tuning_plan.md for the diagnostic spec.
        err_r_f = 0.0
        err_p_f = 0.0
        integ_r = 0.0
        integ_p = 0.0
        tilt_r_corr = 0.0
        tilt_p_corr = 0.0
        nan6 = [float('nan')] * 6
        # Pre-compute the rate-scaling factor between the gain-tuning
        # reference rate (LEVEL_REF_HZ) and the loop's actual rate. The
        # YAML's `rate_limit_deg_per_iter` and `filter_alpha` are
        # interpreted as "calibrated at LEVEL_REF_HZ"; we map them to
        # their per-iter equivalents at the current rate so the closed-
        # loop frequency response stays the same regardless of dt.
        ref_dt = 1.0 / LEVEL_REF_HZ
        dt = self.ctrl_period_s
        # rate_limit: ° per iteration → preserve the °/s slew limit.
        rate_scale = dt / ref_dt
        while not self.level_stop.is_set():
            t0 = time.monotonic()
            # Snapshot gains for this tick (allows live reload).
            g = self.level_gains
            kp = g['kp']
            ki = g['ki']
            alpha_ref = g['filter_alpha']
            deadband = g['deadband_deg']
            rate_limit_ref = g['rate_limit_deg_per_iter']
            max_corr = g['max_corr_deg']
            # Scale per-iter values to this loop's dt. Slew is linear:
            #   °/iter at dt = (°/iter at ref_dt) × (dt / ref_dt).
            # IIR alpha keeps the same time constant tau:
            #   tau = -ref_dt / log(1 - alpha_ref)
            #   alpha_at_dt = 1 - exp(-dt / tau)  =  1 - (1 - alpha_ref) ** (dt/ref_dt)
            rate_limit = rate_limit_ref * rate_scale
            alpha_clip = min(max(alpha_ref, 1e-6), 0.999999)
            alpha = 1.0 - (1.0 - alpha_clip) ** rate_scale
            # Integrator decay (when in deadband) scaled the same way:
            # decay_at_dt = decay_at_ref ** (dt / ref_dt) preserves the
            # exponential time constant.
            decay_ref = g.get('integ_decay_per_tick_at_50hz', 0.99)
            decay_clip = min(max(decay_ref, 0.0), 1.0)
            integ_decay = decay_clip ** rate_scale
            if self._level_zero_integ_request.is_set():
                integ_r = 0.0
                integ_p = 0.0
                self._level_zero_integ_request.clear()
            with self.imu_lock:
                rpy = self.imu_rpy
                last_rx = self.imu_last_rx
            if rpy is None or (t0 - last_rx) > 0.3:
                time.sleep(dt)
                continue
            # DYNAMIC reference: target IMU reading follows the commanded
            # pose (current_rpy) plus the tuning step-offset. When
            # current_rpy == 0 and the step-offset == 0, this collapses
            # back to static ref = level_ref.
            step_or = self.level_step_offset_roll
            step_op = self.level_step_offset_pitch
            target_r = self.level_ref_roll  + self.current_rpy[0] + step_or
            target_p = self.level_ref_pitch + self.current_rpy[1] + step_op
            err_r = rpy[0] - target_r
            err_p = rpy[1] - target_p
            err_r_f = alpha * err_r + (1 - alpha) * err_r_f
            err_p_f = alpha * err_p + (1 - alpha) * err_p_f
            clip_flags = 0
            # Graduated integrator update:
            #   |err| > outer  → full integration, no decay (decay_w=1, scale=1)
            #   |err| < deadband → no integration, full decay (decay_w=0, scale=0)
            #   between: linear blend, so wind-up bleeds off while error
            #            is still small but non-zero.
            outer_r = max(deadband + 1e-6,
                          g.get('integ_decay_outer_deg', 0.10))
            for axis_n, err_f in (('r', err_r_f), ('p', err_p_f)):
                ae = abs(err_f)
                if ae >= outer_r:
                    blend = 1.0
                elif ae <= deadband:
                    blend = 0.0
                    clip_flags |= (1 << (4 if axis_n == 'r' else 5))
                else:
                    blend = (ae - deadband) / (outer_r - deadband)
                # decay_weight: 1 at outer (no decay), integ_decay at deadband
                decay_weight = integ_decay + blend * (1.0 - integ_decay)
                if axis_n == 'r':
                    integ_r = integ_r * decay_weight + (-err_f * ki * dt) * blend
                else:
                    integ_p = integ_p * decay_weight + (-err_f * ki * dt) * blend
            if integ_r > max_corr or integ_r < -max_corr:
                clip_flags |= (1 << 6)   # integ_clamp_r
            integ_r = max(-max_corr, min(max_corr, integ_r))
            if integ_p > max_corr or integ_p < -max_corr:
                clip_flags |= (1 << 7)   # integ_clamp_p
            integ_p = max(-max_corr, min(max_corr, integ_p))
            # Unclipped PI output — captured for diagnostics. When this
            # diverges from the post-clip corr, the controller is fighting
            # saturation, which is exactly what the analyzer flags.
            pi_out_r = -err_r_f * kp + integ_r
            pi_out_p = -err_p_f * kp + integ_p
            d_r_raw = pi_out_r - tilt_r_corr
            d_p_raw = pi_out_p - tilt_p_corr
            d_r = max(-rate_limit, min(rate_limit, d_r_raw))
            d_p = max(-rate_limit, min(rate_limit, d_p_raw))
            if d_r != d_r_raw:
                clip_flags |= (1 << 0)   # rate_limit_r
            if d_p != d_p_raw:
                clip_flags |= (1 << 1)   # rate_limit_p
            new_r_raw = tilt_r_corr + d_r
            new_p_raw = tilt_p_corr + d_p
            new_r = max(-max_corr, min(max_corr, new_r_raw))
            new_p = max(-max_corr, min(max_corr, new_p_raw))
            if new_r != new_r_raw:
                clip_flags |= (1 << 2)   # max_corr_r
            if new_p != new_p_raw:
                clip_flags |= (1 << 3)   # max_corr_p
            # Back-calculation anti-windup: if the actual (rate-limited +
            # clamped) command differs from the unsaturated PI output,
            # pull the integrator back by that deficit so it stops winding
            # up against the saturation. Without this, the I term keeps
            # growing during transients and produces a limit cycle around
            # the target instead of settling.
            integ_r += (new_r - pi_out_r)
            integ_p += (new_p - pi_out_p)
            integ_r = max(-max_corr, min(max_corr, integ_r))
            integ_p = max(-max_corr, min(max_corr, integ_p))
            tilt_r_corr = new_r
            tilt_p_corr = new_p
            self.level_corr = [tilt_r_corr, tilt_p_corr]
            xyz = self.current_xyz
            rpy_cmd = [self.current_rpy[0] + tilt_r_corr,
                       self.current_rpy[1] + tilt_p_corr,
                       self.current_rpy[2]]
            motor_targets = list(nan6)
            if self.limits is not None and self.feeder is not None:
                # Pick IK: empirical (if loaded AND gain enabled) or geometric.
                use_emp = bool(g.get('use_empirical_ik', 0)) and \
                    self.empirical_ik is not None and \
                    self.empirical_ik.is_loaded()
                ik = self.empirical_ik if use_emp else None
                targets, _ = _compute_motor_targets(
                    tuple(xyz), tuple(rpy_cmd), self.geom, self.limits,
                    empirical_ik=ik)
                self.feeder.set_pos_targets(targets)
                motor_targets = [float(v) for v in targets]
            # Diag publish — outside the IK guard so we still get data
            # when the feeder isn't running (e.g., during stand-down tests).
            try:
                msg = LevelDiag()
                msg.t_imu = float(last_rx)
                msg.t_tick = float(t0)
                msg.roll = float(rpy[0])
                msg.pitch = float(rpy[1])
                msg.yaw = float(rpy[2]) if len(rpy) > 2 else float('nan')
                msg.target_roll = float(target_r)
                msg.target_pitch = float(target_p)
                msg.err_r = float(err_r)
                msg.err_p = float(err_p)
                msg.err_r_filt = float(err_r_f)
                msg.err_p_filt = float(err_p_f)
                msg.integ_r = float(integ_r)
                msg.integ_p = float(integ_p)
                msg.pi_out_r = float(pi_out_r)
                msg.pi_out_p = float(pi_out_p)
                msg.corr_r = float(tilt_r_corr)
                msg.corr_p = float(tilt_p_corr)
                msg.clip_flags = int(clip_flags)
                msg.motor_targets = motor_targets
                if self.listener is not None:
                    enc = self.listener.get_all(max_age_s=0.5)
                    msg.leg_enc = [float(v) if v is not None else float('nan')
                                   for v in enc]
                    msg.leg_vel = [float(v) for v in self.listener.get_vel(max_age_s=0.5)]
                    msg.leg_iq = [float(v) for v in self.listener.get_iq(max_age_s=2.0)]
                    msg.leg_iq_sp = [float(v) for v in
                                     self.listener.get_iq_setpoint(max_age_s=2.0)]
                    msg.axis_state = [int(s) for s in
                                      self.listener.get_states(max_age_s=2.0)]
                    err_pairs = self.listener.get_errors(max_age_s=3.0)
                    # 0xFFFFFFFF = "no fresh error frame this tick"; the
                    # analyzer treats this as missing rather than a real
                    # all-bits-set error code.
                    msg.active_errors = [
                        int(p[0]) if p is not None else 0xFFFFFFFF
                        for p in err_pairs
                    ]
                else:
                    msg.leg_enc = list(nan6)
                    msg.leg_vel = list(nan6)
                    msg.leg_iq = list(nan6)
                    msg.leg_iq_sp = list(nan6)
                    msg.axis_state = [0] * 6
                    msg.active_errors = [0xFFFFFFFF] * 6
                # Feeder mode per leg ('idle'/'pos'/'vel' → 0/1/2; 255 if
                # the feeder isn't running). Confirms _prepare_for_level
                # actually put us in pos mode and that no leg has been
                # silently switched mid-run.
                if self.feeder is not None:
                    _MMAP = {'idle': 0, 'pos': 1, 'vel': 2}
                    msg.feeder_mode = [_MMAP.get(m, 255)
                                       for m in self.feeder.get_modes()]
                else:
                    msg.feeder_mode = [255] * 6
                msg.dt_actual = float(time.monotonic() - t0)
                msg.target_xyzrpy = [
                    float(xyz[0]), float(xyz[1]), float(xyz[2]),
                    float(rpy_cmd[0]), float(rpy_cmd[1]), float(rpy_cmd[2]),
                ]
                self.pub_level_diag.publish(msg)
            except Exception:
                # Never let diag publication crash the control loop.
                pass
            sl = dt - (time.monotonic() - t0)
            if sl > 0:
                time.sleep(sl)

    # ---- Level-PI tuning: bag recording + step tests + auto-sweep ----
    # See docs/level_pi_tuning_plan.md. Inbound commands arrive via
    # /control_cmd JSON ('level_record_*' / 'level_step_test' /
    # 'level_auto_sweep_*'). State for the GUI is published as JSON on
    # /level_record_state at 5 Hz.

    LR_BAG_TOPICS = (
        '/level_diag', '/platform_rpy', '/leg_encoders',
        '/leg_currents', '/control_cmd',
    )
    # Tolerances for the Z-reached gate. POS_TOL is generous (~7 mm leg
    # displacement) because the live level loop drives small tilt
    # corrections that keep the encoders moving even when the platform
    # has visibly arrived at Z. STAB_TOL is the encoder-noise/limit-
    # cycle envelope we accept as "stable" over the window.
    LR_Z_REACHED_POS_TOL_TURNS = 0.10
    LR_Z_REACHED_STAB_TOL_TURNS = 0.10
    LR_Z_REACHED_VEL_WINDOW_S = 0.5
    LR_Z_REACHED_TIMEOUT_S = 15.0
    LR_STEP_AMP_HARD_CAP_DEG = 2.0

    def _lr_git_sha(self):
        try:
            r = subprocess.run(
                ['git', '-C', _BRINGUP_DIR, 'rev-parse', '--short', 'HEAD'],
                capture_output=True, text=True, timeout=2.0)
            if r.returncode == 0:
                return r.stdout.strip()
        except Exception:
            pass
        return 'unknown'

    def _lr_safe_name(self, s):
        """Sanitize a user-supplied test name into a filename component."""
        s = (s or 'untitled').strip()
        out = ''.join(c if c.isalnum() or c in '-_' else '_' for c in s)
        return out[:64] or 'untitled'

    def _lr_set_state(self, state, **kw):
        """Update the sweep state snapshot (used by the heartbeat)."""
        with self._sweep_state_lock:
            self._sweep_state = {'state': state, 't_unix': time.time(), **kw}

    def _tick_level_record_state(self):
        """5 Hz heartbeat for the GUI. Publishes the latest state snapshot
        plus a quick check of whether a bag is currently recording."""
        with self._sweep_state_lock:
            snap = dict(self._sweep_state)
        bag_alive = False
        with self._bag_lock:
            if self._bag_proc is not None and self._bag_proc.poll() is None:
                bag_alive = True
                snap['single_bag_dir'] = self._bag_dir
                snap['single_bag_elapsed_s'] = round(
                    time.monotonic() - self._bag_t0, 1)
        snap['bag_recording'] = bag_alive
        snap['gains'] = dict(self.level_gains)
        snap['level_enabled'] = bool(self.level_enabled)
        m = String()
        m.data = json.dumps(snap)
        self.pub_level_record_state.publish(m)

    # ---- inner-loop config snapshot ----
    # Read once per drive at bag start so the sidecar carries enough
    # context to spot regressions without bisecting commits. List is
    # hand-picked from odrive_endpoints_reference.md — paths + IDs +
    # types verified against fw 0.6.11-1's flat_endpoints.json.
    LR_SNAPSHOT_ENDPOINTS = [
        # path                                              id    type
        ('axis0.config.motor.wL_FF_enable',                 305, 'bool'),
        ('axis0.controller.config.vel_integrator_gain',     382, 'float32'),
        ('axis0.controller.config.vel_integrator_limit',    383, 'float32'),
        ('axis0.controller.config.pos_gain',                380, 'float32'),
        ('axis0.controller.config.vel_gain',                381, 'float32'),
        ('axis0.controller.config.vel_limit',               384, 'float32'),
        ('axis0.controller.config.vel_limit_tolerance',     385, 'float32'),
        ('axis0.controller.config.control_mode',            378, 'uint8'),
        ('axis0.controller.config.input_mode',              379, 'uint8'),
        ('axis0.config.motor.current_soft_max',             315, 'float32'),
        ('axis0.config.motor.current_hard_max',             316, 'float32'),
        ('axis0.config.motor.torque_constant',              302, 'float32'),
        ('axis0.config.watchdog_timeout',                   243, 'float32'),
        ('axis0.config.enable_watchdog',                    244, 'bool'),
        ('axis0.config.can.node_id',                        272, 'uint32'),
        ('axis0.config.can.heartbeat_msg_rate_ms',          274, 'uint32'),
        ('axis0.config.can.encoder_msg_rate_ms',            275, 'uint32'),
        ('axis0.config.can.error_msg_rate_ms',              277, 'uint32'),
        ('axis0.config.can.iq_msg_rate_ms',                 276, 'uint32'),
    ]

    def _lr_capture_inner_loop_config(self, timeout_per_read=0.4):
        """Read every endpoint in LR_SNAPSHOT_ENDPOINTS from each of the
        6 drives via SDO. Returns {node_id: {path: value}}; missing
        reads (timeout / drive offline) come back as None.

        Cost: ~19 endpoints × 6 drives ≈ 114 reads. With <1ms typical
        SDO round-trip on socketcan + drive, total ≈ 200-600ms. Called
        only at bag start; not in any tick path."""
        if self.bus is None or self.listener is None:
            return None
        out = {}
        for n in range(6):
            per_drive = {}
            for path, ep_id, kind in self.LR_SNAPSHOT_ENDPOINTS:
                try:
                    val = self.listener.read_sdo(
                        self.bus_lock, n, ep_id, kind,
                        timeout=timeout_per_read)
                except Exception:
                    val = None
                per_drive[path] = val
            out[n] = per_drive
        return out

    # ---- single bag ----
    @staticmethod
    def _scrub_non_finite(obj):
        """Replace NaN / +Inf / -Inf with None recursively. JSON spec
        rejects non-finite floats; some SDO reads (e.g.
        vel_integrator_limit defaults to +inf) trigger this naturally,
        and the GUI's /bags handler chokes on the resulting sidecar.
        Sanitize on write so neither side has to deal with it."""
        if isinstance(obj, float):
            return obj if math.isfinite(obj) else None
        if isinstance(obj, dict):
            return {k: StewartControlNode._scrub_non_finite(v)
                    for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [StewartControlNode._scrub_non_finite(v) for v in obj]
        return obj

    def _lr_save_notes(self, bag_dir, name, notes, extra_meta):
        """Sidecar notes.json next to the bag dir. Captures gains snapshot,
        free-text notes, git SHA — the per-trial 'why' that the bag itself
        can't carry."""
        try:
            payload = {
                'name': name,
                'notes': notes,
                'gains': dict(self.level_gains),
                'gains_file_path': LEVEL_GAINS_PATH,
                'git_sha': self._lr_git_sha(),
                'started_utc': datetime.datetime.utcnow().isoformat() + 'Z',
                'topics': list(self.LR_BAG_TOPICS),
                **(extra_meta or {}),
            }
            payload = self._scrub_non_finite(payload)
            with open(os.path.join(bag_dir + '_notes.json'), 'w') as f:
                json.dump(payload, f, indent=2, allow_nan=False)
        except Exception as e:
            self.get_logger().warn(f"sidecar notes write failed: {e}")

    def _lr_spawn_bag(self, bag_dir):
        """Start a `ros2 bag record` subprocess writing to bag_dir.
        Returns the Popen. Re-source ROS in the child so this works
        whether or not the parent process has it on PATH."""
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        topics = ' '.join(self.LR_BAG_TOPICS)
        cmd = (
            'source /opt/ros/kilted/setup.bash && '
            'source ~/ros2_ws/install/local_setup.bash && '
            f'exec ros2 bag record -o {bag_dir} {topics}'
        )
        return subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)

    def _lr_stop_proc(self, proc, timeout_s=5.0):
        """SIGINT a bag-record subprocess and wait. Returns True on clean
        stop, False on hard kill."""
        if proc is None or proc.poll() is not None:
            return True
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=timeout_s)
                return True
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=2.0)
                return False
        except Exception:
            return False

    def _lr_start_bag(self, name, notes, extra_meta=None):
        # Capture inner-loop config BEFORE spawning the bag so the
        # sidecar always has it, even if bag spawn fails. Slow path
        # (~200-600ms of SDO reads) but only at bag start.
        inner_cfg = self._lr_capture_inner_loop_config()
        with self._bag_lock:
            if self._bag_proc is not None and self._bag_proc.poll() is None:
                return False, "bag already recording", {}
            stamp = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
            safe = self._lr_safe_name(name)
            bag_dir = os.path.join(BAGS_DIR, f"{stamp}_{safe}")
            try:
                self._bag_proc = self._lr_spawn_bag(bag_dir)
            except Exception as e:
                return False, f"spawn failed: {e}", {}
            self._bag_dir = bag_dir
            self._bag_t0 = time.monotonic()
        meta_with_snapshot = dict(extra_meta or {})
        if inner_cfg is not None:
            meta_with_snapshot['inner_loop_config'] = inner_cfg
        meta_with_snapshot['level_loop_hz'] = self.level_loop_hz
        meta_with_snapshot['ctrl_period_s'] = self.ctrl_period_s
        self._lr_save_notes(bag_dir, name, notes, meta_with_snapshot)
        return True, f"recording -> {bag_dir}", {'bag_dir': bag_dir}

    def _lr_stop_bag(self):
        with self._bag_lock:
            proc = self._bag_proc
            bag_dir = self._bag_dir
            t0 = self._bag_t0
            self._bag_proc = None
            self._bag_dir = None
        if proc is None:
            return False, "no bag recording", {}
        clean = self._lr_stop_proc(proc)
        size = 0
        try:
            for root, _dirs, files in os.walk(bag_dir):
                for fn in files:
                    size += os.path.getsize(os.path.join(root, fn))
        except Exception:
            pass
        duration = time.monotonic() - t0
        return True, "stopped", {
            'bag_dir': bag_dir,
            'duration_s': round(duration, 2),
            'size_bytes': int(size),
            'clean_stop': clean,
        }

    # ---- step test (used standalone and inside auto-sweep) ----
    def _lr_step_battery(self, amp_deg, hold_s, count, abort_event):
        """Drive self.level_step_offset_roll as a square wave:
        +amp / hold_s / 0 / hold_s, count times. Always restores
        offset to 0 on exit (including abort). Returns True if all
        cycles ran, False if aborted."""
        amp = max(-self.LR_STEP_AMP_HARD_CAP_DEG,
                  min(self.LR_STEP_AMP_HARD_CAP_DEG, float(amp_deg)))
        try:
            for k in range(int(count)):
                if abort_event is not None and abort_event.is_set():
                    return False
                self.level_step_offset_roll = amp
                end = time.monotonic() + hold_s
                while time.monotonic() < end:
                    if abort_event is not None and abort_event.is_set():
                        return False
                    time.sleep(0.02)
                self.level_step_offset_roll = 0.0
                end = time.monotonic() + hold_s
                while time.monotonic() < end:
                    if abort_event is not None and abort_event.is_set():
                        return False
                    time.sleep(0.02)
            return True
        finally:
            self.level_step_offset_roll = 0.0

    def _lr_start_step_test_async(self, amp_deg, hold_s, count):
        """Fire-and-forget step test. Reuses the sweep abort flag so the
        GUI's big red button stops everything."""
        if not self.level_enabled:
            return False, "enable level loop before step test"
        if self._sweep_thread is not None and self._sweep_thread.is_alive():
            return False, "auto-sweep is running; abort it first"
        # Use the sweep abort flag so a single 'abort' command cancels
        # whichever thing is running.
        self._sweep_stop.clear()
        def runner():
            self._lr_set_state('step_test_running', amp_deg=amp_deg,
                               hold_s=hold_s, count=count)
            self._lr_step_battery(amp_deg, hold_s, count, self._sweep_stop)
            self._lr_set_state('idle')
        threading.Thread(target=runner, daemon=True).start()
        return True, f"step test x{int(count)} amp {amp_deg}°"

    # ---- auto-sweep ----
    def _lr_validate_sweep(self, params):
        """Clamp + validate user-supplied sweep params. Returns
        (ok, msg, normalized_params)."""
        try:
            z_min = float(params.get('z_min'))
            z_max = float(params.get('z_max'))
            z_step = float(params.get('z_step'))
        except Exception:
            return False, "z_min/z_max/z_step required", None
        if z_step <= 0:
            return False, "z_step must be > 0", None
        if z_min > z_max:
            z_min, z_max = z_max, z_min
        if (z_max - z_min) / z_step > 50:
            return False, "more than 50 levels — increase z_step", None
        amp = float(params.get('step_amp_deg', 1.0))
        if abs(amp) > self.LR_STEP_AMP_HARD_CAP_DEG:
            return False, f"step_amp_deg > {self.LR_STEP_AMP_HARD_CAP_DEG}° hard cap", None
        if self.limits is None:
            return False, "leg_limits.yaml not loaded — home first", None
        # IK every Z; reject if any clamps. _compute_motor_targets sets
        # the second return to True when it had to clip into leg_limits.
        zs = []
        z = z_min
        while z <= z_max + 1e-6:
            try:
                _t, clamped = _compute_motor_targets(
                    (0, 0, z), (0, 0, 0), self.geom, self.limits)
            except Exception as e:
                return False, f"IK failed at Z={z}: {e}", None
            if clamped:
                return False, f"Z={z:.1f} mm exceeds leg limits", None
            zs.append(round(z, 2))
            z += z_step
        norm = {
            'z_values': zs,
            'step_count': max(1, int(params.get('step_count', 8))),
            'step_amp_deg': amp,
            'step_hold_s': float(params.get('step_hold_s', 3.0)),
            'baseline_s': float(params.get('baseline_s', 30.0)),
            'prefix': self._lr_safe_name(params.get('prefix') or 'sweep'),
        }
        return True, "ok", norm

    def _lr_start_sweep(self, params):
        if self._sweep_thread is not None and self._sweep_thread.is_alive():
            return False, "auto-sweep already running", {}
        if not self.level_enabled:
            return False, "enable level loop before auto-sweep", {}
        ok, msg, norm = self._lr_validate_sweep(params)
        if not ok:
            return False, msg, {}
        # Estimate wall-clock so the GUI can show it without re-running
        # the math. Per-Z time budget matches _lr_run_sweep below.
        per_z = (self.LR_Z_REACHED_TIMEOUT_S
                 + norm['baseline_s']
                 + 2 * norm['step_hold_s'] * norm['step_count'])
        norm['estimated_total_s'] = round(per_z * len(norm['z_values']), 1)
        self._sweep_stop.clear()
        self._sweep_thread = threading.Thread(
            target=self._lr_run_sweep, args=(norm,), daemon=True)
        self._sweep_thread.start()
        return True, f"sweep started, {len(norm['z_values'])} levels", {
            'normalized_params': norm,
        }

    def _lr_abort_sweep(self):
        # Set the abort flag (covers both standalone step tests and the
        # full sweep). Then SIGINT any per-Z bag still recording so we
        # don't leave a stuck child behind.
        self._sweep_stop.set()
        with self._bag_lock:
            proc = self._bag_proc
            self._bag_proc = None
            self._bag_dir = None
        if proc is not None:
            self._lr_stop_proc(proc)
        return True, "abort flagged"

    # ---------------- System-ID (empirical Jacobian) ----------------
    # Drive each leg in turn by ±δ from the rest pose at multiple Z
    # heights and multiple δ amplitudes; measure the resulting platform
    # attitude change. Builds an empirical 3×6 Jacobian (∂rpy/∂leg_n)
    # at each Z. Compared to the IK's theoretical Jacobian, this
    # exposes per-leg gain mismatches that explain the systematic
    # offset the level loop has been struggling with.
    #
    # Spec (2026-04-29 conversation):
    #  - level loop must be OFF during measurement (we want clean
    #    Δleg→Δrpy without the loop fighting us)
    #  - drives must be armed in pos mode
    #  - per Z: bag /level_diag for offline review
    #  - measure yaw too (small but informative for asymmetric mounts)
    #  - inner-loop config snapshot saved per-Z bag (drift detector)
    SYSID_DEFAULTS = {
        'z_mm_list':        [30.0, 35.0, 40.0, 45.0, 50.0],
        'delta_turns_list': [0.025, 0.050, 0.100],
        'settle_s':         1.5,
        'samples_per_step': 5,
        # Multiple +δ/-δ cycles per (Z, leg, δ) — final rpy_pos and
        # rpy_neg are averaged across reps. Cuts measurement noise by
        # ~√reps. reps=1 keeps the original single-shot behavior.
        'reps_per_step':    1,
    }

    def _do_sysid_start(self, params):
        if not self.armed:
            return False, "arm first (system-id needs all 6 legs in CLOSED_LOOP pos mode)"
        if self.limits is None:
            return False, "no leg_limits.yaml"
        if self.bus is None or self.feeder is None or self.listener is None:
            return False, "bus/feeder/listener not running"
        if self.level_enabled:
            return False, ("level loop is ON. Disable it first — system-ID "
                           "measures bare Δleg→Δrpy without the loop fighting "
                           "the perturbation.")
        if getattr(self, '_sysid_thread', None) is not None and \
                self._sysid_thread.is_alive():
            return False, "system-id already running"
        # Validate params; fall back to defaults.
        z_list = params.get('z_mm_list') or self.SYSID_DEFAULTS['z_mm_list']
        d_list = params.get('delta_turns_list') or self.SYSID_DEFAULTS['delta_turns_list']
        try:
            z_list = [float(z) for z in z_list]
            d_list = [float(d) for d in d_list]
        except (TypeError, ValueError) as e:
            return False, f"bad z_mm_list / delta_turns_list: {e}"
        if not z_list or not d_list:
            return False, "z_mm_list and delta_turns_list must each have ≥1 entry"
        if any(d <= 0 for d in d_list):
            return False, "all delta values must be > 0"
        if any(d > 0.3 for d in d_list):
            return False, ("delta > 0.3 turns is dangerous (may exceed leg "
                           "soft limits). Reduce.")
        try:
            reps = int(params.get('reps_per_step',
                                  self.SYSID_DEFAULTS['reps_per_step']))
        except (TypeError, ValueError):
            return False, "reps_per_step must be a positive integer"
        if reps < 1 or reps > 20:
            return False, "reps_per_step must be in [1, 20]"
        run_params = dict(self.SYSID_DEFAULTS)
        run_params.update({
            'z_mm_list': z_list,
            'delta_turns_list': d_list,
            'settle_s':         float(params.get('settle_s',
                                                 self.SYSID_DEFAULTS['settle_s'])),
            'samples_per_step': int(params.get('samples_per_step',
                                               self.SYSID_DEFAULTS['samples_per_step'])),
            'reps_per_step':    reps,
        })
        # Spawn worker thread. State exported via _sweep_state so the
        # existing /level_record_state heartbeat shows progress with
        # no GUI plumbing change.
        self._sweep_stop.clear()
        self._sysid_thread = threading.Thread(
            target=self._sysid_run, args=(run_params,), daemon=True)
        self._sysid_thread.start()
        return True, (f"system-id started: {len(z_list)} Z × "
                      f"{len(d_list)} δ × 6 legs")

    def _do_sysid_abort(self):
        self._sweep_stop.set()
        with self._bag_lock:
            proc = self._bag_proc
            self._bag_proc = None
            self._bag_dir = None
        if proc is not None:
            self._lr_stop_proc(proc)
        return True, "system-id abort flagged"

    def _sysid_eta_s(self, z_count, d_count, settle_s):
        # Per leg per delta: +δ (settle), back, -δ (settle), back. ~4 × settle.
        per_leg = 4 * settle_s
        per_z = 6 * d_count * per_leg + 2.0   # +2s for Z transition + bag start
        return z_count * per_z

    def _sysid_run(self, params):
        try:
            self._sysid_run_inner(params)
        except Exception as e:
            self.get_logger().error(f"system-id thread crashed: {e}")
            self._lr_set_state('aborted', error=f"sysid: {e}")

    def _sysid_run_inner(self, params):
        z_list = params['z_mm_list']
        d_list = params['delta_turns_list']
        settle_s = params['settle_s']
        n_samples = params['samples_per_step']
        n_reps = params['reps_per_step']

        # Total perturbation count for ETA / progress %.
        # Each (Z, δ, leg, rep) does +δ, return, -δ, return = 4 settle
        # waits + 2 IMU averages. Plus moving_to_z + capture rest at
        # each Z (~1.5 s).
        total_legs_to_perturb = len(z_list) * len(d_list) * 6
        per_perturb_s = (4 * settle_s + 0.2) * n_reps
        total_s_estimate = (
            len(z_list) * (per_perturb_s * 6 * len(d_list) + 4.0))
        t_start_mono = time.monotonic()

        stamp = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
        sysid_dir = os.path.join(BAGS_DIR, f"system_id_{stamp}")
        os.makedirs(sysid_dir, exist_ok=True)
        # Mirror the result into the repo's tuning_data/ so jacobian.json
        # is committable. The bags themselves stay in BAGS_DIR (they're
        # gitignored anyway). _sysid_dump writes to BOTH locations now.
        repo_sysid_dir = os.path.join(_BRINGUP_DIR, '..', 'tuning_data',
                                      f"system_id_{stamp}")
        repo_sysid_dir = os.path.abspath(repo_sysid_dir)
        os.makedirs(repo_sysid_dir, exist_ok=True)
        legs_done = 0
        # Stash for use later by the dump helper + state updates.
        self._sysid_progress = {
            't_start_mono': t_start_mono,
            'total_perturbs': total_legs_to_perturb,
            'estimate_s': total_s_estimate,
            'legs_done': 0,
            'sysid_dir': sysid_dir,
            'repo_sysid_dir': repo_sysid_dir,
        }

        # Same FF runtime self-heal as level — write
        # control_mode=POSITION, input_mode=PASSTHROUGH, wL_FF=True
        # before starting the perturbation series. Keeps the loop's
        # inner config consistent with how we'd run it during normal
        # level operation, so the Jacobian we measure here is valid
        # for the level loop as deployed.
        ok, msg = self._prepare_for_level()
        if not ok:
            self._lr_set_state('aborted', error=f"sysid prep: {msg}")
            return
        self.get_logger().info(f"system-id: {msg}")
        self.get_logger().info(
            f"system-id: starting — {len(z_list)} Z heights × "
            f"{len(d_list)} δ values × 6 legs × {n_reps} rep(s) = "
            f"{total_legs_to_perturb} (Z, δ, leg) × {n_reps} reps. "
            f"Estimated runtime ~{total_s_estimate/60:.1f} min.")

        # Master record: per-Z dicts of {delta: {'leg_n': {rpy_pos, rpy_neg, rpy_zero, encoder_zero}}}
        master = {
            'started_utc': datetime.datetime.utcnow().isoformat() + 'Z',
            'z_mm_list': z_list,
            'delta_turns_list': d_list,
            'settle_s': settle_s,
            'samples_per_step': n_samples,
            'git_sha': self._lr_git_sha(),
            'per_z': {},   # {z_str: {...}}
            'bags': [],    # list of bag_dir strings, one per Z
        }

        for z_idx, z in enumerate(z_list):
            if self._sweep_stop.is_set():
                self._lr_set_state('aborted', reason='user abort', sysid=True)
                break

            self._lr_set_state('sysid_running', phase='moving_to_z',
                               z=z, z_idx=z_idx + 1, z_total=len(z_list),
                               legs_done=self._sysid_progress['legs_done'],
                               legs_total=total_legs_to_perturb,
                               percent=round(
                                   self._sysid_progress['legs_done']
                                   / max(1, total_legs_to_perturb) * 100, 1))
            self.get_logger().info(
                f"sysid: moving to Z={z:.1f} mm "
                f"(Z {z_idx+1}/{len(z_list)})")
            ok, _ = self._do_set_pose(0.0, 0.0, float(z), 0.0, 0.0, 0.0,
                                      allow_large=True)
            if not ok:
                self._lr_set_state('aborted', error=f"set_pose Z={z} failed")
                break

            # Wait for the platform to physically reach Z and stop
            # slewing. _lr_wait_z_reached compares against the live
            # commanded targets so it works even when the level loop
            # is OFF (target == current_xyz IK output).
            if not self._lr_wait_z_reached(z, self._sweep_stop):
                self.get_logger().warn(f"sysid Z={z}: settle timeout, "
                                       f"continuing anyway")

            # Snapshot rest IMU + encoders for this Z (the linearization point).
            time.sleep(0.5)
            rest_rpy = self._sysid_avg_imu(n_samples)
            rest_enc = self.listener.get_all(max_age_s=0.5)
            if any(v is None for v in rest_enc):
                self._lr_set_state('aborted',
                                   error=f"Z={z}: stale encoders before perturbation")
                break

            # Spawn one bag for this Z, recording the entire perturbation
            # battery so we can review the per-leg responses offline.
            bag_dir = os.path.join(
                sysid_dir,
                datetime.datetime.utcnow().strftime('%H%M%SZ_') +
                f"z{int(round(z*10)):04d}")
            inner_cfg = self._lr_capture_inner_loop_config()
            with self._bag_lock:
                if self._bag_proc is not None and self._bag_proc.poll() is None:
                    self._lr_stop_proc(self._bag_proc)
                self._bag_proc = self._lr_spawn_bag(bag_dir)
                self._bag_dir = bag_dir
                self._bag_t0 = time.monotonic()
            meta = {
                'sysid_run': True,
                'sysid_z_mm': z,
                'sysid_z_idx': z_idx + 1,
                'sysid_z_total': len(z_list),
                'rest_rpy': rest_rpy,
                'rest_encoders': rest_enc,
                'delta_turns_list': d_list,
                'level_loop_hz': self.level_loop_hz,
            }
            if inner_cfg is not None:
                meta['inner_loop_config'] = inner_cfg
            self._lr_save_notes(bag_dir, f"sysid_z{int(round(z*10)):04d}",
                                'system-id child bag', meta)
            master['bags'].append(bag_dir)

            # Per Z: walk all 6 legs × all deltas. For each
            # combination: ±δ from rest, return to rest between.
            # Capture IMU averages at +δ, -δ, and at rest after return.
            per_z_data = {
                'rest_rpy': rest_rpy,
                'rest_encoders': rest_enc,
                'measurements': {},   # str(delta): {leg_n: {pos_rpy, neg_rpy}}
            }
            for d in d_list:
                if self._sweep_stop.is_set():
                    break
                per_d = {}
                for leg in range(6):
                    if self._sweep_stop.is_set():
                        break
                    elapsed = time.monotonic() - t_start_mono
                    legs_done = self._sysid_progress['legs_done']
                    pct = (legs_done / max(1, total_legs_to_perturb)) * 100
                    # Remaining estimate: assume current rate, fall
                    # back to original estimate when we have <2 done.
                    if legs_done >= 2:
                        s_per_leg = elapsed / legs_done
                        remaining_s = (total_legs_to_perturb - legs_done) * s_per_leg
                    else:
                        remaining_s = max(0.0, total_s_estimate - elapsed)
                    # Reps loop: each rep does +δ, return, -δ, return.
                    # Final rpy_pos / rpy_neg are averaged across reps so
                    # noise cuts ~√reps. reps=1 keeps the original
                    # single-shot timing for short scoping runs.
                    rpy_pos_reps = []
                    rpy_neg_reps = []
                    aborted = False
                    for rep in range(n_reps):
                        elapsed = time.monotonic() - t_start_mono
                        legs_done_now = self._sysid_progress['legs_done']
                        # Smooth percent: count partial credit for the
                        # rep currently in progress, so the bar doesn't
                        # freeze when reps>1.
                        partial = (rep + 0.0) / max(1, n_reps)
                        legs_eff = legs_done_now + partial
                        pct = (legs_eff / max(1, total_legs_to_perturb)) * 100
                        if legs_eff >= 2:
                            s_per_leg = elapsed / legs_eff
                            remaining_s = (total_legs_to_perturb - legs_eff) * s_per_leg
                        else:
                            remaining_s = max(0.0, total_s_estimate - elapsed)
                        self._lr_set_state(
                            'sysid_running', phase='perturbing',
                            z=z, z_idx=z_idx + 1, z_total=len(z_list),
                            leg=leg, delta=d,
                            delta_idx=d_list.index(d) + 1,
                            delta_total=len(d_list),
                            rep_idx=rep + 1, rep_total=n_reps,
                            legs_done=legs_done_now,
                            legs_total=total_legs_to_perturb,
                            elapsed_s=round(elapsed, 1),
                            remaining_s=round(remaining_s, 1),
                            percent=round(pct, 1))
                        self.get_logger().info(
                            f"sysid Z={z:.1f} δ={d:.3f} leg={leg} "
                            f"rep {rep+1}/{n_reps} → {pct:.1f}%  "
                            f"({legs_eff:.1f}/{total_legs_to_perturb} "
                            f"perturbs, ~{remaining_s/60:.1f} min left)")

                        # +δ
                        self.feeder.set_pos_target_one(
                            leg, float(rest_enc[leg]) + d)
                        if not self._sysid_wait(leg, settle_s, self._sweep_stop):
                            aborted = True; break
                        rpy_pos_reps.append(self._sysid_avg_imu(n_samples))

                        # Return to rest
                        self.feeder.set_pos_target_one(leg, float(rest_enc[leg]))
                        if not self._sysid_wait(leg, settle_s, self._sweep_stop):
                            aborted = True; break

                        # -δ
                        self.feeder.set_pos_target_one(
                            leg, float(rest_enc[leg]) - d)
                        if not self._sysid_wait(leg, settle_s, self._sweep_stop):
                            aborted = True; break
                        rpy_neg_reps.append(self._sysid_avg_imu(n_samples))

                        # Return to rest
                        self.feeder.set_pos_target_one(leg, float(rest_enc[leg]))
                        if not self._sysid_wait(leg, settle_s, self._sweep_stop):
                            aborted = True; break
                    if aborted:
                        break

                    # Average rpy across reps. Per-axis mean of each
                    # of the 3 components, dropping any rep whose IMU
                    # avg returned None.
                    def avg_axis(reps_list, axis):
                        vals = [r[axis] for r in reps_list
                                if r and r[axis] is not None]
                        return float(sum(vals) / len(vals)) if vals else None
                    rpy_pos = [avg_axis(rpy_pos_reps, i) for i in range(3)]
                    rpy_neg = [avg_axis(rpy_neg_reps, i) for i in range(3)]
                    per_d[str(leg)] = {
                        'rpy_pos': rpy_pos,
                        'rpy_neg': rpy_neg,
                        'rpy_pos_reps': rpy_pos_reps,   # raw for offline review
                        'rpy_neg_reps': rpy_neg_reps,
                        'reps_completed': len(rpy_pos_reps),
                    }
                    self._sysid_progress['legs_done'] += 1
                    # Per-leg checkpoint log so the terminal feels alive.
                    self.get_logger().info(
                        f"sysid Z={z:.1f} δ={d:.3f} leg={leg} done: "
                        f"rpy+ = ({rpy_pos[0]:+.3f},{rpy_pos[1]:+.3f},"
                        f"{rpy_pos[2]:+.3f}) "
                        f"rpy- = ({rpy_neg[0]:+.3f},{rpy_neg[1]:+.3f},"
                        f"{rpy_neg[2]:+.3f})")
                per_z_data['measurements'][f"{d:.4f}"] = per_d

            master['per_z'][f"{z:.1f}"] = per_z_data
            # Stop bag for this Z.
            with self._bag_lock:
                proc = self._bag_proc
                self._bag_proc = None
                self._bag_dir = None
            if proc is not None:
                self._lr_stop_proc(proc)
            time.sleep(0.5)
            # Incremental save of the master so we don't lose data
            # if a later Z aborts.
            self._sysid_dump(sysid_dir, master)

        # Compute Jacobian per (Z, δ).
        master['empirical_jacobian'] = self._sysid_compute_empirical(master)
        master['theoretical_jacobian'] = self._sysid_compute_theoretical(z_list, d_list)
        master['diff_pct_per_leg_per_z'] = self._sysid_compute_diff(master)
        self._sysid_dump(sysid_dir, master)

        if self._sweep_stop.is_set():
            self._lr_set_state('aborted', reason='user abort', sysid=True)
        else:
            self._lr_set_state('sysid_done', sysid_dir=sysid_dir,
                               z_count=len(z_list), d_count=len(d_list))

    def _sysid_wait(self, leg, settle_s, abort_event):
        """Wait `settle_s` for IMU/leg to settle. Bails on abort.
        Returns True if settle completed, False if aborted."""
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            if abort_event is not None and abort_event.is_set():
                return False
            time.sleep(0.02)
        return True

    def _sysid_avg_imu(self, n=5):
        """Average n IMU samples over ~50 ms. Drops the last reading
        timestamp into the result so the sidecar has the time series."""
        rpys = []
        for _ in range(max(1, n)):
            with self.imu_lock:
                rpy = self.imu_rpy
            if rpy is not None:
                rpys.append(tuple(rpy))
            time.sleep(0.01)
        if not rpys:
            return [None, None, None]
        ar = np.array(rpys)
        return [float(ar[:, i].mean()) for i in range(3)]

    def _sysid_compute_empirical(self, master):
        """Build {z_str: {delta_str: 3x6 jacobian list-of-lists}} from
        per_z['measurements']. Each Jacobian column for leg n is
        (rpy_pos - rpy_neg) / (2δ) for that (Z, δ)."""
        out = {}
        for z_key, z_data in master.get('per_z', {}).items():
            per_z_jac = {}
            meas = z_data.get('measurements', {}) or {}
            for d_key, per_d in meas.items():
                try:
                    d = float(d_key)
                except ValueError:
                    continue
                if d <= 0:
                    continue
                # 3 (roll/pitch/yaw) × 6 legs
                jac = [[None] * 6 for _ in range(3)]
                for leg_str, per_leg in per_d.items():
                    try:
                        leg = int(leg_str)
                    except ValueError:
                        continue
                    rp = per_leg.get('rpy_pos')
                    rn = per_leg.get('rpy_neg')
                    if rp is None or rn is None:
                        continue
                    for axis in range(3):
                        if rp[axis] is None or rn[axis] is None:
                            continue
                        jac[axis][leg] = (rp[axis] - rn[axis]) / (2.0 * d)
                per_z_jac[d_key] = jac
            out[z_key] = per_z_jac
        return out

    def _sysid_compute_theoretical(self, z_list, d_list):
        """Numeric Jacobian from _compute_motor_targets, for comparison.
        At pose (0,0,Z,0,0,0), perturb leg n by Δ in IK and infer the
        rpy that would produce that — finite-diff'd. Same convention
        as the empirical: 3×6 ∂rpy/∂leg_n. Empirical and theoretical
        should agree if the IK matches reality."""
        # The IK produces leg lengths from rpy. The inverse (rpy from
        # legs) is what we want here. We take a finite-difference of
        # the forward IK at +rpy_eps for each axis, which gives us
        # ∂legs/∂rpy (3x6 transpose). Pseudo-invert to get ∂rpy/∂legs.
        eps = 0.01  # 0.01° rpy perturbation for the FD
        out = {}
        for z in z_list:
            t0, _ = _compute_motor_targets(
                (0.0, 0.0, float(z)), (0.0, 0.0, 0.0),
                self.geom, self.limits)
            cols = []
            for axis in range(3):
                rpy = [0.0, 0.0, 0.0]
                rpy[axis] = eps
                t1, _ = _compute_motor_targets(
                    (0.0, 0.0, float(z)),
                    (rpy[0], rpy[1], rpy[2]),
                    self.geom, self.limits)
                col = [(t1[i] - t0[i]) / eps for i in range(6)]
                cols.append(col)
            # cols is 3x6: ∂legs/∂rpy_axis. We want ∂rpy/∂legs (3x6).
            # The 6×3 tall matrix is rank-deficient (only 3 dims of
            # rpy); pseudo-inverse gives the least-squares mapping
            # ∂rpy/∂legs.
            d_legs_d_rpy = np.array(cols).T   # shape (6, 3)
            d_rpy_d_legs = np.linalg.pinv(d_legs_d_rpy)   # shape (3, 6)
            # Same shape as empirical: dict by delta (theoretical
            # doesn't depend on δ, so emit identical entries for each).
            per_d = {}
            for d in d_list:
                per_d[f"{d:.4f}"] = d_rpy_d_legs.tolist()
            out[f"{z:.1f}"] = per_d
        return out

    def _sysid_compute_diff(self, master):
        """Per-cell %deviation: (emp - theo) / theo × 100. Useful for
        spotting which legs/axes deviate most. Cells where theo is
        near zero show up as None to avoid divide-by-zero noise."""
        emp = master.get('empirical_jacobian', {}) or {}
        theo = master.get('theoretical_jacobian', {}) or {}
        out = {}
        for z_key, e_per_d in emp.items():
            t_per_d = theo.get(z_key, {})
            per_d_out = {}
            for d_key, e_jac in e_per_d.items():
                t_jac = t_per_d.get(d_key)
                if not t_jac:
                    continue
                rows = []
                for axis in range(3):
                    row = []
                    for leg in range(6):
                        e = e_jac[axis][leg]
                        t = t_jac[axis][leg]
                        if e is None or t is None or abs(t) < 1e-6:
                            row.append(None)
                        else:
                            row.append(float((e - t) / t * 100.0))
                    rows.append(row)
                per_d_out[d_key] = rows
            out[z_key] = per_d_out
        return out

    def _sysid_dump(self, sysid_dir, master):
        # Write to BOTH the bag dir (for review alongside the rosbags)
        # AND the repo's tuning_data dir (so jacobian.json is committable
        # and can be loaded by future code as the empirical IK reference).
        targets = [os.path.join(sysid_dir, 'jacobian.json')]
        repo_dir = (getattr(self, '_sysid_progress', None) or {}).get('repo_sysid_dir')
        if repo_dir:
            targets.append(os.path.join(repo_dir, 'jacobian.json'))
        scrubbed = self._scrub_non_finite(master)
        for path in targets:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    json.dump(scrubbed, f, indent=2, allow_nan=False)
            except Exception as e:
                self.get_logger().warn(f"sysid dump to {path} failed: {e}")

    def _lr_wait_z_reached(self, z, abort_event):
        """Wait until leg encoders match the live commanded targets AND
        the encoders have stopped slewing (max-min < tol over the
        window). Returns True on success, False on timeout/abort.

        We compare against feeder.get_pos_targets() (the targets the
        level loop is actively writing) rather than a static IK of
        (0,0,z). The active level loop adds tilt corrections on top
        of bare IK, so a static target is offset from the actual
        commanded leg positions by however much tilt-correction the
        loop is currently applying — that's why the previous static-IK
        gate timed out forever once the loop wound up its corrections."""
        if self.listener is None or self.feeder is None:
            return False
        ring = []   # list of (t_mono, [pos[0..5]])
        deadline = time.monotonic() + self.LR_Z_REACHED_TIMEOUT_S
        tol_pos = self.LR_Z_REACHED_POS_TOL_TURNS
        tol_stab = self.LR_Z_REACHED_STAB_TOL_TURNS
        win_s = self.LR_Z_REACHED_VEL_WINDOW_S
        while time.monotonic() < deadline:
            if abort_event is not None and abort_event.is_set():
                return False
            now = time.monotonic()
            enc = self.listener.get_all(max_age_s=0.5)
            try:
                tgt = list(self.feeder.get_pos_targets())
            except Exception:
                tgt = None
            if all(v is not None for v in enc) and tgt is not None:
                ring.append((now, list(enc)))
                ring = [(t, p) for (t, p) in ring if now - t <= win_s]
                # Check 1: each encoder near its currently-commanded target
                near_target = all(
                    abs(enc[i] - float(tgt[i])) < tol_pos for i in range(6))
                # Check 2: encoders stable over the last win_s
                stable = False
                if near_target and ring and (now - ring[0][0]) >= (win_s * 0.9):
                    stable = True
                    for i in range(6):
                        vals = [p[i] for (_t, p) in ring]
                        if (max(vals) - min(vals)) > tol_stab:
                            stable = False
                            break
                if near_target and stable:
                    return True
            time.sleep(0.05)
        return False

    def _lr_run_sweep(self, params):
        zs = params['z_values']
        prefix = params['prefix']
        sweep_stamp = datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')
        sweep_dir = os.path.join(BAGS_DIR, f'sweep_{sweep_stamp}_{prefix}')
        try:
            os.makedirs(sweep_dir, exist_ok=True)
        except Exception as e:
            self._lr_set_state('aborted', error=f'mkdir: {e}')
            return
        sweep_t0_unix = time.time()
        bags = []
        # Track why the for-loop exited so the finally block can record
        # it in the manifest + state. None means "completed cleanly".
        # Using `break` instead of `return` for early exits ensures the
        # finally block runs and the manifest gets written — partial
        # sweeps are still analyzable.
        abort_reason = None
        # Per-Z notes: did the settle gate trip cleanly, or did we
        # record despite a timeout? Manifest captures this so the
        # analyzer can flag affected Zs.
        z_settle_status = []

        try:
            for idx, z in enumerate(zs):
                if self._sweep_stop.is_set():
                    abort_reason = 'user abort'
                    break
                # Phase: command Z, wait for arrival
                self._lr_set_state(
                    'sweep_running', phase='z_settle', z=z,
                    test_idx=idx + 1, test_total=len(zs))
                self.current_xyz = [0.0, 0.0, float(z)]
                self.current_rpy = [0.0, 0.0, 0.0]
                settled = self._lr_wait_z_reached(z, self._sweep_stop)
                if self._sweep_stop.is_set():
                    abort_reason = 'user abort'
                    break
                if not settled:
                    # Don't abort the whole sweep on settle timeout — the
                    # data we collect at unsettled Zs is exactly what we
                    # need to diagnose *why* the platform isn't settling.
                    # Log it, tag the manifest, continue.
                    z_settle_status.append({'z_mm': z, 'settled': False})
                    self._lr_set_state(
                        'sweep_running', phase='z_settle_timeout_continuing',
                        z=z, test_idx=idx + 1, test_total=len(zs),
                        warning=f'Z={z}: legs did not stabilize in '
                                f'{self.LR_Z_REACHED_TIMEOUT_S}s, recording anyway')
                    self.get_logger().warn(
                        f'sweep: Z={z} settle timeout — recording anyway')
                else:
                    z_settle_status.append({'z_mm': z, 'settled': True})
                # Phase: zero integrator, start bag, baseline
                self._level_zero_integ_request.set()
                # Per-Z bag goes inside the sweep dir, not in BAGS_DIR root.
                bag_name = f'z{int(round(z*10)):04d}'   # Z=40.0 → z0400 (deci-mm)
                bag_dir = os.path.join(
                    sweep_dir, datetime.datetime.utcnow().strftime('%H%M%SZ_') + bag_name)
                # Inner-loop config snapshot for this Z (same as the
                # single-bag path in _lr_start_bag). Captured per-Z so
                # if a parameter drifts mid-sweep — say, a drive's
                # `current_soft_max` self-throttles after thermal
                # buildup — the per-Z sidecar will capture it.
                inner_cfg = self._lr_capture_inner_loop_config()
                meta = {
                    'sweep_dir': sweep_dir,
                    'sweep_test_idx': idx + 1,
                    'sweep_test_total': len(zs),
                    'commanded_z_mm': z,
                    'step_count': params['step_count'],
                    'step_amp_deg': params['step_amp_deg'],
                    'step_hold_s': params['step_hold_s'],
                    'baseline_s': params['baseline_s'],
                    'level_loop_hz': self.level_loop_hz,
                    'ctrl_period_s': self.ctrl_period_s,
                }
                if inner_cfg is not None:
                    meta['inner_loop_config'] = inner_cfg
                bag_spawn_failed = False
                with self._bag_lock:
                    if self._bag_proc is not None and self._bag_proc.poll() is None:
                        self._lr_stop_proc(self._bag_proc)
                    try:
                        self._bag_proc = self._lr_spawn_bag(bag_dir)
                        self._bag_dir = bag_dir
                        self._bag_t0 = time.monotonic()
                    except Exception as e:
                        bag_spawn_failed = True
                        abort_reason = f'bag spawn at Z={z}: {e}'
                if bag_spawn_failed:
                    break
                self._lr_save_notes(bag_dir, bag_name,
                                    'auto-sweep child bag', meta)
                # Baseline
                self._lr_set_state(
                    'sweep_running', phase='baseline', z=z,
                    test_idx=idx + 1, test_total=len(zs),
                    baseline_s=params['baseline_s'])
                end = time.monotonic() + params['baseline_s']
                while time.monotonic() < end:
                    if self._sweep_stop.is_set():
                        break
                    time.sleep(0.05)
                # Step battery
                if not self._sweep_stop.is_set():
                    self._lr_set_state(
                        'sweep_running', phase='steps', z=z,
                        test_idx=idx + 1, test_total=len(zs),
                        step_total=params['step_count'])
                    self._lr_step_battery(
                        params['step_amp_deg'],
                        params['step_hold_s'],
                        params['step_count'],
                        self._sweep_stop)
                # Stop bag for this Z
                with self._bag_lock:
                    proc = self._bag_proc
                    self._bag_proc = None
                    self._bag_dir = None
                if proc is not None:
                    self._lr_stop_proc(proc)
                bags.append({'z_mm': z, 'bag_dir': bag_dir})
                if self._sweep_stop.is_set():
                    abort_reason = 'user abort'
                    break
        except Exception as e:
            abort_reason = f'sweep crashed: {e}'
            self.get_logger().error(f"sweep crashed: {e}")
        finally:
            # Always write a manifest — even on abort or crash — so
            # partial sweeps are analyzable. (Bug fix 2026-04-28: prior
            # code returned early on settle-timeout and skipped this.)
            try:
                manifest = {
                    'sweep_dir': sweep_dir,
                    'started_utc': datetime.datetime.utcfromtimestamp(
                        sweep_t0_unix).isoformat() + 'Z',
                    'ended_utc': datetime.datetime.utcnow().isoformat() + 'Z',
                    'aborted': abort_reason is not None,
                    'abort_reason': abort_reason,
                    'params': params,
                    'gains': dict(self.level_gains),
                    'git_sha': self._lr_git_sha(),
                    'bags': bags,
                    # Per-Z record of whether the settle gate fired
                    # cleanly. Useful when interpreting RMS / step
                    # response — Zs marked settled=False were
                    # recorded under non-settled conditions.
                    'z_settle_status': z_settle_status,
                }
                with open(os.path.join(sweep_dir, 'manifest.json'), 'w') as f:
                    json.dump(manifest, f, indent=2)
            except Exception as e:
                self.get_logger().warn(f"manifest write failed: {e}")
            # Publish final state
            if abort_reason is not None:
                self._lr_set_state(
                    'aborted', sweep_dir=sweep_dir, reason=abort_reason,
                    completed_levels=len(bags), total_levels=len(zs))
            else:
                self._lr_set_state(
                    'idle', last_sweep_dir=sweep_dir,
                    completed_levels=len(bags), total_levels=len(zs))
            # Ensure step offsets are zeroed even on uncaught exit.
            self.level_step_offset_roll = 0.0
            self.level_step_offset_pitch = 0.0

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
    # Strip our own CLI flags (everything after `--ros-args` is rclpy's).
    # `--level-loop-hz N` is the only stewart-side arg today.
    import argparse
    own_args = sys.argv[1:]
    if '--ros-args' in own_args:
        cut = own_args.index('--ros-args')
        own_args, ros_tail = own_args[:cut], own_args[cut:]
        rclpy_argv = [sys.argv[0]] + ros_tail
    else:
        rclpy_argv = sys.argv
    p = argparse.ArgumentParser(add_help=False)
    p.add_argument('--level-loop-hz', type=float, default=None,
                   help=f'level loop sample rate in Hz (default '
                        f'{LEVEL_DEFAULT_HZ:.0f}). Outer-loop gains in '
                        f'level_gains.yaml are interpreted physically '
                        f'(cutoff Hz, slew °/s) and scaled internally to '
                        f'this rate, so YAML tuned at any rate stays valid.')
    parsed, _unknown = p.parse_known_args(own_args)
    rclpy.init(args=rclpy_argv)
    node = StewartControlNode(level_loop_hz=parsed.level_loop_hz)
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
