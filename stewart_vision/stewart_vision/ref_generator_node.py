#!/usr/bin/env python3
"""ref_generator_node — reference-trajectory generator for the three demos.

Subscribes to:
  /control_cmd  (std_msgs/String) — mode + JSON params (spec §12)
  /ball_state   (geometry_msgs/PoseStamped) — current ball position
  /ball_path/upload (nav_msgs/Path) — full polyline for Demo 3 (one-shot)

Publishes:
  /ball_ref          (geometry_msgs/PointStamped) — current target (mm, platform frame)
  /ball_path/active  (nav_msgs/Path) — the validated polyline being followed (Demo 3)

Mode dispatcher follows the /control_cmd payloads documented in the
spec (§12). Each mode produces /ball_ref differently:
  LEVEL_HOLD             — no /ball_ref
  BALL_TRACK_GOTO        — fixed point until cancelled
  BALL_TRACK_TRAJECTORY  — Demo 1: orbit at radius R, period T, dir, phase
  BALL_TRACK_PATH        — Demo 3: traverse uploaded polyline at speed

This scaffold implements the orbit and goto modes. Path mode is
stubbed pending integration with the GUI's path-upload protocol.

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md (§3, §11.2, §12).
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Optional

import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

from stewart_vision._ball_physics import BALL_PRESETS, get_preset


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('stewart_vision')


class RefGeneratorNode(Node):
    REF_RATE = 60.0

    def __init__(self):
        super().__init__('ref_generator')

        # Load safety parameters (dead-zone enforcement is mirrored from
        # the controller — first line of defence, spec §10).
        share = _share_dir()
        safety_path = os.path.join(share, 'config', 'ball_safety.yaml')
        with open(safety_path, 'r') as f:
            self.safety = yaml.safe_load(f)
        self.dead_zone_mm = float(self.safety['dead_zone_mm'])
        self.ball_radius_mm = float(self.safety['ball_radius_mm'])
        self.min_radius_mm = self.dead_zone_mm + self.ball_radius_mm

        # Mode state.
        self.mode = 'LEVEL_HOLD'
        self.params = {}
        self.t0 = time.monotonic()
        self.path: Optional[Path] = None

        # Waypoint-driven trajectory state. When BALL_TRACK_TRAJECTORY
        # is started with mode='waypoint' (the default since 2026-04-30),
        # the ref doesn't smoothly interpolate around the orbit at fixed
        # angular speed — it sits on each precomputed orbit waypoint
        # until the ball arrives within `arrival_tol_mm`, dwells for
        # `dwell_s`, then advances to the next. This makes Demo 1 a
        # tracking-error test instead of a chase test, and works at
        # any control gain set instead of needing the loop tuned for
        # the orbit's angular speed.
        self._wp_list: list = []          # [(x_mm, y_mm), ...] CCW order
        self._wp_idx: int = 0
        self._wp_arrived_t: Optional[float] = None
        self._wp_started_t: Optional[float] = None    # for max_wait timeout
        self._wp_signature: tuple = ()    # (radius, dir, n) — rebuild if changed

        # Ball-config state (spec v9 §9, Q47). Default to foam until the
        # GUI publishes its first ball_config: payload. The actual control
        # feedforward lives in stewart_control_node (stewart_bringup); we
        # just track + log here so the bag has the value at run start.
        self.ball_cfg: dict = get_preset('foam')

        self.create_subscription(String, '/control_cmd',
                                 self._on_control_cmd, 10)
        self.create_subscription(Path, '/ball_path/upload',
                                 self._on_path, 10)
        self.create_subscription(PoseStamped, '/ball_state',
                                 self._on_state, 10)
        self.last_state: Optional[PoseStamped] = None

        self.pub_ref = self.create_publisher(PointStamped, '/ball_ref', 10)
        self.pub_active = self.create_publisher(Path, '/ball_path/active', 10)

        self.create_timer(1.0 / self.REF_RATE, self._tick)

    def _on_state(self, msg: PoseStamped):
        self.last_state = msg

    def _on_control_cmd(self, msg: String):
        text = msg.data
        if ':' not in text:
            return
        key, _, payload = text.partition(':')
        key = key.strip()
        payload = payload.strip()
        if key == 'ball_config':
            try:
                cfg = json.loads(payload)
                # Accept any preset-shaped dict; minimal validation.
                if 'type' not in cfg or 'alpha' not in cfg:
                    self.get_logger().warn(
                        f"ball_config missing required keys: {cfg}")
                    return
                changed = (cfg.get('type') != self.ball_cfg.get('type'))
                self.ball_cfg = cfg
                if changed:
                    self.get_logger().info(
                        f"Ball config → {cfg['type']} "
                        f"(α={cfg.get('alpha', '?'):.4f}, "
                        f"density={cfg.get('density', '?')})")
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"Bad ball_config JSON: {e}")
            return

        if key == 'mode':
            # Payload is "MODE_NAME" or "MODE_NAME <JSON>"
            parts = payload.split(maxsplit=1)
            mode = parts[0].upper()
            if mode not in (
                'LEVEL_HOLD', 'BALL_TRACK_GOTO',
                'BALL_TRACK_TRAJECTORY', 'BALL_TRACK_PATH',
                'BALL_HOLD',  # active stabilization, scaffolded v9, full impl v10
            ):
                self.get_logger().warn(f"Unknown mode: {mode}")
                return
            try:
                self.params = json.loads(parts[1]) if len(parts) > 1 else {}
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"Bad JSON params: {e}")
                self.params = {}
            # Pick up ball config if it was embedded in the start payload.
            if isinstance(self.params.get('ball'), dict):
                self.ball_cfg = self.params['ball']
            self.mode = mode
            self.t0 = time.monotonic()
            self.get_logger().info(
                f"Mode → {mode} params={self.params} ball={self.ball_cfg.get('type')}")

    def _on_path(self, msg: Path):
        if msg.header.frame_id != 'platform':
            self.get_logger().warn(
                f"Path frame_id is {msg.header.frame_id!r}, expected "
                f"'platform' — rejecting.")
            return
        # Pre-flight reject: any pose inside the dead-zone or off
        # platform fails the whole upload (spec §10.1, Demo 3 rule).
        for ps in msg.poses:
            r = math.hypot(ps.pose.position.x, ps.pose.position.y)
            if r < self.min_radius_mm:
                self.get_logger().warn(
                    f"Path rejected: sample at r={r:.1f} mm violates "
                    f"dead-zone (min {self.min_radius_mm:.1f} mm).")
                return
            if r > self.safety['off_platform_radius_mm']:
                self.get_logger().warn(
                    f"Path rejected: sample at r={r:.1f} mm is off "
                    f"platform.")
                return
        self.path = msg
        self.pub_active.publish(msg)
        self.get_logger().info(
            f"Path accepted: {len(msg.poses)} samples.")

    def _tick(self):
        if self.mode == 'LEVEL_HOLD':
            return
        ref = self._compute_ref()
        if ref is None:
            return
        x_mm, y_mm = ref
        # Last-line safety on the generator side: clamp out of dead-zone.
        r = math.hypot(x_mm, y_mm)
        if r < self.min_radius_mm:
            scale = (self.min_radius_mm + 1e-3) / max(r, 1e-3)
            x_mm *= scale
            y_mm *= scale
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'platform'
        msg.point.x = float(x_mm)
        msg.point.y = float(y_mm)
        msg.point.z = 0.0
        self.pub_ref.publish(msg)

    def _compute_ref(self):
        if self.mode == 'BALL_TRACK_GOTO':
            x = float(self.params.get('x_mm', 0.0))
            y = float(self.params.get('y_mm', 0.0))
            return x, y
        if self.mode == 'BALL_TRACK_TRAJECTORY':
            R = float(self.params.get('radius_mm', 80.0))
            direction = 1.0 if self.params.get('dir', 'CCW').upper() == 'CCW' \
                else -1.0
            traj_mode = str(self.params.get('mode', 'waypoint')).lower()

            if traj_mode == 'continuous':
                # Original time-parameterized smooth orbit. Kept for
                # the rare case where you actually want to test phase
                # lag at high orbital speed.
                T = float(self.params.get('period_s', 12.0))
                phase = float(self.params.get('phase_rad', 0.0))
                t = time.monotonic() - self.t0
                theta = direction * 2.0 * math.pi * t / T + phase
                return R * math.cos(theta), R * math.sin(theta)

            # Waypoint-driven orbit (default). The ref sits on each
            # waypoint until the ball arrives within tolerance, dwells
            # briefly, then advances. Per-waypoint timeout prevents a
            # stuck ball from locking the demo forever.
            n_wp = max(3, int(self.params.get('n_waypoints', 12)))
            tol_mm = float(self.params.get('arrival_tol_mm', 20.0))
            dwell_s = float(self.params.get('dwell_s', 0.3))
            max_wait_s = float(self.params.get('max_wait_s', 8.0))
            phase = float(self.params.get('phase_rad', 0.0))

            sig = (R, direction, n_wp, phase)
            if sig != self._wp_signature or not self._wp_list:
                # Rebuild the waypoint list. Always CCW in math; the
                # `direction` flag controls iteration order.
                pts = []
                for i in range(n_wp):
                    theta = phase + 2.0 * math.pi * i / n_wp
                    pts.append((R * math.cos(theta),
                                R * math.sin(theta)))
                if direction < 0:
                    pts = [pts[0]] + list(reversed(pts[1:]))
                self._wp_list = pts
                self._wp_idx = 0
                self._wp_arrived_t = None
                self._wp_started_t = time.monotonic()
                self._wp_signature = sig

            now = time.monotonic()
            if self._wp_started_t is None:
                self._wp_started_t = now
            wp = self._wp_list[self._wp_idx]

            if self.last_state is not None:
                p = self.last_state.pose.position
                dist = math.hypot(p.x - wp[0], p.y - wp[1])
                if dist < tol_mm:
                    if self._wp_arrived_t is None:
                        self._wp_arrived_t = now
                    elif (now - self._wp_arrived_t) >= dwell_s:
                        # Advance.
                        self._wp_idx = (self._wp_idx + 1) % n_wp
                        self._wp_arrived_t = None
                        self._wp_started_t = now
                        wp = self._wp_list[self._wp_idx]
                else:
                    # Ball drifted out of tolerance; reset arrival
                    # timer so we don't advance prematurely.
                    self._wp_arrived_t = None

            # Per-waypoint timeout. If the ball can't reach the
            # current waypoint within max_wait_s, log and advance —
            # better to keep the orbit moving than have the demo
            # silently lock up on a stuck ball.
            if (self._wp_started_t is not None
                    and (now - self._wp_started_t) > max_wait_s):
                self.get_logger().warn(
                    f"orbit waypoint {self._wp_idx} ({wp[0]:.0f}, {wp[1]:.0f}) "
                    f"timeout after {max_wait_s:.1f}s — advancing")
                self._wp_idx = (self._wp_idx + 1) % n_wp
                self._wp_arrived_t = None
                self._wp_started_t = now
                wp = self._wp_list[self._wp_idx]

            return wp[0], wp[1]
        if self.mode == 'BALL_HOLD':
            # Active stabilization: hold at the captured target. The
            # base-IMU feedforward layer that makes this useful lives
            # in stewart_control_node (deferred to v10, spec §11.7).
            # For now we just emit the stationary reference.
            x = float(self.params.get('x_mm', 0.0))
            y = float(self.params.get('y_mm', 0.0))
            return x, y
        if self.mode == 'BALL_TRACK_PATH':
            # TODO: parameterize the uploaded path by arc length, advance
            # at self.params.get('speed_mm_s', 80) mm/s, support reverse
            # and loop. For now, target the path's first point so the
            # goto law drives the ball there and stops.
            if self.path is None or not self.path.poses:
                return None
            p = self.path.poses[0].pose.position
            return p.x, p.y
        return None


def main():
    rclpy.init()
    node = RefGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
