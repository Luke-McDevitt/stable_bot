#!/usr/bin/env python3
"""ball_kf_node — constant-velocity Kalman filter on the ball position.

State vector x = [px, py, vx, vy] in the platform frame, units mm and mm/s.

This is the bounding-box-tracker derivation from Lecture 8 slides 32–34
applied to a 2-D position measurement. Predict at 100 Hz; update
whenever ball_localizer_node publishes /ball_xy_mono.

Outputs:
  /ball_state (geometry_msgs/PoseStamped)  — position in pose.position;
                                              velocity in orientation.x/y
                                              (scaffold; replace with a
                                              custom BallState.msg later)
  /ball_state/cov (std_msgs/Float32MultiArray) — flattened 4x4 covariance

TODOs flagged in code:
  - Tune Q from rolling-friction acceleration variance.
  - Tune R from a logged sit-still session (parked ball, log noise).
  - Optional EKF upgrade: fold tilted-plate dynamics into the motion
    model (spec §8). Worth it if Demo 2 settling is choppy.
  - Re-emit predictions BETWEEN measurements (the controller runs
    faster than detection); current code only publishes on update.
"""
from __future__ import annotations

import os
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float32MultiArray


class BallKFNode(Node):
    DT = 1.0 / 100.0   # nominal predict step

    def __init__(self):
        super().__init__('ball_kf')

        self.x = np.zeros(4)             # [px, py, vx, vy]
        self.P = np.eye(4) * 1e3         # large initial uncertainty (mm^2)

        self.F = np.array([
            [1, 0, self.DT, 0],
            [0, 1, 0, self.DT],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float64)

        # Process noise (Q): models unmodeled acceleration as white noise.
        # The ball's real max accel on a tilted plate is α·g·sin(max_tilt) ≈
        # 1000 mm/s²; the old 5000 over-trusted the measurement and let the
        # velocity estimate chase vision noise (the spiky |v| we saw in the
        # coast/demo bags). Default 2000 (physical + margin). Tune via
        # BALL_KF_SIGMA_A; see estimate_kf_noise.py + docs/pi_power_throttling.
        sigma_a = float(os.environ.get('BALL_KF_SIGMA_A', '2000.0'))
        sigma_a2 = sigma_a ** 2
        dt = self.DT
        # Discrete white-noise acceleration model
        G = np.array([[0.5 * dt * dt], [0.5 * dt * dt], [dt], [dt]])
        # Independent x and y → block-diagonal
        Gx = np.array([[0.5 * dt * dt], [0], [dt], [0]])
        Gy = np.array([[0], [0.5 * dt * dt], [0], [dt]])
        self.Q = sigma_a2 * (Gx @ Gx.T + Gy @ Gy.T)

        # Measurement model: observe (px, py) directly.
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float64)

        # Measurement noise (R): vision std (mm). Tune via BALL_KF_R_MM from
        # the still-ball scatter (estimate_kf_noise.py reads /ball_xy_mono).
        # 1 mm is the historical placeholder.
        r_mm = float(os.environ.get('BALL_KF_R_MM', '1.0'))
        self.R = np.eye(2) * (r_mm ** 2)

        # Innovation gating — reject false-positive DETECTION outliers (a
        # detection that jumps far from the prediction), the dominant source of
        # the 4000+ mm/s velocity spikes that a fixed R can't suppress. Reject
        # a measurement when its Mahalanobis distance y'·S⁻¹·y (chi², 2 DOF)
        # exceeds the gate. The gate is ADAPTIVE: S grows when the filter is
        # uncertain (fast ball), so genuine fast motion is still accepted; only
        # a stationary/slow ball's wild jumps are dropped. After too many
        # consecutive rejects the ball really moved/was replaced → re-acquire.
        # Defaults: chi²(2) at ~99.97% (16) and 8 rejects (~0.3–0.6 s of
        # detections). Tune via BALL_KF_GATE_CHI2 / BALL_KF_GATE_MAX_REJECT;
        # set BALL_KF_GATE_CHI2 huge to disable.
        self._gate_chi2 = float(os.environ.get('BALL_KF_GATE_CHI2', '16.0'))
        self._gate_max_reject = int(
            float(os.environ.get('BALL_KF_GATE_MAX_REJECT', '8')))
        self._reject_streak = 0
        self.get_logger().info(
            f"ball_kf: R={r_mm}mm sigma_a={sigma_a}mm/s^2 "
            f"gate_chi2={self._gate_chi2} max_reject={self._gate_max_reject}")

        self.have_meas = False
        # Time of last accepted measurement. Used to gate /ball_state
        # publication: when the ball falls off the platform, the
        # localizer's on-platform gate drops /ball_xy_mono publication,
        # but the KF would otherwise keep extrapolating forever and
        # /ball_state would keep arriving at downstream consumers
        # (BALL_TRACK loop, GUI SVG). Without this gate, the controller
        # reacts to a phantom ball indefinitely until the operator
        # presses Stop.
        self._last_meas_t = None
        # Capture stamp (OAK photon time) of the last accepted measurement,
        # propagated from the localizer via /ball_xy_mono.header.stamp. Used
        # ONLY to report photon→state latency in /ball_state.orientation.z;
        # never used for filter timing (that stays on the local clock).
        self._last_meas_cap_stamp = None
        # Ball is "lost" if no measurement received in this many
        # seconds. Tuned to be longer than the typical V0 detection
        # interval (~70 ms at 14 fps) plus a generous margin, but
        # shorter than the BALL_TRACK ball-fall recovery (3 s).
        self._meas_stale_s = 0.4

        self.create_subscription(
            PointStamped, '/ball_xy_mono', self._on_meas, 10)
        self.pub_state = self.create_publisher(
            PoseStamped, '/ball_state', 10)
        self.pub_cov = self.create_publisher(
            Float32MultiArray, '/ball_state/cov', 10)

        self.create_timer(self.DT, self._predict_and_publish)

    def _predict_and_publish(self):
        # Predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        # Publish only when measurements are reasonably fresh. KF still
        # predicts internally so the next measurement sees a sensible
        # prior, but /ball_state stops broadcasting → BALL_TRACK loop
        # sees stale state, fires the existing 0.5 s zero-tilt branch,
        # then the 3 s ball-fall recovery.
        if self.have_meas and self._last_meas_t is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_meas_t) <= self._meas_stale_s:
                self._publish()

    def _on_meas(self, msg: PointStamped):
        z = np.array([msg.point.x, msg.point.y])  # mm in platform frame
        # Innovation + gating.
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        Sinv = np.linalg.inv(S)
        d2 = float(y @ Sinv @ y)                  # Mahalanobis², chi²(2 DOF)
        if d2 > self._gate_chi2:
            self._reject_streak += 1
            if self._reject_streak < self._gate_max_reject:
                # Outlier (false-positive detection): drop it, keep predicting.
                # NOTE: do NOT refresh _last_meas_t — if every detection is an
                # outlier the staleness gate should still fire (ball lost).
                return
            # Too many rejects in a row → the ball genuinely moved or was
            # replaced. Re-acquire: snap to the measurement, zero velocity,
            # large uncertainty, then fall through to a (now-consistent) update.
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            self.P = np.eye(4) * 1e3
            y = np.zeros(2)
            S = self.H @ self.P @ self.H.T + self.R
            Sinv = np.linalg.inv(S)
        self._reject_streak = 0
        # Record the photon capture stamp carried on the measurement (see
        # _publish). Behaviour-neutral: not used in the update below.
        self._last_meas_cap_stamp = msg.header.stamp
        # Update
        K = self.P @ self.H.T @ Sinv
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.have_meas = True
        self._last_meas_t = self.get_clock().now().nanoseconds * 1e-9
        # Event-driven publish: emit the fresh posterior immediately instead
        # of waiting up to one 100 Hz predict tick (cuts ≤10 ms off the
        # vision→control latency). The timer keeps publishing predictions
        # between measurements. The single-threaded executor serialises this
        # with the timer, so there's no re-entrancy.
        self._publish()

    def _publish(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'platform'
        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.position.z = 0.0
        # Stuffing velocity into orientation.{x,y} as a scaffold hack;
        # see TODO at top of file for the proper BallState.msg path.
        msg.pose.orientation.x = float(self.x[2])
        msg.pose.orientation.y = float(self.x[3])
        # orientation.z carries the photon→state latency in SECONDS: the
        # age of the last measurement's capture time at publish. Right
        # after an update this is the fresh vision-pipeline latency
        # (capture→localize→KF→publish); between updates it grows, so the
        # digest reads the per-cycle troughs as latency and the rest as
        # staleness. Spare field — verified nothing downstream reads
        # /ball_state.orientation.z/.w (control + digests use only x/y), so
        # this adds a per-stage latency probe with no new topic or msg.
        if self._last_meas_cap_stamp is not None:
            cap_s = (self._last_meas_cap_stamp.sec
                     + self._last_meas_cap_stamp.nanosec * 1e-9)
            now_s = self.get_clock().now().nanoseconds * 1e-9
            msg.pose.orientation.z = float(max(0.0, now_s - cap_s))
        else:
            msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub_state.publish(msg)

        cov = Float32MultiArray()
        cov.data = [float(v) for v in self.P.ravel()]
        self.pub_cov.publish(cov)


def main():
    rclpy.init()
    node = BallKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
