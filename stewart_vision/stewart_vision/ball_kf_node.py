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
        # TODO tune; placeholder uses sigma_a = 5000 mm/s^2 (rough order
        # of magnitude for a tilted-plate ball under nominal control).
        sigma_a2 = 5000.0 ** 2
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

        # Measurement noise (R): TODO measure empirically. 1 mm rms is
        # a reasonable starting guess for the mono-projection path with
        # a clean ArUco pose.
        self.R = np.eye(2) * (1.0 ** 2)

        self.have_meas = False

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
        if self.have_meas:
            self._publish()

    def _on_meas(self, msg: PointStamped):
        z = np.array([msg.point.x, msg.point.y])  # mm in platform frame
        # Update
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.have_meas = True

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
