#!/usr/bin/env python3
"""Measure true Pi-side cv2 vs YOLO detector latency.

The GUI's latency readouts go through rosbridge + websocket to the
laptop, which adds 50-200 ms of transport plus any Pi-laptop clock
skew. The CONTROLLER subscribes directly via DDS on the Pi, so its
latency is whatever this script reports.

Each PointStamped message carries the OAK capture time in
header.stamp (set by oak_driver_node from dai.Clock, converted to
the ROS clock). Subtracting that from the Pi's current ROS clock at
receipt time gives the true capture → publish-receive latency.

Run on the Pi (NOT a remote machine — needs same DDS domain):

    source /opt/ros/kilted/setup.bash
    python3 stewart_vision/scripts/measure_detector_latency.py

Reports rolling mean + min/max for each backend over 10 seconds.
"""
from __future__ import annotations

import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped


class LatencyMeter(Node):
    def __init__(self, duration_s: float = 10.0):
        super().__init__('detector_latency_meter')
        self.duration_s = duration_s
        self.start_t = self.get_clock().now()
        self.samples = {'cv2': [], 'yolo': []}
        self.create_subscription(
            PointStamped, '/oak/ball/v0/cv2_pixel',
            lambda m: self._on(m, 'cv2'),
            qos_profile_sensor_data)
        self.create_subscription(
            PointStamped, '/oak/ball/v0/yolo_pixel',
            lambda m: self._on(m, 'yolo'),
            qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)

    def _on(self, msg: PointStamped, label: str) -> None:
        now = self.get_clock().now()
        stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                    + msg.header.stamp.nanosec)
        now_ns = now.nanoseconds
        lat_ms = (now_ns - stamp_ns) / 1e6
        if 0 <= lat_ms < 1000:
            self.samples[label].append(lat_ms)

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds / 1e9
        if elapsed < self.duration_s:
            return
        # Time's up — print and exit.
        print(f"\nMeasured for {elapsed:.1f} s (Pi-side, no rosbridge)\n")
        print(f"{'backend':<8} {'n':>5} {'mean (ms)':>12}"
              f" {'min':>8} {'max':>8} {'rate Hz':>10}")
        print('-' * 60)
        for label, vals in self.samples.items():
            if not vals:
                print(f"{label:<8} {0:>5}    (no messages — backend "
                      f"probably not active)")
                continue
            mean = statistics.mean(vals)
            mn = min(vals)
            mx = max(vals)
            rate = len(vals) / elapsed
            print(f"{label:<8} {len(vals):>5} {mean:>12.1f}"
                  f" {mn:>8.1f} {mx:>8.1f} {rate:>10.1f}")
        print()
        rclpy.shutdown()


def main():
    rclpy.init()
    duration = 10.0
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            pass
    print(f"Measuring detector latency for {duration:.1f} s "
          f"(switch backend mid-run via /oak/cmd_v0_backend if you "
          f"want both populated)")
    print("(NOTE: cv2 only publishes when the host actually runs cv2 "
          "detection — flip to cv2 in the GUI dropdown OR enable the "
          "cv2 circle toggle to wake it up.)")
    node = LatencyMeter(duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


if __name__ == '__main__':
    main()
