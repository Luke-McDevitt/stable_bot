#!/usr/bin/env python3
"""bag_recorder_node — auto-record a rosbag2 bag during each ball demo.

Every time the controller transitions from LEVEL_HOLD into one of the
BALL_TRACK_* modes, this node spawns `ros2 bag record` on a fixed
topic allowlist and tags the bag with a timestamp + demo type. When
the controller transitions back to LEVEL_HOLD (or the off-platform
watchdog trips), the recording is terminated cleanly.

Why a subprocess instead of `rosbag2_py`:
  - Subprocess has battle-tested error recovery and storage rotation.
  - The `ros2 bag record` CLI handles topic discovery edge cases that
    rosbag2_py forces us to reproduce (topics that show up after the
    record starts, etc.).
  - Easier for the user to inspect the running process via `ps`.

Bag location: ~/stable_bot_bags/<ISO-timestamp>_<mode>.bag/
Faulty runs get a `_fault` suffix when the watchdog fires.

Topic allowlist (no images by default — they're ~50 MB/min and the
plotter doesn't need them):
  /encoders, /platform_rpy, /imu/data, /platform_pose
  /ball_state, /ball_ref
  /ball_xy_oak, /ball_xy_mono, /ball_xy_stereo
  /method_comparison
  /control_cmd, /control_result, /odrive_errors
  /oak/ball/diagnostic
  /platform_pose/markers_visible
  /ball_path/active

To include images (large bags, only for runs going into the report):
  publish `String('record_images:on')` on /control_cmd; the next
  demo Start will include the image topics.

Spec: ../../stewart_bringup/docs/closed_loop_ball_demos.md §13.6.
"""
from __future__ import annotations

import datetime
import json
import os
import shlex
import signal
import subprocess
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


BAG_DIR = os.path.expanduser('~/stable_bot_bags')

# Default topic allowlist — small messages only.
TOPICS_DEFAULT: List[str] = [
    # Existing stewart_bringup stack
    '/encoders',
    '/platform_rpy',
    '/imu/data',
    '/odrive_errors',
    '/control_cmd',
    '/control_result',
    # Vision pipeline
    '/platform_pose',
    '/platform_pose/markers_visible',
    '/ball_state',
    '/ball_state/cov',
    '/ball_ref',
    '/ball_xy_oak',
    '/ball_xy_mono',
    '/ball_xy_stereo',
    '/method_comparison',
    '/oak/ball/diagnostic',
    '/oak/ball/v0/rgb_pixel',
    '/oak/ball/v0/diagnostic',
    '/oak/ball/v1/rgb_pixel',
    # Demo 3
    '/ball_path/active',
]

# Optional image topics — large, opt-in via /control_cmd.
TOPICS_IMAGES: List[str] = [
    '/oak/rgb/image_compressed',
    '/oak/left/image_compressed',
    '/oak/right/image_compressed',
    '/oak/disparity_compressed',
]


def _timestamp_iso() -> str:
    return datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ')


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        os.makedirs(BAG_DIR, exist_ok=True)

        self.proc: Optional[subprocess.Popen] = None
        self.bag_path: Optional[str] = None
        self.current_mode: str = 'LEVEL_HOLD'
        self.include_images: bool = False
        self._fault_pending: bool = False
        self._lock = threading.Lock()

        self.create_subscription(String, '/control_cmd',
                                 self._on_control_cmd, 10)
        # Fault detection: when ball_localizer or ref_generator
        # publishes a fault status to /control_result, we tag the bag.
        self.create_subscription(String, '/control_result',
                                 self._on_control_result, 10)

        self.pub_status = self.create_publisher(
            String, '/bag_recorder/status', 5)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"bag_recorder ready. Bags will land in {BAG_DIR}.")

    # ---- /control_cmd dispatch ---------------------------------------------

    def _on_control_cmd(self, msg: String):
        text = msg.data
        if ':' not in text:
            return
        key, _, payload = text.partition(':')
        key = key.strip()
        payload = payload.strip()

        # Image-recording toggle.
        if key == 'record_images':
            self.include_images = (payload.lower() in ('on', 'true', '1'))
            self.get_logger().info(
                f"include_images={self.include_images} "
                f"(applies to next demo Start).")
            return

        # Mode-transition handling.
        if key == 'mode':
            new_mode = payload.split(maxsplit=1)[0].upper()
            old_mode = self.current_mode
            self.current_mode = new_mode

            # Entering a tracking mode → start recording.
            tracking_modes = ('BALL_TRACK_GOTO',
                              'BALL_TRACK_TRAJECTORY',
                              'BALL_TRACK_PATH')
            if new_mode in tracking_modes and old_mode == 'LEVEL_HOLD':
                self._start_recording(new_mode)
            # Leaving tracking → stop recording.
            elif old_mode in tracking_modes and new_mode == 'LEVEL_HOLD':
                self._stop_recording()

    def _on_control_result(self, msg: String):
        """Detect off-platform watchdog or other faults that should
        tag the bag for later filtering. Convention: control_result
        payloads matching ".*fault.*" or ".*ball_lost.*" trigger the
        tag."""
        try:
            d = json.loads(msg.data) if msg.data.startswith('{') else {}
        except Exception:
            d = {}
        text = (d.get('event') or msg.data or '').lower()
        if 'fault' in text or 'ball_lost' in text or 'off_platform' in text:
            with self._lock:
                self._fault_pending = True

    # ---- recording control --------------------------------------------------

    def _start_recording(self, mode: str):
        with self._lock:
            if self.proc is not None:
                self.get_logger().warn(
                    "tried to start recording but a recording is already "
                    "running — ignoring.")
                return

            self._fault_pending = False
            ts = _timestamp_iso()
            mode_short = mode.replace('BALL_TRACK_', '').lower()
            name = f"{ts}_{mode_short}"
            bag_path = os.path.join(BAG_DIR, name)
            topics = list(TOPICS_DEFAULT)
            if self.include_images:
                topics += TOPICS_IMAGES

            cmd = ['ros2', 'bag', 'record',
                   '-o', bag_path,
                   '--storage', 'mcap'] + topics
            self.get_logger().info(
                f"starting bag: {bag_path}  ({len(topics)} topics, "
                f"images={'on' if self.include_images else 'off'})")
            try:
                # Detach so SIGINT to the parent doesn't immediately
                # take down the child; we send SIGINT explicitly to the
                # child when we want recording to stop.
                self.proc = subprocess.Popen(
                    cmd,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    start_new_session=True,
                )
                self.bag_path = bag_path
            except FileNotFoundError:
                self.get_logger().error(
                    "ros2 bag record not found — is ROS 2 sourced in this "
                    "shell? Bag recording disabled until restart.")
                self.proc = None
                self.bag_path = None

    def _stop_recording(self):
        with self._lock:
            proc = self.proc
            bag_path = self.bag_path
            fault = self._fault_pending
            self.proc = None
            self.bag_path = None
            self._fault_pending = False

        if proc is None:
            return

        # Send SIGINT to the process group so rosbag2 can flush cleanly.
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                "ros2 bag did not exit after SIGINT, sending SIGKILL.")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=2)

        # Tag the bag if a fault was raised during the run.
        if fault and bag_path and os.path.isdir(bag_path):
            new_path = bag_path + '_fault'
            try:
                os.rename(bag_path, new_path)
                bag_path = new_path
            except OSError as e:
                self.get_logger().warn(
                    f"could not rename to fault-tagged path: {e}")

        self.get_logger().info(
            f"bag closed: {bag_path}  "
            f"({'WITH FAULT' if fault else 'clean'})")

    # ---- status -------------------------------------------------------------

    def _publish_status(self):
        with self._lock:
            recording = self.proc is not None
            bag = self.bag_path
            mode = self.current_mode
            include_images = self.include_images
        status = {
            'recording': recording,
            'bag_path': bag,
            'current_mode': mode,
            'include_images': include_images,
        }
        m = String()
        m.data = json.dumps(status)
        self.pub_status.publish(m)

    def destroy_node(self):
        # Ensure any in-flight recording is closed cleanly on shutdown.
        try:
            self._stop_recording()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
