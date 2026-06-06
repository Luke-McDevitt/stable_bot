"""Bring up the full vision stack for the Stable-Bot ball demos.

Launches:
  - oak_driver_node      (DepthAI pipeline; RGB + L/R mono + disparity)
  - platform_pose_node   (ArUco ring → /platform_pose @ 30 Hz)
  - ball_localizer_node  (mono projection + stereo triangulation)
  - ball_kf_node         (constant-velocity KF → /ball_state @ 100 Hz)
  - ref_generator_node   (modes from /control_cmd → /ball_ref)

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='stewart_vision',
        executable='oak_driver_node',
        name='oak_driver',
        output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='stewart_vision',
        executable='platform_pose_node',
        name='platform_pose',
        output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='stewart_vision',
        executable='ball_localizer_node',
        name='ball_localizer',
        output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='stewart_vision',
        executable='ball_kf_node',
        name='ball_kf',
        output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='stewart_vision',
        executable='ref_generator_node',
        name='ref_generator',
        output='screen',
        emulate_tty=True,
    ))
    ld.add_action(Node(
        package='stewart_vision',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
        emulate_tty=True,
    ))
    # Legacy auto-recorder: spawns its OWN `ros2 bag record` to
    # ~/stable_bot_bags/ on every LEVEL→BALL_TRACK transition. DISABLED BY
    # DEFAULT (2026-06-06): it fully duplicates the GUI auto-bag (which
    # records to tuning_data/ and is what the digest pipeline consumes) —
    # the YOLO demo profile caught the two overlapping recorders at ~36% of
    # a core combined, this one ~16-19% of pure redundancy. Re-enable with
    # STABLE_BOT_LEGACY_BAGREC=1 if you still want the stable_bot_bags
    # copies (broader topic set: /ball_state/cov, /ball_xy_stereo, …).
    if os.environ.get('STABLE_BOT_LEGACY_BAGREC', '0') == '1':
        ld.add_action(Node(
            package='stewart_vision',
            executable='bag_recorder_node',
            name='bag_recorder',
            output='screen',
            emulate_tty=True,
        ))

    return ld
