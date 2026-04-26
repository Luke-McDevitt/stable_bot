"""Bring up the full vision stack for the Stable-Bot ball demos.

Launches:
  - oak_driver_node      (DepthAI pipeline; RGB + L/R mono + disparity)
  - platform_pose_node   (ArUco ring → /platform_pose @ 30 Hz)
  - ball_localizer_node  (mono projection + stereo triangulation)
  - ball_kf_node         (constant-velocity KF → /ball_state @ 100 Hz)
  - ref_generator_node   (modes from /control_cmd → /ball_ref)

Spec: ../stewart_bringup/docs/closed_loop_ball_demos.md
"""
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

    return ld
