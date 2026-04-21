"""Bundle launch: dual Xsens IMUs + stewart_control_node + rosbridge.

After launch:
  - both IMU topics publishing at 400 Hz
  - stewart_control_node offering services and publishing leg_encoders /
    platform_rpy / status
  - rosbridge websocket listening on ws://localhost:9090
  - open ~/ros2_ws/install/stewart_bringup/share/stewart_bringup/web/index.html
    (or the symlink source at ~/ros2_ws/src/stewart_bringup/web/index.html)
    in a browser to drive the robot
"""
import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    # NOTE: previously set ROS_LOCALHOST_ONLY=1 and
    # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST to work around a rosbridge
    # service-discovery bug. We've since switched all browser <-> node
    # control to topic-based /control_cmd, and the LOCALHOST env var
    # turned out to cause its own problem: launched nodes and CLI tools
    # ended up in mismatched discovery scopes, hiding /status from
    # `ros2 topic echo` and from the reset-script's --verify poll.
    # Now using the default discovery behavior.

    # Dual IMU launch (reused verbatim)
    imu_launch = os.path.join(
        get_package_share_directory('stewart_bringup'),
        'launch', 'imu_dual_launch.py')
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch)))

    # Control node
    ld.add_action(Node(
        package='stewart_bringup',
        executable='stewart_control_node',
        name='stewart_control_node',
        output='screen',
        emulate_tty=True,
    ))

    # rosbridge_websocket (default port 9090, host 0.0.0.0). Delay 3 s so
    # stewart_control_node finishes DDS discovery publishing; otherwise the
    # first browser connect can hit the ~1 s service-wait timeout for some
    # services (seen as "Service /X does not exist" errors even though the
    # CLI finds them fine).
    rb_launch = os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch', 'rosbridge_websocket_launch.xml')
    ld.add_action(TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rb_launch))],
    ))

    return ld
