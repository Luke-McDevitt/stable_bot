"""Launch the Xsens MTi-630 driver with the stewart_bringup param override.

Uses our version-controlled copy of the YAML at
stewart_bringup/config/xsens_mti_stewart.yaml, which sets:
  - port to /dev/imu_mti630 (udev symlink for serial DBBBRHGX)
  - baudrate 921600
  - output_data_rate 400 Hz
  - pub_imu=true (publishes /imu/data), pub_mag=true
  - GNSS/pressure/temperature publishers disabled (not an INS variant)

Run:
  ros2 launch stewart_bringup imu_launch.py
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    param_file = Path(
        get_package_share_directory('stewart_bringup'),
        'config',
        'xsens_mti_stewart.yaml',
    )

    ld.add_action(Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        parameters=[str(param_file)],
    ))
    return ld
