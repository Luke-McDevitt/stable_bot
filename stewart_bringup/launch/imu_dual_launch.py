"""Launch two Xsens MTi-630 driver instances simultaneously - base + platform.

Physical setup:
  - /dev/imu_mti630   (serial DBBBRHGX) mounted on the BASE
  - /dev/imu_mti630_b (serial DBBBRHBS) mounted on the PLATFORM

Output topics (both drivers running in parallel):
  /base/imu/data,    /base/filter/euler,    /base/imu/mag,    /base/status ...
  /platform/imu/data, /platform/filter/euler, /platform/imu/mag, /platform/status ...

IMPORTANT: the Xsens driver uses ABSOLUTE topic names (hardcoded "/imu/data"
etc. with leading slashes). Namespace launch arguments DON'T prefix
absolute topics, so we explicitly remap every topic per-instance below.
If you ever add or remove pub_* flags in the YAML, update the remap list.

If you want to swap which physical unit is base vs platform, just flip the
'port' values - the udev symlinks follow the serial number, not the USB port.
"""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# All topic names the Xsens driver hardcodes (leading slash absolute paths).
# Grep'd from src/messagepublishers/*.h on 2026-04-20. If the driver version
# changes, re-verify.
XSENS_ABSOLUTE_TOPICS = [
    '/imu/data',
    '/imu/acceleration',
    '/imu/angular_velocity',
    '/imu/mag',
    '/imu/time_ref',
    '/imu/dq',
    '/imu/dv',
    '/filter/quaternion',
    '/filter/euler',
    '/filter/free_acceleration',
    '/status',
    '/temperature',
    '/pressure',
    # GNSS-only (disabled in our YAML but remap anyway for completeness)
    '/filter/twist',
    '/filter/velocity',
    '/filter/positionlla',
    '/filter/ship_motion',
    '/gnss',
    '/gnss_pose',
    '/nmea',
    '/odometry',
    # High-rate (disabled in our YAML)
    '/imu/acceleration_hr',
    '/imu/angular_velocity_hr',
]


def namespaced_remaps(ns):
    """Build a list of (abs_topic, /<ns>/abs_topic[1:]) pairs."""
    return [(t, f"/{ns}{t}") for t in XSENS_ABSOLUTE_TOPICS]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    yaml_path = str(Path(
        get_package_share_directory('stewart_bringup'),
        'config',
        'xsens_mti_stewart.yaml',
    ))

    base_overrides = {
        'scan_for_devices': False,
        'port': '/dev/imu_mti630',    # serial DBBBRHGX
        'frame_id': 'base_imu_link',
    }
    platform_overrides = {
        'scan_for_devices': False,
        'port': '/dev/imu_mti630_b',  # serial DBBBRHBS
        'frame_id': 'platform_imu_link',
    }

    ld.add_action(Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_base',
        namespace='base',
        output='screen',
        parameters=[yaml_path, base_overrides],
        remappings=namespaced_remaps('base'),
    ))

    ld.add_action(Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_platform',
        namespace='platform',
        output='screen',
        parameters=[yaml_path, platform_overrides],
        remappings=namespaced_remaps('platform'),
    ))
    return ld
