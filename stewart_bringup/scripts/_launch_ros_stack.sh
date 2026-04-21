#!/usr/bin/env bash
# Helper script called by start_stable_bot.bat.
# Sources ROS 2 + workspace overlay, then runs the stewart_gui_launch.
set -e

source /opt/ros/kilted/setup.bash
source "$HOME/ros2_ws/install/local_setup.bash"

echo ""
echo "[_launch_ros_stack] ros2 launch stewart_bringup stewart_gui_launch.py"
echo ""

exec ros2 launch stewart_bringup stewart_gui_launch.py
