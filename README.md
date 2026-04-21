# Stable-Bot

Stewart-platform control stack: ROS 2 Kilted nodes + browser GUI.

Contains two ROS 2 packages:

- `stewart_bringup` — main control node, web GUI, launch files, scripts.
- `jugglebot_interfaces` — custom message/service types.

## Getting started

Clone into a ROS 2 workspace:

    cd ~/ros2_ws/src
    git clone https://github.com/Luke-McDevitt/stable_bot.git
    cd ~/ros2_ws
    colcon build --symlink-install

Full setup + Pi migration instructions live under
`stewart_bringup/docs/` inside the clone.
