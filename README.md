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

Full docs live under `stewart_bringup/docs/`:

- [`ARCHITECTURE.md`](stewart_bringup/docs/ARCHITECTURE.md) — how the nodes, topics, and services fit together.
- [`PI_MIGRATION.md`](stewart_bringup/docs/PI_MIGRATION.md) — headless install on a Raspberry Pi, hotspot setup, connecting a laptop browser, plus the full list of gotchas we hit migrating (apt sources, arm64 rebuild of the Xsens driver, repo-layout path differences, duplicate-stack pitfall).
- [`TROUBLESHOOTING.md`](stewart_bringup/docs/TROUBLESHOOTING.md) — recovery recipes for rosbridge / CAN / IMU issues.
- [`NEXT_STEPS.md`](stewart_bringup/docs/NEXT_STEPS.md) — roadmap for the real-time rolling-ball demo using a stereo camera + IR-illuminated ball.
