# stewart_vision

Vision pipeline for the Stable-Bot Stewart platform.

This package implements the OAK-D Pro AF driver, ArUco-ring platform
pose recovery, ball localization (mono projection + stereo
triangulation), the Kalman tracker, and the reference generators for
the three closed-loop ball demos defined in
`../stewart_bringup/docs/closed_loop_ball_demos.md`.

The package is **kept separate from `stewart_bringup`** so that anyone
wanting a different vision stack (RealSense, mocap, mono webcam, …)
can swap this package out without touching the controller. Topics are
the contract; node implementations are interchangeable.

## Status

**v0.1 — scaffold.** Package builds and nodes run, but most algorithm
implementations are stubs marked with TODO. The OAK driver
(`oak_driver_node`) is the most fleshed-out node and is wired against
real DepthAI for the OAK-D Pro AF.

## Build

From your colcon workspace root (the repo expected to be checked out
under `<ws>/src/stable_bot_repo`):

```bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select stewart_vision
source install/local_setup.bash
```

## Run

```bash
ros2 launch stewart_vision stewart_vision_launch.py
```

Or run nodes individually:

```bash
ros2 run stewart_vision oak_driver_node
ros2 run stewart_vision platform_pose_node
ros2 run stewart_vision ball_localizer_node
ros2 run stewart_vision ball_kf_node
ros2 run stewart_vision ref_generator_node
```

## Calibration

```bash
python3 scripts/calibrate_oak.py --stage all  # first time
python3 scripts/calibrate_oak.py --stage C    # after every camera move
```

See §6 of the spec for stages A/B/C details.

## Layout

```
stewart_vision/
├── stewart_vision/         # ROS 2 nodes (Python)
├── scripts/                # Calibration + dataset + plotting tools
├── config/                 # marker_layout.yaml, ball_safety.yaml, intrinsics
├── launch/                 # stewart_vision_launch.py
└── docs/                   # README; spec lives in stewart_bringup/docs/
```

## Hardware target

Luxonis OAK-D Pro AF (auto-focus IMX378 RGB + dual OV9282 stereo monos
+ IR projector + IR illumination + Movidius Myriad X VPU). DepthAI
≥ 2.24 expected. The driver should also work on plain OAK-D and OAK-D
Pro (fixed-focus) without changes — auto-focus controls are conditional.

## Dependencies (Python)

- `depthai` ≥ 2.24 (OAK device API)
- `opencv-contrib-python` (ArUco lives in contrib)
- `numpy`
- `cv_bridge` (ROS 2 ↔ OpenCV)
- `PyYAML`

Install via apt where possible (`ros-kilted-cv-bridge`); pip the rest.
