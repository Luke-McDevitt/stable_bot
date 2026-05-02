# Stable-Bot 
(https://github.com/Luke-McDevitt/stable_bot)
Stewart-platform ball-balancing system: ROS 2 Kilted nodes + browser GUI
+ vision pipeline. Ships closed-loop click-to-goto control of a ball on
a 6-DOF platform with sub-mm tracking accuracy.

Links to more youtube videos coming soon!
https://youtu.be/bEEpE1CrJFA?si=kh_AgsYBSKObmkfR

## What's here

Three ROS 2 packages:

- **`stewart_bringup`** — main control node (`stewart_control_node`),
  web GUI (`gui_server.py` + `web/index.html`), level-PI inner loop,
  BALL_TRACK closed-loop controller, launch files, deploy scripts,
  digest/comparison tooling.
- **`stewart_vision`** — OAK-D camera driver, ArUco platform-pose
  estimator, parallel ball detectors (cv2 HSV + on-device YOLO),
  Kalman filter, frame-alignment calibration.
- **`jugglebot_interfaces`** — custom message and service types.

Hardware: 6× ODrive Pro motor controllers over CAN, 2× MTi-630 IMUs,
Luxonis OAK-D-PRO-AF camera, Raspberry Pi 5.

## Status

- ✅ **Demo 1 (orbit)** — implemented, tunable
- ✅ **Demo 2 (click-to-goto)** — settled at **0.5 mm median tracking
  error** (bag `20260502T025210Z_demo2`)
- ⚠️ **Demo 3 (path drawing)** — `BALL_TRACK_PATH` is stubbed, returns
  the first waypoint only. Open work.

## Getting started

Clone into a ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Luke-McDevitt/stable_bot.git
cd ~/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install \
    --packages-select jugglebot_interfaces stewart_bringup stewart_vision
source install/local_setup.bash
```

Launch (Pi-side, headless):

```bash
sudo systemctl start stable_bot.service stable_bot_gui.service
```

Then open `http://stablebot.local:8080/` in a laptop browser. The
header connection dot turns green within ~5 s; arm, level, and run
demos from the panel.

For laptop / dev workflow, see
[`PI_MIGRATION.md`](stewart_bringup/docs/PI_MIGRATION.md).

## Documentation

### How it works

- [`ARCHITECTURE.md`](stewart_bringup/docs/ARCHITECTURE.md) — how the
  nodes, topics, and services fit together.
- [`closed_loop_ball_demos.md`](stewart_bringup/docs/closed_loop_ball_demos.md)
  — design spec for the three demos (vision, ArUco markers, ball
  detection, KF, controllers, GUI).
- [`closed_loop_ball_demos_usage.md`](stewart_bringup/docs/closed_loop_ball_demos_usage.md)
  — operator guide: cold start, calibration, running each demo, safety.
- [`level_loop_architecture.md`](stewart_bringup/docs/level_loop_architecture.md)
  — inner level-PI loop design.

### How it got here

The journey docs are read end-to-end if you're going to touch the
controls or vision code; they explain *why* every architectural
choice was made.

- [`controls_journey.md`](stewart_bringup/docs/controls_journey.md)
  — vision rate fixes, ArUco↔IMU frame alignment, sign-convention
  derivation (2026-04-30).
- [`vision_control_tuning_lessons.md`](stewart_bringup/docs/vision_control_tuning_lessons.md)
  — vision pipeline lessons + controller architecture changes
  (2026-04-30).
- [`step_id_tuning_lessons.md`](stewart_bringup/docs/step_id_tuning_lessons.md)
  — STEP_ID plant identification, two-tilt-thresholds insight,
  cascade-bypass decision (2026-05-01).
- [`level_loop_lessons_learned.md`](stewart_bringup/docs/level_loop_lessons_learned.md)
  — runtime-vs-persistent CAN state, empirical Jacobian discovery
  (2026-04-27 → 2026-04-29).
- [`demo2_shipping_journey_2026-05-02.md`](stewart_bringup/docs/demo2_shipping_journey_2026-05-02.md)
  — the day Demo 2 shipped: empirical IK in BALL_TRACK, stiction ramp,
  position-delta exit, the winning gain set.

### Compiled cheat sheet

- [**`LESSONS.md`**](stewart_bringup/docs/LESSONS.md) — every durable
  rule from the journey docs in one terse compilation. Organized by
  topic (ODrive, vision, frames/IK, control theory, stiction, tooling,
  things that didn't work). Read this if you have 10 minutes and want
  the executive summary.

### Reference

- [`ODRIVE_PARAMS.md`](stewart_bringup/docs/ODRIVE_PARAMS.md) — every
  ODrive parameter we set, why, and the runtime-vs-persistent path.
- [`REFLASH_FIRMWARE.md`](stewart_bringup/docs/REFLASH_FIRMWARE.md) —
  ODrive firmware update procedure.
- [`PI_MIGRATION.md`](stewart_bringup/docs/PI_MIGRATION.md) — Pi 5
  headless install, hotspot setup, gotchas (apt sources, arm64
  Xsens rebuild, two-services pattern).
- [`TROUBLESHOOTING.md`](stewart_bringup/docs/TROUBLESHOOTING.md) —
  recovery recipes for rosbridge / CAN / IMU issues.
- [`NEXT_STEPS.md`](stewart_bringup/docs/NEXT_STEPS.md) — roadmap from
  the early scripted-rolling-ball state.

## Tooling

In `stewart_bringup/scripts/`:

| script | purpose |
|---|---|
| `pi_deploy.sh` | full Pi deploy: pull, colcon-build the right packages, restart both services. |
| `digest_demo_bag.py` | post-process a Demo 2/3 bag → digest.png + digest.summary.json + next-step gain recommendation. |
| `digest_step_id_bag.py` | post-process a STEP_ID bag → plant gain G_eff fit + analytic Kp/Kd/Ki recommendation. |
| `compare_demo_bags.py` | walk every digest.summary.json across a bag glob, sortable table of gains vs outcome metrics. `--diff` collapses to only the columns that varied. |
| `smooth_demo_bag.py` | offline non-causal Savitzky-Golay smoother on raw vision detections. Outputs cleaner ground-truth velocity than the live causal KF. Suitable as ML training labels. |
| `calibrate_oak.py` | OAK intrinsic + extrinsic calibration (Stage A / B / C). |

Top-level `repo_loc.py` counts lines of code per language across the
repo.

## Hardware reference

| Component | Detail |
|---|---|
| MCUs | 6× ODrive Pro on CAN bus (gs_usb adapter, VID:PID 1d50:606f) |
| IMUs | 2× MTi-630 over USB serial (`/dev/imu_mti630`, `/dev/imu_mti630_b`), 921600 baud, 400 Hz |
| Camera | Luxonis OAK-D-PRO-AF, USB-3 SuperSpeed, mounted on overhead arm |
| Compute | Raspberry Pi 5 (16 GB), Ubuntu 24.04, ROS 2 Kilted |
| Markers | ArUco DICT_4X4_50, 8 markers on a 120 mm-radius ring on the platform |

## Project layout

```
stable_bot/
  README.md                          ← you are here
  repo_loc.py                        ← lines-of-code counter
  jugglebot_interfaces/              ← custom msg/srv types
  stewart_bringup/
    config/
      ball_track_gains.yaml          ← BALL_TRACK PID + stiction-ramp gains
      level_gains.yaml               ← level-PI gains
      leg_limits.yaml                ← per-leg soft-limits in turns
      global_limits.yaml             ← hard tilt-rate caps
    docs/                            ← architecture + journey + lessons
    launch/                          ← stewart_gui_launch.py
    scripts/
      pi_deploy.sh                   ← deploy
      stable_bot.service             ← systemd unit (control stack)
      stable_bot_gui.service         ← systemd unit (web GUI)
      digest_demo_bag.py             ← per-bag analysis
      compare_demo_bags.py           ← cross-bag comparison
      smooth_demo_bag.py             ← offline smoother
    stewart_bringup/
      stewart_control_node.py        ← main control node (~5500 lines)
      auto_tune_node.py              ← (legacy) hill-climb auto-tuner
    web/index.html                   ← single-file Tailwind+roslibjs GUI
  stewart_vision/
    stewart_vision/
      oak_driver_node.py             ← OAK pipeline + on-device detectors
      ball_localizer_node.py         ← pixel → platform-frame mm + ArUco→IMU rotation
      ball_kf_node.py                ← Kalman filter on ball position
      platform_pose_node.py          ← ArUco-board pose
      ref_generator_node.py          ← demo target trajectories
    config/
      aruco_imu_alignment.yaml       ← Procrustes-fit rotation
      marker_layout.yaml             ← ArUco board geometry
      oak_extrinsics.yaml            ← camera-to-platform calibration
    blobs/                           ← compiled neural-net detectors for the OAK VPU
    scripts/
      calibrate_oak.py
      build_v0_blob.py
  tuning_data/                       ← bag dirs + digests, committed for traceability
  training_data/                     ← YOLO labels + cached metadata (gitignored bulk)
```

## Contributing / future work

Open work, in roughly priority order:

1. **Demo 3 (path drawing)** — implement `BALL_TRACK_PATH` ref
   generator (arc-length traversal at speed). ~30-line implementation
   per the spec.
2. **Live KF Q/R re-tuning** — use offline-smoother truth from
   `smooth_demo_bag.py` as ground truth for a Q/R grid search.
3. **Learned ball-state filter** — train a small RNN on (raw vision +
   IMU) → (smoothed position, velocity), where targets come from the
   offline smoother. Replaces or augments the live KF.
4. **End-to-end latency measurement** — automated digest analyzer
   that times from V0 detection to motor response. Lets us tune
   `control_latency_s` empirically per-config.
5. **Vision noise mitigation at the source** — better lighting,
   shorter exposure, ball-surface texture. Biggest single physical
   improvement, no code.

## License

GNU General Public License (GPL) v3
