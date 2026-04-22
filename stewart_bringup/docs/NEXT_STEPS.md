# Next Steps — Closed-loop rolling-ball demo with vision

Current state (2026-04-22): the platform can execute a **scripted** rolling-ball
routine — the ball's path is precomputed in platform coordinates and the tilt
commands are open-loop. The demo looks right as long as nothing perturbs the
ball. Any disturbance (bump, spin-up wobble, off-center release) and the
controller has no idea.

The next piece of work is to **measure where the ball actually is** in
real time and close the loop: the control node gets the ball's (x, y)
position on the disk, compares to the desired trajectory, and tilts the
platform to correct. That turns the demo from "choreography" into actual
stabilization.

This doc sketches the path.

## What the system needs

- Ball position on the disk plane, (x, y) in mm, at ≥ 60 Hz, with ≤ 30 ms
  end-to-end latency (lens → image → detection → control node → ODrive
  command).
- Reasonable robustness to: ambient light changes, users' hands coming
  into frame, the platform itself moving under the camera (camera sees
  the disk from above, ideally mounted rigid-to-world, not rigid-to-platform).
- Localisation accuracy of a few mm — the disk is ~200–300 mm across and
  the ball is ~40 mm, so mm-level is the right scale.

## Hardware options (ranked by how much work they save you)

1. **Luxonis OAK-D Lite / OAK-D S2** — stereo + onboard Movidius VPU. The
   camera runs the detection (YOLO, blob, or a custom pipeline) and publishes
   just the ball's 3D position to the Pi over USB. The Pi 4 doesn't have
   to burn CPU on disparity maps. This is the path of least resistance.
   ROS 2 driver: [`depthai-ros`](https://github.com/luxonis/depthai-ros).
   ~$150–$300.

2. **Intel RealSense D435 / D435i** — active IR stereo. Excellent depth
   quality on low-texture surfaces, which matters if the disk is a solid
   color. ROS 2 driver: `realsense2_camera`. The Pi 4 can just barely
   handle 640×480 @ 30 Hz if you don't ask for point clouds.
   ~$250.

3. **DIY two-USB-cam stereo + OpenCV** — cheapest (~$40 total) but you
   build the calibration, rectification, and detection pipeline yourself.
   Only reasonable if you want the learning experience.

4. **Top-down monocular + known ball diameter** — skip stereo entirely.
   Mount a single camera rigidly above the disk looking straight down.
   Because the ball always sits on the disk plane, a single image plus
   camera intrinsics gives (x, y) via a homography. Much simpler than
   stereo, and 2D is all the controller actually needs for a rolling-ball
   demo. This is probably where I'd start.

## Making the ball easy to see (the IR angle)

Even a good camera struggles with a black ball on a black mat under
fluorescent office lighting. Two mechanical tricks that turn a hard CV
problem into a trivial one:

- **Retroreflective ball + IR ring light**. Stick a strip of
  retroreflective tape (the stuff on running shoes / bike jackets) on the
  ball. Surround the camera lens with an IR LED ring (~850 nm). Add an
  850 nm bandpass filter on the camera. The ball lights up like a flare
  and *nothing else in the room does*. Detection becomes: threshold at
  ~200, find the biggest blob, centroid = ball position. Rock-solid under
  any ambient lighting, and users in frame don't confuse it.
- **Active IR LED inside the ball**. Drill a 3D-printed ball hollow,
  drop in a coin cell + a single IR LED, glue it shut. Same principle but
  self-powered — useful if the camera can't be lens-on-axis with the
  disk.

IR is the trick professional motion-capture rigs use (OptiTrack, Vicon).
You get their robustness for ~$20 in parts.

## ROS 2 integration sketch

New package `stewart_vision` (separate from `stewart_bringup`):

```
stewart_vision/
  stewart_vision/
    ball_tracker_node.py   # subscribes to camera, publishes /ball_xy
    calibrator.py          # one-shot: find the disk center + radius in pixels
  config/
    camera_intrinsics.yaml
    disk_homography.yaml
  launch/
    ball_tracker_launch.py
```

Topic contract:

- **Input**: `/image_raw` (sensor_msgs/Image) from whatever camera driver
  you pick.
- **Output**: `/ball_xy` (geometry_msgs/PointStamped) in the platform's
  top-surface frame, with `frame_id='platform'` and a timestamp matching
  the camera exposure.
- **Output**: `/ball_state` (std_msgs/String JSON) with `{detected: bool,
  latency_ms: N, quality: 0.0–1.0}` so the GUI can show a health indicator.

The `stewart_control_node` gains a new level-loop mode `BALL_TRACK` that
subscribes to `/ball_xy`, runs a PID on (x, y) error, and writes a tilt
(roll, pitch) target. The existing tilt→leg-target pipeline stays the
same — vision just replaces the precomputed trajectory.

## Putting it on the Pi

- The Pi 4 can't do dense stereo in real time. If you pick RealSense or
  DIY stereo, the Pi 5 is basically required. An OAK-D offloads the work
  and makes a Pi 4 fine.
- Camera power: USB3 is mandatory for any stereo camera doing meaningful
  frame rates. The Pi 4's USB3 ports share a single bus — don't put the
  CAN adapter on the same hub.
- Add `ball_tracker.service` as a third systemd unit so it starts/stops
  with the rest of the stack.

## Milestones (suggested order)

1. **Static test**: mount camera, drop the ball on the disk, publish
   `/ball_xy` at 30 Hz from a laptop. Verify coordinates match a ruler.
2. **Latency measurement**: time from LED flash → `/ball_xy` message.
   Budget: 30 ms. Measure before you try to close the loop.
3. **Closed-loop ball-centering** (easier than trajectory tracking):
   level the disk, drop the ball off-center, platform tilts to push it to
   the middle. This is the "ball balancer" classic demo and is a great
   sanity check.
4. **Trajectory tracking**: replace the scripted rolling-ball routine's
   feed-forward trajectory with a reference `(x_ref(t), y_ref(t))` and
   a PID on (ball - ref). Now the demo is robust to perturbations.
5. **Robustness**: add IR filter + retroreflective ball, retune, demo
   with people waving at the camera.

## Things to decide before starting

- Mount the camera on the platform (moves with tilt) or on a fixed
  frame looking down (moves with the world)? Fixed-frame is much easier —
  the disk appears as an ellipse in image space and the homography is
  nearly constant.
- Which coordinate frame does `/ball_xy` live in — image pixels, disk
  mm, or world mm? **Disk mm** makes the PID trivial and lets you tune
  gains without re-thinking units.
- Do you want the ball-tracker node to auto-detect the disk each frame
  (robust to camera drift) or only at startup (simpler, requires rigid
  mount)?

None of these decisions are permanent. Start with the simplest choice in
each category (fixed top-down monocular, disk-mm coordinates, calibrate
at startup) and complicate only when something specifically breaks.
