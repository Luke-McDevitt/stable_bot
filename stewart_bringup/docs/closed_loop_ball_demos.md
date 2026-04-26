# Closed-Loop Ball Demos — Vision-Based Control Plan

**Project:** Stable-Bot (6-DOF Stewart platform, ROS 2 Kilted)
**Scope:** Replace the open-loop rolling-ball demo with a vision-driven
closed loop. Three demos: orbit (Demo 1), click-to-position (Demo 2),
and path-drawing follow (Demo 3, stretch goal).
**Authors:** Luke McDevitt + Claude (Opus 4.7).
**Last revised:** 2026-04-26 (v9.1 — Active Stabilization (Ball-Hold)
mode scaffolded for v10 (§11.7): GUI panel with disabled controls,
mode token `BALL_HOLD` reserved in `ref_generator_node`, feedforward
+ saturation math added to `_ball_physics.py` with unit tests. Full
implementation (controller-side base-IMU feedforward) deferred — see
§16 step 8.5).

This plan supersedes the relevant sections of
`stewart_bringup/docs/NEXT_STEPS.md`. It pulls in lecture material from
EEL 4930/5934 (Lectures 6 — Robot Perception, 8 — Filtering & PID).

---

## 1. Status and goals

- Open-loop rolling-ball demo previously failed the professor's review.
- New requirement: real closed loop using vision.
- Extra-credit: drive the ball to any operator-specified point on the
  platform.
- Hardware has been purchased; bring-up is the work that remains.

The two demos defined below are the minimum viable deliverables. Both
use the same vision pipeline; only the reference generator differs.

---

## 2. Hardware summary

| Component | Detail |
|---|---|
| Camera | Luxonis OAK-D-PRO-AF — stereo + 12 MP color + IR dot projector + Movidius Myriad X VPU. Onboard inference frees the Pi. |
| Camera mount | 5 ft aluminum rod fixed to the **bottom** platform (clear of actuators and the moving top plate), with an articulated camera arm + ball-head mount so the camera angle and height can be adjusted between trials. |
| Ball | **40 mm foam ball, mass-matched to a ping-pong ball.** Solid foam → moment of inertia I = (2/5)·m·R² instead of the hollow-sphere (2/3)·m·R². See §9 for the control implication. |
| Platform | 400 mm-diameter top plate; IMU bolt protrudes through the geometric center. |
| ArUco ring | Pre-printed via `generate_aruco_ring.py` (DICT_4X4_50, 8 markers, 50 mm side, ring radius 120 mm). `marker_layout.yaml` accompanies the print. |
| IMU | Movella MTi-630 AHRS, mounted parallel to the top plate; `platform_level.yaml` references roll/pitch zero to the IMU's factory-calibrated frame (see the MTi reference-frame discussion in chat history dated 2026-04-25). |
| Compute | **Raspberry Pi 5, 16 GB RAM, ROS 2 Kilted.** Phase 8 migration is complete (2026-04-26). Two systemd services auto-start the stack on power-up: `stable_bot.service` (ROS 2 nodes) + `stable_bot_gui.service` (gui_server.py + rosbridge). Pi 5 USB-C is power-only — OAK plugs into a **blue USB-A 3.0** port (dedicated PCIe-fed bandwidth, no Pi 4 shared-controller bottleneck). |
| Network | Laptop ↔ Pi over the user's Pixel 9 phone hotspot (same LAN). Hostname `stablebot`; current IP is hotspot-DHCP-assigned (look up in the hotspot's "Connected devices"). GUI is at `http://<pi-ip>:8080/`; rosbridge websocket at `ws://<pi-ip>:9090/`. SSH user: `sorak`. |
| Bring-up laptop | Used only to host the browser. WSL2 Ubuntu 24.04 retained for development + GitHub flow. |

The articulated mount is a deliberate choice — the camera angle WILL be
re-tuned during bring-up. **Every camera move invalidates the
extrinsics**; intrinsics survive. The calibration workflow in §6 is
designed around this.

---

## 3. The two demos

### Demo 1 — Orbit at fixed radius (closed loop)

The ball orbits the platform center at a user-specified radius and
period. The demo starts from wherever the ball happens to be sitting:
the controller first drives it to the orbit radius (re-using Demo-2's
goto law), then engages the orbit reference. No operator interaction
during the run beyond Start / Stop.

- Acquisition phase: Demo-2 goto law to (R, 0).
- Orbit phase reference:
  `(x*(t), y*(t)) = (R·cos(2πt/T + φ), R·sin(2πt/T + φ))`.
- R is bounded below by the dead-zone (§10) and above by
  `platform_radius − ball_radius − margin`.
- T (period), direction, and R are GUI-selectable.
- Termination: Stop button, ball lost > 500 ms, or platform fault.

### Demo 2 — Click-to-position (extra credit, primary deliverable)

The ball sits stationary at its current location until the operator
clicks a different point on the GUI's graphical disk view; the
controller then drives the ball to that point and holds.

GUI for Demo 2 is **a graphical (SVG) representation of the platform**,
not a camera feed. It displays:

- The platform circle with the ArUco ring drawn at r = 120 mm.
- The dead-zone disk (red, see §10) at the center.
- The current ball position (filled dot) updated from real vision data.
- The current ball reference (target marker, hollow ring) — clicking
  inside the platform-but-outside-dead-zone publishes a new reference.
- A z-height slider (commands pure heave on the Stewart top plate; no
  tilt change; range 0 to 80 mm above neutral pose).
- **Predicted-path overlay** (toggled by checkbox, default off): a
  dashed line from current ball position to the target.
- **Start / Stop buttons** (mandatory safety). A second click while in
  flight cancels the current target and retargets immediately.

The live stereo camera feed remains available in a separate "Vision
Debug" panel (§11) but is not the primary view for Demo 2.

### Demo 3 — Path drawing & follow (stretch goal)

Operator drags the cursor across the SVG disk to draw a freehand path;
the ball is driven first to the path's start (Demo-2 goto law) and then
follows the drawn polyline at a configurable traversal speed. Optional
"loop" checkbox closes the path with a smooth join from end to start
and repeats indefinitely until Stop.

- Path captured as a sampled polyline (one (x_mm, y_mm) point every
  ~30 ms during draw), then smoothed (e.g. 5-point moving average) and
  resampled at constant arc-length spacing.
- Reference generator parameterizes the smoothed polyline by time at
  the operator-set traversal speed (default 80 mm/s, slider).
- Loop mode: closes path with a cubic-spline join end→start, modulo
  traversal.
- Pre-flight rejection: if any path sample falls inside the dead-zone
  or off the platform edge, the path is rejected with a UI flash —
  user must redraw. (Auto-clipping was considered but introduces
  surprise: a redrawn path is more honest than a silently-mutated one.)
- **Start / Stop buttons** (mandatory). Stop drops to `LEVEL_HOLD`.
- Predicted-path checkbox shows the upcoming traversal direction as a
  faint arrow chain along the polyline.

This is gated on Demos 1 and 2 working cleanly. Specification details
that need confirmation are in §15 Q11–Q14.

### Why this order

Demo 1 stresses the loop's ability to track time-varying setpoints
(differentiator, feed-forward). Demo 2 stresses settling and the safety
machinery for arbitrary targets. Demo 3 combines both. All three share
everything below the reference generator; debugging is additive.

---

## 4. Coordinate frames

We will publish the ball position in **both** frames; subscribers pick
what they need.

| Frame | Symbol | Origin | Axes | When to use |
|---|---|---|---|---|
| World | `W` | Bottom platform center, gravity-aligned | +z up (gravity-anti-parallel), +x toward the operator | Camera extrinsics, GUI graphical view, planning |
| Platform | `P` | Top plate center, rotates+translates with the plate | +z normal to plate, +x along the IMU's reported +x | Controller error computation, ball dynamics |
| Camera | `C` | OAK-D left optical center | OpenCV convention (+z forward, +x right, +y down) | Vision math only |
| ArUco board | `B` | Center of the marker ring on the plate (= platform origin by design) | Same as P | Pose link from C to P |

Transform chain: `C → B (= P) → W` via the ArUco-board pose every frame,
plus a one-shot `C → W` from a known-position calibration marker laid
on the leveled platform.

---

## 5. Vision stack architecture (new ROS 2 package)

`stewart_vision` is a **separate** package from `stewart_bringup`. This
keeps the controller portable for users who want to swap in a different
vision stack (e.g. external mocap, RealSense, mono webcam). Topics are
the contract; node implementations are interchangeable.

```
stewart_vision/
  stewart_vision/
    oak_driver_node.py       # DepthAI pipeline, raw + on-VPU detections
    rectifier_node.py        # publishes rectified pairs on demand (GUI)
    platform_pose_node.py    # ArUco board → /platform_pose @ 30 Hz
    ball_localizer_node.py   # mono-projection AND triangulation paths
    ball_kf_node.py          # constant-vel KF (or EKF) → /ball_xy @ 100 Hz
    ref_generator_node.py    # /ball_ref from orbit OR click-to-goto
  scripts/
    calibrate_oak.py         # one-shot intrinsics + stereo + extrinsics
    aruco_pose_demo.py       # standalone ArUco-board sanity tool
    sfm_demo.py              # SIFT→F→E→R,t pipeline (Lecture 6)
    epipolar_overlay.py      # rectification verification, drawn on pairs
  config/
    marker_layout.yaml       # produced by generate_aruco_ring.py
    oak_intrinsics.yaml      # produced by calibrate_oak.py
    oak_extrinsics.yaml      # produced by calibrate_oak.py (re-run on every camera move)
    ball_safety.yaml         # dead-zone + margins + loss timeout
  launch/
    stewart_vision_launch.py
  docs/
    closed_loop_ball_demos.md   # this file (final home)
```

`stewart_control_node` (in `stewart_bringup`) gains a `BALL_TRACK` mode
that subscribes to `/ball_state` + `/ball_ref` and runs the controller
described in §9. No other changes to `stewart_bringup` are required.

---

## 6. Camera calibration workflow

Calibration runs **on the Pi** (where the OAK is plugged in). The
user-facing flow is **split by audience** (locked v7, 2026-04-26):

- **GUI button — Stage C only.** The laptop's GUI exposes capture /
  solve / reset for the camera-to-world ArUco-ring step only. This
  is the operation that matters between demo runs (every camera arm
  move re-runs it) and is the only one a non-developer operator
  ever needs.

- **CLI only — `--stage factory`, `--stage A`, `--stage B`.** Reading
  factory cal from EEPROM and any custom chessboard recalibration
  happen only via SSH + `python3 calibrate_oak.py` on the Pi.
  Reasoning: A and B *override* the OAK's factory-tuned intrinsics
  and stereo extrinsics. A misclick by an operator could replace a
  working factory cal with a half-baked custom one and silently
  degrade the controller. Gating these behind the CLI keeps them
  available to advanced users without surfacing the foot-gun.

The `stewart_vision` calibration ROS node therefore implements ONLY
Stage C service handlers:

  - `/calibrate/capture_frame` (Trigger) — grab one frame of the
    leveled platform's ArUco ring; auto-detects the board and
    returns success ("ArUco ring detected, all 8 markers visible")
    or failure ("only 5 markers visible — check lighting and angle").
  - `/calibrate/solve` (Trigger) — `cv2.aruco.estimatePoseBoard`
    over the accepted frames; writes `oak_extrinsics.yaml` if the
    reprojection gate passes.
  - `/calibrate/reset` (Trigger) — clears the captured-frame buffer.

There is **no** `/calibrate/stage` selector exposed via ROS — Stages
A/B literally cannot be triggered from the GUI even if someone
malicious typed the right service name. They only run when the
calibrate_oak.py CLI is invoked on the Pi with the explicit flag.

The GUI's Vision Debug panel shows the live OAK feed with detected
ArUco markers drawn in real time, plus the three buttons above.
The marker geometry comes from `marker_layout.yaml` produced by
`generate_aruco_ring.py` so reprinting the ring doesn't require code
edits.

`scripts/calibrate_oak.py` runs in three independent stages so that
moving the camera only invalidates the cheap stage. **Stages A and B
are optional now (v6 default):** the OAK ships with factory-calibrated
intrinsics and stereo extrinsics stored on its EEPROM, accessible via
`Device.readCalibration()`. The default flow uses those directly:

```bash
# Default — write factory cal into oak_intrinsics.yaml in seconds
python3 scripts/calibrate_oak.py --stage factory
```

Re-doing Stages A/B manually only buys you a slightly tighter
comparison-vs-truth baseline (your own chessboard captures may
out-perform the factory <0.3 px reprojection target by a small
margin). For the demo, use factory; document this in the report.

Stage C (camera-to-world via the ArUco ring) is **always** done
ourselves — the OAK has no idea where the platform is.

### Stage A — Intrinsics, per eye (slow; do once per camera, ever)

1. Print a 9×6 chessboard, 25 mm squares, mount on a rigid flat backer.
2. Capture ~30 left+right pairs at varied angles and distances.
3. Run `cv2.calibrateCamera` per eye → K_L, K_R, dist_L, dist_R.
4. Acceptance: per-eye reprojection error < 0.3 px. Else recapture.

Outputs: `oak_intrinsics.yaml` (K, dist for each eye). These are the
camera's permanent properties; Stage A is rarely repeated.

### Stage B — Stereo extrinsics (do once per camera, ever)

1. Same capture set as Stage A.
2. `cv2.stereoCalibrate(..., flags=CALIB_FIX_INTRINSIC)` → R_LR, t_LR.
3. `cv2.stereoRectify` → projection matrices P_L, P_R; rectification
   maps cached to disk.
4. Acceptance: mean epipolar error after rectification < 1 px.
5. Render an HTML report (`calibration_report.html`) showing rectified
   pairs with horizontal epipolar lines drawn on. This is the artifact
   the professor sees.

Outputs: appended to `oak_intrinsics.yaml`. Also rarely repeated.

### Stage C — Camera-to-world extrinsics (re-run after every camera move)

The articulated camera arm has no precise positioning, so this stage
does *not* rely on a known camera pose. Instead it recovers the
camera's pose from the ArUco ring itself, which is rigidly attached to
the top plate at known geometry (8 markers, 50 mm side, 120 mm ring
radius, equal angular spacing — `marker_layout.yaml`). That gives 32
known 3-D corner positions in the platform frame.

1. Manually level the platform: command zero tilt, confirm IMU at
   0/0 ± 0.1° (matches §2 of the IMU discussion in chat history).
2. Capture one frame with the full ArUco ring in view of the left eye.
3. Run `cv2.aruco.estimatePoseBoard` (or equivalently `solvePnP` over
   all 32 corners) using `marker_layout.yaml` for the 3-D positions
   and K_L, dist_L from Stage A for the camera intrinsics.
4. Because the platform is leveled and the ArUco ring is centered on
   the plate (which sits over the world origin), the recovered
   transform IS T_cam_world directly — no offset composition needed.
5. Acceptance: project all 32 ring corners back into the image; mean
   reprojection error < 2 px. Below 1 px is the goal.

Output: `oak_extrinsics.yaml`. This file goes stale every time the
camera arm moves. The calibrate script auto-detects whether Stage A/B
outputs already exist and skips them — so re-extrinsics is ~30 seconds
of work.

**Why this works without preset support:** the ArUco ring's known
geometry over-determines the pose problem (32 correspondences vs 6 DOF),
so OpenCV's solver converges robustly from any camera angle that sees
≥ 3 markers. Camera height, tilt, and azimuth are all recovered from
the marker warping in a single frame. Mount the arm wherever; the math
figures out the rest.

### Calibration sanity gate

`calibrate_oak.py` refuses to write its YAML if any acceptance metric
fails, and prints which stage failed. Mis-calibrated extrinsics are the
single most common cause of "the ball is in the wrong place" bugs;
gating prevents bad data from ever entering the pipeline.

---

## 7. Stereo / vision techniques from Lecture 6

**Strategy (locked in v5, 2026-04-26):** the OAK's onboard stereo depth
is treated as **ground truth**, and the controller closes the loop on
that. The lecture techniques (homography, F/E matrices, 8-point +
RANSAC, triangulation, SfM) all run **in parallel as comparison
estimators only** — they have no control authority. Each technique
writes its output to a dedicated topic, and a comparison artifact
(JSON + the per-demo HTML report from §13.5) shows agreement /
disagreement between the OAK truth and each classical method.

Why: this is the highest-probability path to a working demo while
still showcasing every Lecture 6 technique. If a classical estimator
has a bad frame, the controller is unaffected (it's looking at the
OAK truth). If a classical estimator agrees with the OAK to within a
few mm across the demo, that's a *strong* "I understand this" claim
to the professor — a single-frame disagreement plot is worth more
than a flawless real-time estimator that anyone can run from a
DepthAI tutorial.

Each item below is implemented explicitly somewhere in the pipeline.

| Lecture topic | Implementation in this project |
|---|---|
| Pinhole model + intrinsic K (slides 13–15) | `oak_intrinsics.yaml`; the visualization helper draws K's principal point + focal-length cones. |
| Projection matrix P = K[R\|t] (slide 16) | Used in mono ball-pixel-to-platform-plane projection (`ball_localizer_node.py`). |
| Homography + 4-point + SVD (slides 17–22) | ArUco-board pose recovery (each marker → 4 correspondences). Also used internally by `cv2.stereoRectify`. |
| Checkerboard calibration (slides 23–25) | Stage A above. Calibration report shows the chessboard reprojection. |
| Stereo baseline + disparity (slides 26–27) | StereoSGBM disparity map computed on the OAK; published at low rate; rendered in the GUI debug pane. |
| Stereo rectification (slide 28) | Stage B; epipolar overlay in the report. |
| Epipolar geometry / F & E (slides 29–33) | `epipolar_overlay.py` draws horizontal epipolar lines on rectified pairs as a visual proof. |
| 8-point + RANSAC (slides 34–36) | `sfm_demo.py` recovers F from feature matches and confirms it agrees with factory-calibrated F (within RANSAC noise). |
| Triangulation (slide 40) | **Primary stereo ball estimator** — DLT triangulation of (uL, uR) → (X, Y, Z)_C for `ball_xy_stereo`. |
| Cheirality + R,t from E (slides 38–39) | Done in `sfm_demo.py`. |
| SfM pipeline (slide 41–42) | `sfm_demo.py`: pan camera by hand, recover camera trajectory via SIFT/ORB → F → E → triangulation. Standalone demo for the slides. |

Three ball-localization paths run in parallel; only `/ball_xy_oak` has
control authority:

1. **OAK onboard depth (truth)** (`/ball_xy_oak`) — ball pixel + the
   OAK's StereoDepth output sampled at the ball ROI via DepthAI's
   `SpatialLocationCalculator`. This is what `ball_kf_node` consumes
   and what the controller closes on.
2. **Mono + plane projection (classical)** (`/ball_xy_mono`) — ball
   pixel + ArUco plane → ray-plane intersection. Pure Lecture 6 math
   (no stereo). Logged for comparison.
3. **Stereo triangulation (classical)** (`/ball_xy_stereo`) — rectified
   pair → ball pixel in each → DLT triangulation → into platform frame.
   Uses our own intrinsics + 8-point fundamental matrix verification.
   Logged for comparison.

`/ball_xy_oak` ↔ `/ball_xy_mono` ↔ `/ball_xy_stereo` agreement is
written to `/method_comparison` (custom JSON-encoded String at 30 Hz)
and rolled up post-run by the plotter into the comparison section
of the demo's HTML report (§13.5). Per-frame error vs OAK truth
(mm), Bland-Altman plots, and a histogram of disagreements per
method.

### Other OAK-on-chip computations we shadow with Lecture-6 techniques

Beyond the ball-position estimators above, the OAK does several other
things on-chip that we can replicate manually with classical methods.
Each goes in the comparison artifact as another "we built it ourselves
even though the camera does it" line. Recommended tier indicates
implementation cost vs Lecture-6 demonstration value.

| OAK on-chip | Manual equivalent | Recommended? |
|---|---|---|
| Stereo rectification (factory maps) | `cv2.stereoRectify` from our (or the factory's) K + R + T → our own rectification maps; render both left/right pairs side-by-side with horizontal epipolar lines drawn (Lecture 6 slide 28) | **Yes** — one-shot, big visual proof, ~50 lines |
| Disparity (SGBM on Myriad X, real-time) | `cv2.StereoSGBM_create` on the host applied to the OAK's rectified pair; per-pixel difference heatmap | **Yes** — a debug pane in the GUI is a great slide |
| Depth from disparity (`Q` matrix on-chip) | `depth = (f · baseline) / disparity` per-pixel using our own Q from `cv2.stereoRectify`; agreement scatter | Optional — implicit in disparity comparison |
| 3-D point at ROI (`SpatialLocationCalculator`) | Sample our own depth map at the ball pixel, project via `K^-1 · pixel · depth`; this is `/ball_xy_stereo` already | **Already in the plan** |
| Camera intrinsics (factory K, dist) | Stage-A chessboard `cv2.calibrateCamera`; compare K element-wise + reprojection error histograms | Optional (skipped for v6 default); only do if Stage A captures happen for some other reason |
| Stereo extrinsics (factory R, t) | `cv2.stereoCalibrate` after our own intrinsics; compare baseline length and rotation Euler angles | Optional; tied to Stage B |

**Fallback story (extreme failure scenarios).** If the OAK's *depth*
fails for some reason but raw frames keep flowing (e.g. stereo
mismatch in heavy IR contamination), the classical mono ray-plane
estimator (`/ball_xy_mono`) can take over control authority — same
node already runs, just toggle which topic feeds `ball_kf_node`.
The ArUco-ring pose recovery is independent of any depth path, so
that survives. If the OAK *device* dies (USB drop, firmware lock-up),
nothing recovers — the watchdog drops to LEVEL_HOLD; the operator
power-cycles the OAK.

---

## 8. Ball detection and tracking

### Detection — onboard the OAK VPU, two detectors in parallel

Both detectors run simultaneously from day one (decision: 2026-04-25).
Running them in parallel gives us cross-checking diagnostics and
graceful degradation if either fails in some lighting condition.

1. **Color threshold (V0)** — saturated-orange foam ball → HSV →
   threshold → biggest contour → centroid + radius. ~1 ms on the
   Myriad X. Zero training. Strong baseline given a bright orange ball
   on a black-and-white platform; light invariance is its main weakness.
2. **YOLOv5-nano (V1)** — runs on the same Myriad X VPU at >30 fps.
   Trained on ~600–700 hand-labeled frames captured during a 5–6
   minute free-roll session. `scripts/capture_training_frames.py`
   bags color frames at 2 Hz with timestamps and writes them as
   numbered JPEGs into a dataset directory; labels are drawn
   afterward in LabelImg, Roboflow, CVAT, or any YOLO-compatible
   tool. YOLOv5-nano is the natural fit: pre-quantized OpenVINO
   models exist; minimal training compute (a CPU-only train run
   takes ~30 min for this dataset size). Larger dataset = better
   generalization to varied lighting, glare from the IR projector,
   and partial occlusion by the operator's hand.

Both detectors publish independently:

- `/oak/ball/v0/left_pixel`,  `/oak/ball/v0/right_pixel`  (color)
- `/oak/ball/v1/left_pixel`,  `/oak/ball/v1/right_pixel`  (YOLO)

`ball_localizer_node` subscribes to both. Selection rule:

1. If both report a detection within 5 px and confidence ≥ 0.8 each,
   use the average → highest-quality measurement.
2. Else if exactly one reports confidence ≥ 0.8, use it.
3. Else if both report confidence < 0.8 but their detections agree
   within 10 px, use the average (low-confidence consensus).
4. Else flag "ball lost" — KF predicts forward; if no acceptable
   detection for > 500 ms, drop to LEVEL_HOLD.

Each frame's per-detector position + the chosen-output is published
on `/oak/ball/diagnostic` so the GUI's vision-debug strip shows live
agreement / disagreement, telling you which detector is winning under
the current lighting. This is itself a strong "we did the work" slide.

The Pi never decodes raw frames for tracking — only for the GUI live
feed (compressed, 15 fps).

### Tracking — Kalman filter on the Pi

`ball_kf_node.py` runs a **constant-velocity KF** with state
`x = [px, py, vx, vy]` (platform frame). This is exactly the
bounding-box tracker derivation from Lecture 8 slides 32–34.

- Predict (100 Hz, fast loop): `x ← F·x; P ← F·P·Fᵀ + Q`.
- Update (~60 Hz, when measurement arrives):
  innovation `y = z − H·x; S = H·P·Hᵀ + R; K = P·Hᵀ·S⁻¹`.
- Q tuned from rolling-friction acceleration variance.
- R from empirical projection noise (estimate during calibration: park
  the ball, log 200 measurements, fit a 2-D Gaussian).

Optional **EKF** upgrade (Lecture 8 slides 35–37) — fold the tilted-plate
dynamics directly into the motion model:
`ẍ = (5/7)·g·sin(θ_x)` (foam, solid sphere; see §9).
This makes Q smaller and predictions accurate during sensor dropouts.
Worth doing if Demo 2 settling is choppy with the constant-velocity KF.

The KF posterior `(px, py, vx, vy, P)` is published on `/ball_state`.
The controller subscribes to that — never to raw pixels.

---

## 9. Controller — PID with foam-ball feedforward

Ball-on-plate is a **double integrator** in each axis: tilt commands
acceleration, position is the double-integral of acceleration.

- Pure P feedback on position is marginally stable.
- PI feedback is *unstable* (a third integrator in the loop).
- **PID** (P with derivative damping) is the minimum stable architecture.
- Adding a small I term compensates rolling-friction stiction at low
  speed.

### Plant equation (rolling ball)

For a sphere on a tilted plane, neglecting slip:

```
ẍ = (g · sin θ_x) / (1 + I/(m·R²))
```

The ratio `I/(m·R²)` differs by ball type:

| Ball | I/(m·R²) | ẍ as fraction of g·sin θ |
|---|---|---|
| Hollow ping-pong (thin shell) | 2/3 | 3/5 = 0.600 |
| Solid foam (uniform density) | 2/5 | 5/7 ≈ 0.714 |

Foam accelerates ~19 % faster than ping-pong at the same tilt.

### Foam vs ping-pong — control trade-off

You purchased a 40 mm foam ball mass-matched to a ping-pong ball for
its uniform mass distribution. From a controls standpoint:

- **Ping-pong (hollow)** — slower response → more time for the PID to
  react → easier to tune, more forgiving gain windows. But hollow +
  light makes it perturbed by air currents (fan, breath), and it dents
  on impact.
- **Foam (solid)** — faster response → tighter timing requirement,
  needs more aggressive Kd. But mass distribution is uniform (no
  off-center weight from a manufacturing seam), more durable, less
  air-perturbed, repeatable across thousands of runs.

**Recommendation:** keep both. Use the **ping-pong** for initial PID
tuning (gentler dynamics → faster convergence to stable gains), then
swap to the **foam** for the final demo (more visually crisp orbital
motion, more reliable repeatability). Same vision pipeline either way
(both 40 mm, both bright orange).

The `ball_safety.yaml` file (§10) carries a `ball_density` field —
`solid` or `hollow` — so swapping balls just changes one config line
and the feedforward coefficient updates automatically.

### Control law (per axis, x and y identical)

```
e   = x* − px
edot= ẋ* − vx               // KF gives vx
α   = 5/7 if solid else 3/5  // from ball_safety.yaml
θ_x = Kp·e + Kd·edot + Ki·∫e + (1/(α·g))·ẍ*    // last term is feedforward
```

Tune order: Kp first (Kd=Ki=0, watch oscillation), add Kd to damp, add
small Ki for stiction. Then verify the feedforward term reduces
tracking error at high orbital speeds (Demo 1 is the test case).

### Output handling

- Saturate `θ_x, θ_y` to whatever the platform achieves smoothly,
  enforced by the existing `global_limits.yaml`.
- Slew-rate limit on tilt commands (~30°/s) to prevent the Stewart
  legs from chattering.
- Roll/pitch only — never yaw — because the ArUco ring rejects yaw
  estimation noise more cleanly anyway.

### Z-height handling

The Stewart platform can translate the top plate vertically as well as
tilt it. Demo 2's z-height slider commands a constant z-offset added to
all six leg lengths (no tilt, just up/down). This is independent of the
ball-tracking PID, but the controller must be told the current
plate-z so the ArUco-pose-based ball projection uses the right plane
height. **Confirm this is the intended slider behavior** — see §15 Q1.

---

## 10. Safety dead-zone (10 mm rule)

Constraint: the ball's surface must stay ≥ 10 mm from the platform
geometric center to clear the IMU bolt.

Ball radius = 20 mm (40 mm foam ball) → ball *center* must stay at
radius ≥ 30 mm from platform center.

`config/ball_safety.yaml`:

```yaml
dead_zone_mm: 10        # ball-edge to platform-center clearance
ball_radius_mm: 20      # solid foam, 40 mm dia
ball_density: solid     # affects feedforward (5/7 vs 3/5 of g)
margin_mm: 5            # soft-repulsion starts here outside dead-zone
loss_timeout_ms: 500    # fall back to level if ball not detected
```

Three independent enforcement layers (matches the existing
`global_limits.yaml` redundancy philosophy):

1. **Reference generator** — any reference point with
   `√(x*² + y*²) < dead_zone + ball_radius` is rejected:
   - Demo 1: orbit radius silently clamped to ≥ 35 mm.
   - Demo 2: clicks inside the red disk flash and are ignored.
   - Demo 3: any path sample inside the dead-zone fails the
     pre-flight check and the entire path is rejected (operator must
     redraw — silent clipping was considered and rejected; a redrawn
     path is more honest than a silently-mutated one).
2. **Controller** — if the *measured* ball is within
   `dead_zone + ball_radius + margin` of center, add a soft repulsive
   tilt term that accelerates the ball outward. This kicks in before
   the ball can reach the IMU bolt and continues even if the reference
   would otherwise drive it inward.
3. **Watchdog — center incursion** — if the measured ball radius falls
   below `dead_zone + ball_radius` for > 100 ms, command level + slow
   z-down and fault out. Recovery requires operator clear via GUI.
4. **Watchdog — off-platform** — if the measured ball position has
   `√(x² + y²) > 200 mm` (i.e. outside the platform's physical radius)
   *or* both detectors report ball-lost for > 500 ms, abort to
   `LEVEL_HOLD` immediately. **This is the only condition that
   triggers the strobing red ball-lost UI flash (§11.5)** — transient
   in-bounds detection dropouts (e.g. the operator's hand briefly
   shadowing the ball) are recovered silently by the controller.
   Re-arming is **manual**: the operator must press Start again after
   the ball is back inside a 180 mm hysteresis radius and detected
   with confidence ≥ 0.8 for 1 s. Manual is intentional — automatic
   re-arm during a "hand still on the platform" moment would be
   surprising.
5. **Watchdog — saturation (BALL_HOLD only, v10)** — when active
   stabilization (§11.7) is engaged, the controller computes
   `saturation = |commanded_tilt| / max_safe_tilt` per axis. If
   saturation ≥ 0.95 sustained for > 100 ms, drop from `BALL_HOLD`
   → `LEVEL_HOLD` with a toast. Below that, the GUI surfaces
   saturation in the Ball-Hold panel (green / yellow / red). This
   prevents the controller from demanding tilt the platform can't
   deliver and going unstable. Math: `_ball_physics.saturation_fraction`.

The graphical disk in the Demo-2 / Demo-3 GUI draws the dead-zone in
semi-opaque red so the operator can't accidentally click or draw into
it.

---

## 11. GUI design

The GUI remains a single `web/index.html` Tailwind + roslibjs page
served by `gui_server.py`. The page now uses a **three-column layout**
to keep all live information visible at once.

### 11.0 Three-column layout (already in `web/index.html`)

The current `web/index.html` is already three-column on `lg:` breakpoint
and above (`<main class="grid lg:grid-cols-3">` with three
`<section class="lg:col-span-1">` children). On screens narrower than
1280 px the columns stack vertically. The vision-driven demos add
panels into the existing layout rather than restructuring it:

```
┌──────────────┬──────────────────────────────┬──────────────┐
│   LEFT       │           MIDDLE              │   RIGHT      │
│              │                               │              │
│  + Vision    │  + Demo 1 (Orbit)             │  Homing      │
│    Debug     │  + Demo 2 (Goto)              │  Jog         │
│  + Calib     │  + Demo 3 (Path) [stretch]    │  Level       │
│    Stage C   │   each with Start / Stop +    │  Reset stack │
│    only      │   graphical SVG disk view     │  (existing)  │
│              │                               │              │
│  Encoder &   │  (Open-loop "rolling ball"    │              │
│  current     │   panel hidden by default     │              │
│  debug       │   when vision is active)      │              │
│  (existing)  │                               │              │
└──────────────┴──────────────────────────────┴──────────────┘
```

`+` = NEW panels added by the vision work. Existing panels keep
working unchanged.

### 11.1 Left column — Vision Debug + existing diagnostics

**Vision Debug** (top of left column, collapsible):

- Live left-eye feed (compressed, 15 fps, `<canvas>` from a
  `sensor_msgs/CompressedImage` topic).
- Live right-eye feed, stacked under or beside the left depending on
  column width.
- Toggle: "Rectified" overlays horizontal epipolar lines (visual proof
  of Lecture 6 slide 28).
- Toggle: "Disparity" — replaces feeds with the StereoSGBM color-mapped
  disparity map at low rate.
- Telemetry strip:
  - ArUco markers visible (n / 8)
  - Tracker confidence (KF NIS, color-coded)
  - End-to-end latency (`stamp_now − stamp_capture`)
  - **Method comparison** (live, mm): OAK-vs-mono, OAK-vs-stereo,
    mono-vs-stereo discrepancies. Color-coded:
    - **green** when |error| < 5 mm
    - **yellow** when 5 mm ≤ |error| < 15 mm
    - **red** when |error| ≥ 15 mm

**Calibration controls** (collapsible sub-panel, hidden during demos):

This panel exposes **Stage C only** — the ArUco-ring camera-to-world
step that's re-run after every camera arm move. Stages A/B are
CLI-only by design (see §6).

- Live OAK left-eye feed with detected ArUco-ring corners drawn in
  real time (so the user can see whether the current pose has all
  8 markers visible before pressing Capture).
- Frame counter "N captured" — typically 1–3 frames is enough for
  Stage C with a well-leveled platform; the solver accepts a
  single frame as long as ≥ 3 markers are detected.
- Three buttons (only Stage C operations exposed via ROS):
  - **Capture frame** — calls `/calibrate/capture_frame`. Result
    toast: "all 8 markers detected" / "only N/8 markers visible —
    rotate camera or improve lighting".
  - **Solve** — calls `/calibrate/solve`. Runs
    `cv2.aruco.estimatePoseBoard` over the accepted frames; writes
    `oak_extrinsics.yaml` if mean reprojection error < 2 px.
  - **Reset** — clears the buffer.

The GUI does **not** offer a stage selector. Operators cannot
override the OAK's factory intrinsics or stereo extrinsics from
the browser — those are protected behind the CLI flow on the Pi
to prevent accidental damage to a working calibration.

**Encoder & current debug** (bottom of left column): the existing
plots and read-outs, unchanged.

### 11.2 Middle column — Demo panels

Each demo panel contains its own SVG disk view (so the operator never
has to scroll between controls and the visualization), plus mode-
specific controls. Only one demo can be active at a time; activating
one disables the others' Start buttons.

#### 11.2.1 Demo 1 panel — Orbit

- Orbit radius slider (35 mm to platform_radius − ball_radius − 5).
- Orbit period slider (1 to 10 s).
- Direction toggle (CW / CCW).
- **Reverse Direction** button — visually styled like the UNO "reverse"
  card: two curved arrows chasing each other in opposite directions,
  bright yellow background. Pressing it flips the orbit direction
  *while running* — the reference reverses instantly and the
  controller commands the appropriate tilt to decelerate then
  accelerate the ball the other way (great visual stress test of the
  loop's responsiveness; matches lecture-8 PID material). Disabled
  when not orbiting.
- **Start** button: publishes mode change to `BALL_TRACK_TRAJECTORY`;
  the controller first runs the goto law to drive the ball to (R, 0),
  then engages the orbit reference.
- **Stop** button: drops to `LEVEL_HOLD`.
- Mini-SVG disk: shows reference orbit circle + measured ball trail.

#### 11.2.2 Demo 2 panel — Click-to-position

A square `<svg>` showing a top-down platform representation:

- Outer circle: platform_radius (200 mm, drawn 1:1 in mm at the user's
  zoom).
- Dotted circle at 120 mm: ArUco ring (with the 8 marker squares drawn
  to scale, labeled 0–7 to match `marker_layout.yaml`).
- Solid red filled disk: dead-zone (no-go region for both clicks and
  ball).
- Filled blue dot: current ball position from `/ball_xy` (platform
  frame).
- Hollow blue ring: current ball reference from `/ball_ref` (only
  drawn while in BALL_TRACK_GOTO mode).
- **Predicted-path overlay**: dashed line from current ball position
  to target, controlled by a "Show predicted path" checkbox (default
  off).
- Faint trail: last 2 seconds of ball positions (debug toggle).

Click handler: any click inside the platform-but-outside-dead-zone
publishes a new `/ball_ref` and switches mode to `BALL_TRACK_GOTO`.
A second click while a goto is in flight cancels the current target
and retargets immediately. Clicks inside the dead-zone flash the red
region and ignore the input.

Auxiliary controls:

- **Start** button: arms goto mode (clicks become live targets).
  Without Start armed, clicks are previewed on the SVG but do not
  publish.
- **Stop** button: clears `/ball_ref`, drops to `LEVEL_HOLD`,
  disarms goto.
- Z-height slider: commands pure heave on the Stewart top plate (no
  tilt change). Range 0 to 80 mm above neutral pose. Updates take
  effect immediately while the demo is running.
- "Cancel target" button: clears `/ball_ref` without disarming Start.
- Mode indicator: LEVEL_HOLD / BALL_TRACK_TRAJECTORY / BALL_TRACK_GOTO
  / BALL_TRACK_PATH.

#### 11.2.3 Demo 3 panel — Path drawing & follow (stretch)

Same SVG disk as Demo 2, plus:

- **Draw mode** toggle: while on, mouse drag draws a freehand path
  (one (x_mm, y_mm) sample every ~30 ms during drag).
- Path display modes (toggle inside the panel):
  - **(b) Trail-erase mode (default)** — the segment behind the ball
    greys out as the ball passes, giving an unmistakable "you are
    here" cue along the polyline. Easiest to read at a glance.
  - **(a) Static green** — entire polyline stays bright green for the
    whole traversal, no progress indication.
- The drawn path renders as a green polyline during draw; segments
  that pass through the dead-zone or off-platform render in red so the
  operator sees the violation immediately.
- **Loop** checkbox: closes the path with a cubic-spline join end→start
  and traverses repeatedly.
- Traversal speed slider (10 to 200 mm/s, default 80) — operator-set,
  not automatic.
- **Predicted direction** checkbox: shows arrows along the polyline
  indicating traversal direction (faint when off-current-segment,
  bright on the current segment).
- **Reverse Direction** button (UNO-reverse style, same yellow
  curved-arrows look as Demo 1) — flips traversal direction at the
  ball's *current* path-position (no jump back to the start). With
  Loop on, the looping closure direction also flips. With Loop off,
  reaching either endpoint of the polyline ends the run and drops to
  LEVEL_HOLD. The trail-erase state resets at each reverse so the
  display reflects "where the ball has gone since the last reverse"
  rather than carrying stale greying. Disabled when not running.
- **Clear** button: erases the drawn path.
- **Start** button: validates the path against safety rules (any red
  segment → reject with toast), drives the ball to the path's start
  via goto law, then engages `BALL_TRACK_PATH` mode.
- **Stop** button: drops to `LEVEL_HOLD`. The drawn path is preserved
  so Start can be pressed again without redrawing.

##### Path persistence

- **Browser localStorage** with named save/load slots (e.g. "figure
  eight", "spiral in", "demo day path"). Slots persist across page
  reloads and across browser sessions on the same machine.
- **Export to file** — a "Download .json" button serializes the
  active path to a JSON file:
  ```
  {
    "schema_version": 1,
    "frame": "platform",
    "units": "mm",
    "loop": false,
    "speed_mm_s": 80,
    "samples": [[x0, y0], [x1, y1], ...]
  }
  ```
- **Import from file** — an "Upload .json" button reads the same
  format and loads it as the active path. The schema_version field
  lets future format changes coexist with old files.

##### Disturbance recovery

If the ball is bumped off the path mid-traversal, the controller's
behavior depends on where the ball ends up:

- **Ball still on platform** (within 200 mm of center): retarget to
  the closest point on the polyline and continue from there. No abort.
- **Ball off platform** (radius > 200 mm) or detected lost > 500 ms:
  abort to `LEVEL_HOLD`. This is the "operator picked it up to do
  something else" path. Re-arm via the path's Start button when ready.

### 11.3 Right column — Existing controls

The right column hosts the existing operator controls unchanged:

- Homing routine
- Jog leg
- Level platform
- Reset stack (the gui_server.py HTTP endpoint, useful when rosbridge
  is unreachable)
- Open-loop "Demo: rolling ball" panel — kept for fallback /
  regression testing only; hidden by default when vision is active,
  expandable from a small toggle.

### 11.4 Cross-cutting controls

Above the three columns, a thin status bar:

- Connection state (rosbridge, OAK, control node)
- Active demo + mode indicator
- Big red **Emergency Stop** button (calls `e_stop` service, drops
  legs, levels platform). Always visible.

### 11.5 Ball-lost visual feedback

The strobe fires **only** when the off-platform watchdog (§10.4)
trips — that is, the ball physically falls off the platform or both
detectors lose it for > 500 ms together. In-bounds disturbance
recovery (Demo 3 retargeting the closest path point, transient
occlusion, etc.) is silent — no strobe, just the controller working.

When the strobe condition is met:

- The page's `<body>` background colour strobes between neutral and
  red at ~3 Hz for 3 seconds. **The strobe affects only the body
  background, not panel/card backgrounds and not any overlay.** All
  controls stay fully readable and clickable throughout — the goal
  is to alert the eye, not block the GUI.
- After the strobe, the body background settles on a dim red tint
  that persists until the ball is detected again with confidence
  ≥ 0.8 for 1 s (matches the re-arm condition in §10.4) AND the
  operator presses Start (manual re-arm).
- A toast message ("Ball lost — platform leveling. Replace ball on
  platform and press Start to re-arm.") appears top-center.
- The event is also logged silently to the standard ROS bag — the
  visual flash is in addition to, not instead of, the log.
- No audible beep (current decision; trivial to add later if desired).

The strobe is GUI-only; the platform itself goes to LEVEL_HOLD
without delay regardless of the GUI state.

### 11.7 Active Stabilization — Ball-Hold mode (scaffolded v9.1, full implementation deferred to v10)

Sits below the existing Stabilization panel in the middle column.
Keeps the ball at its current plate-frame position despite base
perturbations (someone bumps the table, bot on a wobbly cart, slow
tilting surface). Co-located with the existing plate-level loop
because both are "stabilization against base disturbance" — same
mental category, different control variable.

The two modes are mutually exclusive:

| Mode | Control variable | Sensor feedback | Use case |
|---|---|---|---|
| **Level** (existing) | top-plate orientation | platform IMU | "keep the disc flat regardless of base tilt" |
| **Ball-Hold** (new, deferred v10) | ball position on the disc | base IMU + vision | "keep the ball put despite base motion" |

Engage Ball-Hold:
1. Captures the ball's current `(px, py)` from `/ball_state` as the
   hold target.
2. Switches to mode `BALL_HOLD` with that target embedded in the
   `/control_cmd` payload.
3. Controller runs the existing PID against the captured target +
   adds a base-IMU feedforward layer.

#### Math (lives in `stewart_vision/_ball_physics.py`)

```python
# Anticipatory tilt that cancels a horizontal base-frame acceleration.
theta_ff = feedforward_tilt_for_base_accel(base_accel_mps2, ball_alpha)
# Equivalently: theta_ff ≈ base_accel / (alpha · g)
```

Both `feedforward_tilt_for_base_accel` and `saturation_fraction` are
unit-tested in `test/test_ball_physics.py` so the implementation in
`stewart_control_node` (v10) is just integration work.

#### Saturation graceful degradation (4th watchdog layer, see §10)

The Stewart top plate has a tilt envelope (set by `global_limits.yaml`)
and a translation workspace (~30 mm). Large base perturbations can
exceed both. The controller computes
`saturation = |commanded_tilt| / max_safe_tilt` per axis:

| Saturation | Behavior |
|---|---|
| < 0.80 | normal operation; status indicator GREEN |
| 0.80 – 0.95 | status YELLOW; toast "Ball-Hold approaching limit" |
| ≥ 0.95 sustained > 100 ms | drop from BALL_HOLD → LEVEL_HOLD; toast "Saturated, falling back to plate-level"; ball will drift in the base direction |

This avoids the alternative failure mode where the controller
demands tilt the platform can't deliver and the loop becomes
unstable.

#### Limitations

1. **~30 mm translation workspace.** Beyond that, BALL_HOLD silently
   degrades to plate-level holding (the ball is along for the ride).
2. **Base IMU drift.** Over sustained motion (>30 s), velocity
   integration drifts; ball position will slowly creep. Manageable
   for demo-length perturbations.
3. **~25 Hz bandwidth.** Controller runs at 50 Hz; perturbations
   above ~25 Hz can't be tracked. Hand-tap on the table = fine;
   high-frequency vibration = not tracked.
4. **Vision-link latency.** Closed-loop correction needs `/ball_state`
   at low latency. Hotspot congestion → oscillation. Surfaced via
   the existing latency KPI in Vision Debug.

#### Reserved interface (so v10 is purely additive)

- **Mode token:** `BALL_HOLD` (already accepted by `ref_generator_node`).
- **Topic payload:** `mode:BALL_HOLD {x_mm: <float>, y_mm: <float>, ball: <preset>}`
- **GUI:** "Active Stabilization (Ball-Hold)" collapsible panel with
  disabled Engage / Disengage / Recapture target buttons +
  saturation display.
- **Ball-config integration:** the feedforward uses `ball.alpha` from
  the active `ball_config` payload (Q47), so foam-vs-ping-pong
  swaps automatically retune the feedforward.

### 11.6 3D viewer (scaffolded v9, full implementation deferred)

A Three.js scene mirroring the platform state in real time. Lives as
a collapsible panel below Demo 3 in the middle column. The scaffold
is in place (placeholder div + the toggle below); the actual scene
code lands in a later iteration (post-deployment, see §16).

What the v9 implementation will render:

| Element | Source | Notes |
|---|---|---|
| Bottom plate | static | reference geometry |
| Top plate | `/platform_rpy` | tilts roll/pitch in real time |
| ArUco ring (8 squares) + dead-zone disc | static, parented to top plate | tilts with plate |
| Ball (40 mm sphere, orange) | `/ball_state` | position in platform frame |
| Trail (last ~8 s) | `/ball_state` history | see frame toggle below |
| Reference marker (Demo 2) | `/ball_ref` | hollow blue ring |
| Drawn path (Demo 3) | `/ball_path/active` | green polyline |
| Camera frustum | `oak_extrinsics.yaml` | wireframe pyramid (toggleable) |
| Classical-method ghosts | `/ball_xy_oak`/`mono`/`stereo` | semi-transparent dots in different colors (toggleable) |
| Stewart legs | `/encoders` + IK | six cylinders (toggleable; needs forward kinematics) |

#### Trail coordinate frame — user toggle (Q44, locked v9)

Two modes:

- **Plate-parented (default)** — trail is rendered as a child of the
  top-plate group, so it tilts with the platform and reads as "the
  ball's path across the disc." This is what an operator expects
  from a demo and what reads cleanly when viewed at any angle.
- **World-frame** — trail stays at the world-space positions where
  the ball physically was. More physically accurate during tilt,
  and useful for showing the platform's tilting motion against a
  static reference.

Switching the toggle re-projects the existing trail buffer into the
new frame; no data loss. Both modes consume the same /ball_state
samples; only the rendering differs.

#### Bag replay (no extra code)

`ros2 bag play <bag>` re-publishes the topics the 3D viewer
subscribes to, so a recorded run plays back automatically with full
3D fidelity. No "replay mode" to write. Slow-motion via
`--rate 0.25`; pause/resume via the bag CLI.

#### Implementation tiers (post-deployment, see §16)

| Phase | Scope | Estimate |
|---|---|---|
| 1 | Live 3D in browser: plate, ArUco, ball, trail with frame toggle, ref marker, path. OrbitControls + reset view. | ~300 lines Three.js module via ES import map |
| 2 | Static 3D plot in the post-run HTML report (Plotly inside `plot_demo_run.py`) | ~80 lines |
| 3 | Camera frustum, classical-method ghosts, Stewart leg rendering | ~150 lines |
| 4 | Animated playback scrubber + frame-by-frame stepping | ~100 lines (custom replay node, not just `ros2 bag play`) |

---

## 12. Topics, services, and messages

### New topics published by `stewart_vision`

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/oak/left/image_compressed` | sensor_msgs/CompressedImage | 15 Hz | GUI live feed only |
| `/oak/right/image_compressed` | sensor_msgs/CompressedImage | 15 Hz | GUI live feed only |
| `/oak/disparity_compressed` | sensor_msgs/CompressedImage | 5 Hz | Debug toggle |
| `/oak/ball/left_pixel` | geometry_msgs/PointStamped + confidence | ≤ 60 Hz | (cx, cy) in left image |
| `/oak/ball/right_pixel` | geometry_msgs/PointStamped + confidence | ≤ 60 Hz | (cx, cy) in right image |
| `/platform_pose` | geometry_msgs/PoseStamped | 30 Hz | ArUco board → camera frame |
| `/ball_xy_oak` | geometry_msgs/PointStamped | 60 Hz | **Truth** — OAK SpatialLocationCalculator output, platform frame. Controller closes on this. |
| `/ball_xy_mono` | geometry_msgs/PointStamped | 60 Hz | Classical mono ray-plane projection. **Logged only**, no control authority. |
| `/ball_xy_stereo` | geometry_msgs/PointStamped | 60 Hz | Classical 8-point + DLT triangulation. **Logged only**, no control authority. |
| `/method_comparison` | std_msgs/String (JSON) | 30 Hz | Per-frame OAK-vs-classical disagreements (mm) for live + post-run analysis. |
| `/ball_state` | custom (px, py, vx, vy, covariance) | 100 Hz | KF posterior fed by `/ball_xy_oak`. |
| `/ball_ref` | geometry_msgs/PointStamped | event-driven | Set by ref_generator_node |

### Calibration services (Stage C ONLY — A/B are CLI-only by design)

| Service | Type | Notes |
|---|---|---|
| `/calibrate/capture_frame` | std_srvs/Trigger | One frame per click; rejects frames with < 3 ArUco markers visible. |
| `/calibrate/solve` | std_srvs/Trigger | Runs `cv2.aruco.estimatePoseBoard`, writes `oak_extrinsics.yaml` if reprojection error < 2 px. |
| `/calibrate/reset` | std_srvs/Trigger | Clears the captured-frame buffer. |

There is **no** stage selector service. Stages A/B (custom intrinsics
and stereo recalibration) override the OAK's factory tuning and
therefore live behind the CLI only — see §6 for the rationale.

### New `/control_cmd` payloads (existing topic)

The GUI uses the existing `/control_cmd` String topic to switch modes
(rosbridge service-discovery workaround):

- `mode:LEVEL_HOLD`
- `mode:BALL_TRACK_TRAJECTORY` + JSON params (radius, period, dir, phase)
- `mode:BALL_TRACK_GOTO` + JSON target (x_mm, y_mm)
- `mode:BALL_TRACK_PATH` + JSON params (path_id, speed_mm_s, loop)
- `mode:BALL_HOLD` + JSON target (x_mm, y_mm, ball) — v10 (§11.7)
- `z_offset:<mm>` for the z-height slider
- `ball_config:<json>` — current ball preset (foam vs ping-pong, §11.7 + Q47)

`stewart_control_node` already routes `/control_cmd`; we just add the
new payload patterns to its dispatcher.

### Path upload (Demo 3)

Path uploads are large enough that stuffing them into `/control_cmd`'s
String is awkward. Use a dedicated topic:

- `/ball_path/upload` (`nav_msgs/Path` in platform frame, header.frame_id
  = "platform") — published once when the operator presses Start in
  Demo 3. The reference generator stores the path under a UUID and
  returns it via `/ball_path/active` for the GUI to render the
  current-segment highlight.

---

## 13. Latency budget

End-to-end loop must close in ≤ 40 ms to avoid wobble:

| Stage | Target |
|---|---|
| OAK exposure + on-VPU detection | ≤ 15 ms |
| USB3 → Pi + ROS topic hop | ≤ 3 ms |
| ArUco pose + projection | ≤ 2 ms |
| KF predict/update | < 1 ms |
| Controller tick | ≤ 5 ms |
| Stewart leg command via CAN | ≤ 5 ms (already measured) |
| **Total worst case** | **≤ 31 ms** |

Measure this with `stamp_now − stamp_capture` rolling average displayed
in the Vision Debug telemetry strip. Tune *only after* the budget is
green — tightening PID gains to fight latency hides the real fault.

---

## 13.5 Logging and post-run plotter (per-demo bag schemas)

Each demo records its own bag schema and ships with a tailored plotter
in `stewart_vision/scripts/plot_demo_run.py`. The reasoning: the
existing `analyze_routine_log.py` is great for the open-loop bring-up
runs but lacks the vision-loop fields, and a single one-size-fits-all
plot grid for the new demos would be cluttered and ugly. Per-demo
plots impress.

### Common bag tracks (all demos)

| Topic | Type |
|---|---|
| `/encoders` | existing |
| `/imu/data` | sensor_msgs/Imu |
| `/platform_pose` | geometry_msgs/PoseStamped |
| `/ball_state` | custom (px, py, vx, vy, P) |
| `/ball_xy_mono`, `/ball_xy_stereo` | both paths |
| `/oak/ball/diagnostic` | per-detector positions + chosen output |
| `/control_cmd` | mode transitions |
| `/oak/disparity_compressed` (downsampled) | for the cover slide |

### Demo-1-specific tracks

| Topic | Notes |
|---|---|
| `/ball_ref` | current orbit reference |
| `/orbit_params` | R, T, direction, reverse events |

Plots produced (one PNG per panel, plus a summary HTML):

- Reference orbit + measured trajectory in platform frame, with the
  orbit circle drawn for context, colored by time.
- Radial error vs time: |√(px² + py²) − R|.
- Tangential / phase error vs time.
- Tilt commands (roll, pitch) and per-leg lengths vs time.
- Latency histogram (`stamp_now − stamp_capture`).
- V0 vs V1 detector agreement (scatter + Bland-Altman).
- Direction-reversal events overlaid as vertical bars on every plot.

### Demo-2-specific tracks

| Topic | Notes |
|---|---|
| `/ball_ref` | each click target as a transition |

Plots produced:

- Trajectory traces from each previous-position to each clicked
  target (one polyline per goto leg, colored by leg).
- Settling-time histogram across the run.
- Time-to-target vs Euclidean target distance scatter (regress for
  expected speed; outliers are interesting).
- Tilt commands vs time, segmented by goto leg.
- 2D error heatmap: where on the platform does settling take longer?

### Demo-3-specific tracks

| Topic | Notes |
|---|---|
| `/ball_path/active` | the validated polyline being followed |
| `/path_params` | speed, loop flag, reverse events |

Plots produced:

- Reference path vs measured trajectory, ball trajectory colored by
  along-track progress.
- Cross-track error (perpendicular distance from path) vs time.
- Along-track progress vs time (should be linear in the constant-speed
  regime — slope deviations show where the ball lagged or led).
- Per-loop overlay if Loop was on (lap 1 vs lap 2 vs … in different
  colors) — repeatability is the wow plot here.
- Reverse events as vertical bars; trail-erase resets visible.

### Method-comparison section (added v5, every demo's report)

Drives the "we implemented every Lecture 6 technique" claim. From
`/method_comparison` (JSON @ 30 Hz) and the per-method ball-position
topics:

- 2-D scatter: OAK truth (x) vs each classical method (y), per axis,
  with the y = x line drawn. Tight clustering = method works.
- Bland-Altman plot per classical method: mean of (truth, classical)
  on x-axis, difference on y-axis; mean ± 1.96σ horizontal lines.
  The standard way to visualize agreement between two estimators.
- Per-method statistics table:
  - mean error vs truth (mm)
  - RMS error vs truth (mm)
  - 95th-percentile error (mm)
  - frames where the method produced no estimate (failure rate)
- Time-series of per-method error during the run, with demo events
  (orbit reverse, goto target reached, path waypoints) overlaid as
  vertical bars.

This is the artifact that proves to the professor that every
classical technique was implemented correctly, even though we chose
the OAK's hardware-accelerated depth for robust real-time control.
Also identifies which classical method would have been the viable
fallback had the OAK failed — useful "what if" answer for any
"why didn't you use stereo triangulation directly?" question.

## 13.6 Bag recording (added v8)

Every demo run automatically produces a rosbag2 bag for offline
analysis, plotter consumption, regression testing, and
reproducibility. `bag_recorder_node` (in `stewart_vision`) listens
to `/control_cmd` for mode transitions and runs `ros2 bag record`
as a managed subprocess.

### Trigger

| Transition | Action |
|---|---|
| `LEVEL_HOLD` → `BALL_TRACK_*` | Start recording on a fixed topic allowlist. |
| `BALL_TRACK_*` → `LEVEL_HOLD` | SIGINT the recorder so rosbag2 flushes cleanly. |
| Off-platform watchdog fires during a recording | Bag is renamed with a `_fault` suffix on close so faulty runs are easy to filter for the post-mortem. |

### Recovering the ball type from a bag

Per Q47, `/control_cmd` is captured by default and includes
`ball_config:` payloads (every 5 s + on every dropdown change + at
every demo Start). Two ways to read which ball was in use during a
recorded run:

```bash
# Latest ball_config message before the run started:
ros2 bag info ~/stable_bot_bags/<run>.bag/   # confirm /control_cmd is in the bag
ros2 bag play ~/stable_bot_bags/<run>.bag/ \
    --topics /control_cmd | grep ball_config | head -1
```

Or programmatically in `plot_demo_run.py` — read the last
`ball_config:` payload before the first `mode:BALL_TRACK_*` and
annotate the report with "Ball: foam (α=0.71)".

### Topic allowlist (default — small messages only)

```
/encoders, /platform_rpy, /imu/data, /odrive_errors,
/control_cmd, /control_result,
/platform_pose, /platform_pose/markers_visible,
/ball_state, /ball_state/cov, /ball_ref,
/ball_xy_oak, /ball_xy_mono, /ball_xy_stereo,
/method_comparison,
/oak/ball/diagnostic, /oak/ball/v0/rgb_pixel, /oak/ball/v0/diagnostic,
/oak/ball/v1/rgb_pixel,
/ball_path/active
```

Total bandwidth: typically < 50 KB/s = < 3 MB / minute.

### Image-recording opt-in

Image topics (~50 MB/min) are excluded by default. When a run is
going into the formal report and you need raw frames, publish
`record_images:on` to `/control_cmd` BEFORE starting the demo. The
toggle persists until you publish `record_images:off` or restart
the node. The GUI will surface this as a checkbox in each demo
panel.

Topics added when image recording is on:
```
/oak/rgb/image_compressed, /oak/left/image_compressed,
/oak/right/image_compressed, /oak/disparity_compressed
```

### Storage

Bags land in `~/stable_bot_bags/` on the Pi:
```
~/stable_bot_bags/
  20260426T103015Z_trajectory.bag/        # Demo 1
  20260426T103420Z_goto.bag/              # Demo 2
  20260426T103820Z_path.bag/              # Demo 3
  20260426T104235Z_goto_fault.bag/        # ball fell off mid-demo
```

`mcap` storage format (modern rosbag2 default; better random-access
seeking and smaller than sqlite3 for our schema). The plotter
(§13.5) opens these directly.

### Disk usage and rotation

A typical demo run is 10–60 seconds. Without images, bags are
~1–3 MB each. The Pi has a 32 GB+ SD card, so even 1000 demo runs
fits comfortably without rotation. **No automatic deletion** — the
operator clears `~/stable_bot_bags/` manually when needed. (We
don't want a "we lost the great demo run" footgun from a too-eager
auto-rotate.)

### Plotter consumption

```bash
# On the Pi, after a run:
ros2 run stewart_vision plot_demo_run \
    --bag ~/stable_bot_bags/20260426T103015Z_trajectory.bag \
    --demo 1 \
    --out ~/stable_bot_reports/20260426T103015Z/
```

The plotter writes the HTML index + per-panel PNGs + comparison.csv
+ summary.json into `--out` (see §13.5).

### Tier-2 GUI integration (deferred until GUI work)

When the index.html additions land, each demo panel will get a
"Logging" sub-section with:
- "Record bag" toggle (default ON) — feeds `record_images:off|on`
  to /control_cmd before Start.
- "View last bag" link — shows the most recent bag's path + offers
  a "Generate report" button that runs the plotter remotely and
  serves the resulting HTML.

### Plotter interface

```bash
ros2 run stewart_vision plot_demo_run \
    --bag <bag_dir> --demo {1|2|3} --out <out_dir>
```

Reads the bag once, classifies events, and emits **three artifact
families** under `<out_dir>` (decision: 2026-04-26):

1. **`index.html`** — browser-friendly summary with embedded
   thumbnails, per-section nav, and inline summary stats. The
   artifact you take into the review meeting.
2. **`panels/*.png`** — full-resolution per-panel images so you can
   pull individual plots into a slide deck or a paper without
   parsing the HTML.
3. **`comparison.csv`** — one row per frame, columns for OAK truth
   (x, y, z), each classical method (x, y, z), per-axis errors, and
   the chosen-method tag. Easier for grading scripts, Excel
   pivots, or feeding into another notebook.

Plus a `summary.json` with the rolled-up statistics (RMS errors,
settling-time medians, lap repeatability, latency p50/p95) for
machine consumption.

Latency, fault, method-comparison, and detector-agreement panels
are common to all three demo types so the boilerplate is shared in
`_common_panels.py`.

---

## 14. Implementation milestones

Each milestone has an explicit "DONE WHEN" gate, in the same style as
`bringup_plan.md`.

1. **Print and stick the ArUco ring.**
   DONE WHEN: visual inspection confirms 8 markers at 120 mm radius,
   matte sticker is flat (no bubbles), no glare under IR projector.
2. **OAK driver up.**
   DONE WHEN: `/oak/left/image_compressed` shows a stable image in
   the GUI. Frame timestamps monotonic; drop rate < 1 %.
3. **Calibration Stage A + B done; report generated.**
   DONE WHEN: `oak_intrinsics.yaml` written; `calibration_report.html`
   shows < 0.3 px reprojection / < 1 px epipolar; visual inspection of
   rectified pairs shows aligned epipolar lines.
4. **Calibration Stage C done.**
   DONE WHEN: `oak_extrinsics.yaml` written; ArUco ring projects back
   into the image at < 2 px error. (This is repeated whenever the
   camera arm moves.)
5. **`/platform_pose` at ≥ 30 Hz.**
   DONE WHEN: tilt the plate by hand → roll/pitch from
   `/platform_pose` agrees with the IMU within 1° static.
6. **Ball pixel detection.**
   DONE WHEN: drop ball into frame; `/oak/ball/left_pixel` and
   `right_pixel` track it by eye; confidence > 0.8 in normal light.
7. **`/ball_xy_mono` in mm.**
   DONE WHEN: place the ball at known offsets from center; reported
   value within 3 mm of ruler measurement at all four cardinal points
   and at platform_radius − ball_radius.
8. **`/ball_xy_stereo` agrees with mono.**
   DONE WHEN: max disagreement < 5 mm across the platform.
9. **KF tracking.**
   DONE WHEN: roll the ball by hand; `/ball_state` velocity matches
   the visible motion direction and magnitude (eyeball test); NIS
   between 0.5 and 5 most of the time.
10. **Closed-loop ball-centering (sanity).**
    DONE WHEN: drop ball off-center, platform tilts to push it toward
    center; settles within ±10 mm of center inside 10 s. PID tuning
    starts here.
11. **Demo 1: orbit at fixed radius.**
    DONE WHEN: ball orbits at requested R within ±5 mm RMS for one
    full minute at T = 4 s.
12. **Demo 2: click-to-position.**
    DONE WHEN: click any 5 random points on the GUI; ball reaches
    each within ±10 mm and holds for 5 s before next click.
13. **Demo 2: dead-zone enforcement.**
    DONE WHEN: clicks inside red zone are ignored; manually pushing
    the ball into the dead-zone triggers the soft-repulsion controller
    and the watchdog fault.
14. **Demo 3 (stretch): path follow, single-pass.**
    DONE WHEN: draw a 5-vertex polyline; ball reaches start then
    follows path within ±15 mm RMS at 80 mm/s.
15. **Demo 3 (stretch): looped path.**
    DONE WHEN: same path with Loop checked; ball completes 3
    consecutive laps without RMS error degrading > 50 % from lap 1.
16. **Demo 3 (stretch): path safety.**
    DONE WHEN: paths drawn through the dead-zone are rejected at the
    UI with a clear toast; paths drawn off-platform same.
17. **YOLO V1 training data captured and labeled.**
    DONE WHEN: 5–6 min of free-rolling-ball capture run via
    `capture_training_frames.py`; 600+ frames labeled (orange ball
    bbox per frame); train/val split saved.
18. **YOLO V1 trained and deployed on the OAK.**
    DONE WHEN: V1 publishes to `/oak/ball/v1/*pixel` at ≥ 30 fps;
    `/oak/ball/diagnostic` shows median V0–V1 disagreement < 5 px in
    normal lighting.
19. **Per-demo plotter shipping.**
    DONE WHEN: `plot_demo_run.py --demo {1|2|3}` produces an HTML
    index + per-panel PNGs from a real bag for each demo type.
    Panels load in a browser with no missing data warnings.
20. **Off-platform abort + strobe.**
    DONE WHEN: rolling the ball off the platform edge triggers
    LEVEL_HOLD within 200 ms, the GUI body background strobes red
    for 3 s, and Start re-arm only succeeds after the ball is back
    on-platform for 1 s.

---

## 15. Open questions

### Resolved (2026-04-25)

| # | Question | Decision |
|---|---|---|
| Q1 | Z-height slider semantics | Pure heave, no tilt change, 0–80 mm above neutral. |
| Q2 | Live camera feed during Demo 2 | Stays in left-column Vision Debug panel, above the encoder/current debug values. |
| Q3 | Click during in-flight goto | Cancel and retarget immediately. |
| Q4 | Predicted-path overlay | Toggleable checkbox in each demo panel, default off. |
| Q5 | Demo 1 orbit start | Drive ball to orbit radius from current location via Demo-2 goto law, then engage orbit. |
| Q6 | Camera mount preset | Not possible (free-form articulated arm) — instead, recover camera pose from the ArUco ring's known geometry (Stage C, §6). |
| Q7 | GUI layout | Three-column layout: left = Vision Debug + encoders, middle = demo panels, right = existing controls. |
| Q8 | Start / Stop buttons | Mandatory on Demo 1, Demo 2, AND Demo 3. Big red Emergency Stop above the columns. |
| Q9 | Demo 3 (path drawing) | Added as stretch goal. Polyline draw + smoothing + arc-length-resampled traversal + optional loop. |
| Q10 | Ball choice | Foam (40 mm, mass-matched, solid) for the final demo; tune PID with ping-pong first because gentler dynamics converge faster. |
| Q11 | Demo-1 direction reversal | Yes — UNO-reverse-card-style button, available mid-orbit. |
| Q12 | Path traversal speed | Operator-set slider, default 80 mm/s. |
| Q13 | Path mid-traversal disturbance | (a) retarget closest path point if ball stays on platform; (b) abort to LEVEL_HOLD if ball leaves platform (>200 mm from center) or detection lost >500 ms. 180 mm hysteresis radius for re-arm. |
| Q14 | Path persistence | Browser localStorage with named save/load slots **plus** export/import to .json file (schema_version 1, see §11.2.3). |
| Q15 | Path display during traversal | Default = trail-erase (segment behind ball greys out); toggleable to static-green-polyline. |
| Q16 | Path-into-dead-zone handling | Reject the whole path; user must redraw. No auto-clip. |
| Q17 | YOLO timing | Build V1 in parallel with V0 from day one; both publish, ball_localizer cross-checks (§8). |
| Q18 | Ball-lost UI feedback | Background strobes red ~3 Hz for 3 s, then dim-red until re-armed. Silent log also kept. No beep. |

| Q19 | Logging format | Per-demo bag schemas + dedicated `plot_demo_run.py` plotter producing per-demo PNG panels and an HTML index — the wow artifact for review (§13.5). |
| Q20 | YOLO training data capture | `capture_training_frames.py` records JPEGs at 2 Hz; plan calls for 5–6 min of free-rolling capture (~600–700 frames) for stronger generalization. |
| Q21 | Re-arm after off-platform abort | Manual only. Operator must press Start once ball is back inside 180 mm hysteresis radius and detected for 1 s. |
| Q22 | UNO-reverse button placement | Demo 1 panel **and** Demo 3 panel (path drawing). Not Demo 2; not the status bar. |
| Q23 | Strobe-red trigger scope | Off-platform watchdog only. In-bounds disturbance recovery is silent. Strobe is body-background-only — never overlays panels or controls. |

### Resolved in v5 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q24 | Compute platform | Raspberry Pi 5, 16 GB RAM, ROS 2 Kilted. Phase 8 migration complete. systemd auto-launch. |
| Q25 | OAK USB port | Blue USB-A 3.0 on the Pi 5 (USB-C is power-only on Pi 5). |
| Q26 | Network | Same-LAN via Pixel 9 hotspot. GUI at `http://<pi-ip>:8080/`. Hostname `stablebot`. |
| Q27 | Vision strategy | OAK onboard depth = ground truth (controller closes on it). Lecture-6 classical methods run in parallel as comparison estimators only. |
| Q28 | Comparison artifact | Live `/method_comparison` JSON @ 30 Hz for the GUI's vision-debug strip; rolled up post-run by `plot_demo_run.py` into a dedicated section of the demo HTML report. |
| Q29 | Calibration capture flow | GUI buttons (Capture / Solve / Reset) calling Pi-side ROS services — Pi is headless. |
| Q30 | Chessboard generator | Shipped (`stewart_vision/scripts/generate_chessboard.py`). |

### Resolved in v6 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q31 | Calibration shortcut | Default to `--stage factory` — read OAK EEPROM via `Device.readCalibration()` and write to `oak_intrinsics.yaml`. Stages A/B remain available for users who want a tighter custom baseline. |
| Q32 | Plotter outputs | HTML index + `panels/*.png` + `comparison.csv` + `summary.json`. CSV columns documented in §13.5 plotter section. |
| Q33 | Live KPI thresholds | green < 5 mm, yellow ∈ [5, 15) mm, red ≥ 15 mm — applied to all three method-comparison numbers in the vision-debug strip. |
| Q34 | Additional shadowed comparisons | Add custom rectification + custom SGBM disparity as recommended Lecture-6 demonstrations (§7 table). Custom intrinsics/extrinsics deferred (tied to Stages A/B which we're skipping). |
| Q35 | Failure-mode fallbacks | OAK depth fail → toggle controller to `/ball_xy_mono` (already running, just no control authority by default). OAK device fail → watchdog to LEVEL_HOLD; operator power-cycles. |

### Resolved in v7 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q36 | GUI calibration scope | Stage C ONLY (camera-to-world ArUco ring). Stages A/B never reachable from the GUI — they override factory tuning and live behind the CLI flow on the Pi to prevent operator misclicks. |
| Q37 | Existing `web/index.html` layout | Already three-column (`lg:grid-cols-3`); the vision work *adds panels into* the existing structure rather than restructuring. |

### Resolved in v8 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q38 | Bag recording | Auto-record on every demo Start; fault-tag bags when the off-platform watchdog fires. Default allowlist excludes images. mcap storage in `~/stable_bot_bags/`. |
| Q39 | Image-topic recording | Opt-in via `record_images:on` on /control_cmd. GUI exposes this as a per-demo checkbox in v8.1. |
| Q40 | Bag retention | No automatic rotation. Operator clears `~/stable_bot_bags/` manually. Avoids the "we deleted the great run" footgun. |
| Q41 | Vision Debug placement (GUI) | Top of left column — horizontally adjacent to the new demo panels at the top of the middle column. Visible whenever demos are running. |
| Q42 | Demo SVG canvas | Single shared SVG at the top of the middle column. Active demo's panel border + SVG border change color (Demo 1 = blue, Demo 2 = green, Demo 3 = purple, LEVEL_HOLD = grey). |
| Q43 | Demo activation interlock | Soft interlock: pressing Start on demo X automatically stops whatever was running and transitions through LEVEL_HOLD. Fewer clicks than a hard interlock or modal prompt. |

### Resolved in v9 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q44 | 3D viewer trail coordinate frame | User-toggle: plate-parented (default, trail painted on disc and tilts with it) vs world-frame (trail stays where ball physically was in space). Both rendered from the same /ball_state samples. |
| Q45 | 3D viewer scope (now vs later) | Scaffolded only in v8.1: collapsible panel with placeholder canvas + the trail-frame toggle visible-but-disabled. Full Three.js implementation deferred to post-deployment iteration (§16 step 13–14). |
| Q46 | Iteration order after first hardware test | Locked in §16. The 14-step ladder is gated by physical observation; don't advance past a step until its DONE-WHEN condition (§14) is green. |
| Q47 | Ball-type tracking | New `<select>` in the GUI header (foam vs ping-pong). Selection persists in localStorage and publishes `ball_config:<json>` on /control_cmd, repeating every 5 s for late subscribers. Each demo Start payload also embeds the current ball config under a `ball` key. Bag captures both via /control_cmd allowlist. Single source of truth: `stewart_vision/_ball_physics.BALL_PRESETS`. |
| Q48 | Test policy | Yes, but pragmatic: pytest unit tests for pure-math + config schemas (ball physics, ArUco YAML loader, ball_safety.yaml, marker_layout.yaml). No CI; no hardware-loop tests. Run `pytest stewart_vision/test/` locally before each push. v9 baseline: 35 passing, 3 cv2-dependent skips on the laptop, all passing on the Pi where opencv-contrib is installed. |

### Resolved in v9.1 (2026-04-26)

| # | Question | Decision |
|---|---|---|
| Q49 | Active Stabilization combined with existing Stabilization panel? | Yes, conceptually — the new "Active Stabilization (Ball-Hold)" panel is co-located with the existing Stabilization (Level) panel in the middle column. The two are mutually exclusive control modes (Level holds plate orientation; Ball-Hold holds ball position), but they share the "stabilize against base disturbance" parent concept. |
| Q50 | New mode token | `BALL_HOLD` reserved in `ref_generator_node._on_control_cmd`. Payload: `mode:BALL_HOLD {x_mm, y_mm, ball}`. |
| Q51 | Saturation graceful degradation | 4th watchdog layer (§10): saturation < 0.80 GREEN; 0.80–0.95 YELLOW + toast; ≥0.95 sustained 100 ms drops to LEVEL_HOLD with toast. Math via `_ball_physics.saturation_fraction`. |

### Still open (Ball-Hold detail decisions, blocking spec §11.7)

The scaffold reserves the mode and ships the math, but four UX
details for the v10 implementation aren't locked yet. Recommended
defaults are below; confirm or override:

1. **Standalone Ball-Hold OR a toggle on Demo 2?** Two ways to expose
   the base-IMU feedforward layer:
   - (a) standalone: separate Ball-Hold mode, captures current ball
     position as target. **(recommended for v10)** — clean mental
     model, simplest UX.
   - (b) layered: a "World-frame hold" checkbox on Demo 2 that adds
     the same feedforward to a clicked goto target. Could also
     extend to Demos 1 and 3 for orbit/path tracking that rejects
     base motion.

2. **What if no ball is detected when Engage is clicked?**
   - (a) refuse; toast "no ball detected — place a ball on the
     platform first" **(recommended)**.
   - (b) wait; engage hold once ball appears.

3. **While in Ball-Hold, can the operator click on the SVG to update
   the hold target without disengaging?**
   - (a) yes: clicking is treated like Demo 2's retarget, just with
     base-IMU feedforward layered on. **(recommended)**
   - (b) no: target is locked at engage time; must Recapture or
     Disengage to change.

4. **Saturation visual feedback granularity.**
   - (a) status text + color (green/yellow/red) on saturation %
     **(recommended; consistent with no-beep policy)**.
   - (b) also a non-strobing border pulse on the GUI's Ball-Hold
     panel when YELLOW.
   - (c) audio chirp on YELLOW. (would break the no-beep precedent)

---

## 16. Post-deployment iteration plan

Once v9 (with the v8.1 GUI scaffold) is on the Pi and a Stage-C
calibration has succeeded, the demo flow can be exercised
end-to-end against real hardware. From there, implementation work
proceeds in this order, **one piece at a time, gated by physical
observation**. Don't advance past a step until its corresponding
DONE-WHEN gate in §14 is green.

| # | Step | What to flesh out | DONE-WHEN gate (§14) |
|---|---|---|---|
| 1 | **Smoke test** | Full stack auto-launches; GUI loads; live RGB feed shows up; ArUco markers detected at 8/8 with platform leveled. | Milestones 1, 2 |
| 2 | **Stage-C calibration** | GUI button flow works end-to-end; reproj < 1 px. | Milestone 4 |
| 3 | **Stub validation** | Manually publish a fake `/ball_state` from the CLI; confirm the SVG ball moves and Demo 2 click handler fires before any real ball detection works. | (no formal gate; sanity check) |
| 4 | **V0 ball detection** | Verify `oak_driver_node`'s host-side HSV threshold finds the orange foam ball; tune HSV_LO / HSV_HI by watching the live feed. | Milestone 6 |
| 5 | **`/ball_xy_mono`** | Wire `ball_localizer_node`'s ray-plane projection. Hand-move the ball; ruler-test against the reported (x, y). | Milestone 7 |
| 6 | **KF on the mono path** | `ball_kf_node` smooths position + produces velocity. NIS in [0.5, 5] under hand motion. | Milestone 9 |
| 7 | **Closed-loop ball-centering (sanity)** | Drop ball off-center; platform tilts to push it toward center; settles within ±10 mm in 10 s. **Tune Kp first**. | Milestone 10 |
| 8 | **Demo 1 — Orbit** | Ball-track-trajectory mode. Tune Kd to remove wobble, then small Ki for stiction. Reverse button works. | Milestone 11 |
| 8.5 | **BALL_HOLD (Active Stabilization)** | Implement controller-side base-IMU feedforward in `stewart_control_node`: subscribe to `/base/imu/data`, compute `theta_ff` from `_ball_physics.feedforward_tilt_for_base_accel`, add to PID output. Engage from the GUI's Ball-Hold panel; bump the table by hand and watch the ball stay put within ±5 mm. Saturation graceful degradation kicks in past 95% commanded-tilt for >100 ms. Spec §11.7. | (no formal gate; ball stays within ±5 mm of target during a 200 mm hand-bump) |
| 9 | **Demo 2 — Click-to-Goto** | Same gains transfer; ±10 mm settling. Z-height slider commands pure heave. | Milestone 12 |
| 10 | **Stereo triangulation** | Implement `/ball_xy_stereo` in `ball_localizer_node`. `/method_comparison` shows green most of the time. | (extends Milestone 8) |
| 11 | **YOLO V1 detector** | Capture training frames via `capture_training_frames.py`; label; train YOLOv5-nano; deploy to OAK as `/oak/ball/v1/rgb_pixel`. | Milestones 17, 18 |
| 12 | **Demo 3 — Path drawing** | Implement `ref_generator_node`'s path-mode arc-length traversal + smoothing + Loop closure + Reverse. | Milestones 14, 15, 16 |
| 13 | **Plotter — `plot_demo_run.py`** | rosbag2_py reader + matplotlib panels + comparison.csv + summary.json + HTML index. Closes the loop with the bag-recorder. | Milestone 19 |
| 14 | **3D viewer (Phase 1)** | Live Three.js scene: plate, ArUco, ball, trail with frame toggle, ref marker, drawn path. OrbitControls + reset. (§11.6) | (no formal gate; visual diff vs SVG-2D) |
| 15 | **3D viewer (Phase 2)** | Plotly 3D in the post-run HTML report (in `plot_demo_run.py`). | (no formal gate) |
| 16 | **3D viewer (Phase 3)** | Camera frustum, classical-method ghosts, Stewart leg rendering from `/encoders` + IK. | (no formal gate) |

### Pre-flight before pushing this code to the Pi

- [ ] `git pull` on the Pi to grab the new GUI + nodes.
- [ ] `colcon build --packages-select stewart_vision` on the Pi
      (depthai, opencv-contrib-python, cv_bridge available).
- [ ] Restart `stable_bot_gui.service` so the new index.html is
      served.
- [ ] Open `http://stablebot.local:8080/` from the laptop browser;
      confirm the new Vision Debug + Calibration panels render in
      the left column and the demo panels render in the middle.
- [ ] If the OAK isn't plugged in yet, the page will still load —
      `oak_driver_node` will log a fatal but the existing control
      stack is unaffected. Plug in the OAK and `systemctl restart
      stable_bot.service` to bring vision online.
- [ ] Run Stage C calibration. Confirm
      `~/ros2_ws/install/stewart_vision/share/stewart_vision/config/oak_extrinsics.yaml`
      gets written.

Once all four checkboxes pass, you're at step 1 of the iteration
ladder above.

## 17. References

- `stewart_bringup/docs/NEXT_STEPS.md` — prior vision plan (this doc
  refines it).
- `bringup_plan.md` — master 8-phase bring-up plan.
- `stewart_bringup/scripts/generate_aruco_ring.py` — produces the
  printable ArUco ring + `marker_layout.yaml`.
- `stewart_bringup/config/global_limits.yaml` — three-layer speed cap
  redundancy pattern (mirrored for the ball dead-zone).
- `stewart_bringup/config/platform_level.yaml` — IMU-frame level
  reference (now zeroed against the MTi factory cal as of 2026-04-25).
- EEL 4930/5934 Lecture 6 — Robot Perception (camera model,
  homography, calibration, stereo, epipolar geometry, F/E matrices,
  8-point + RANSAC, triangulation, SfM).
- EEL 4930/5934 Lecture 8 — Localization, Filtering & State Estimation
  (Bayesian filter, KF, EKF, PID).
- Movella MTi 600-series User Manual — IMU sensor-frame conventions.
- OpenCV `cv2.aruco`, `cv2.calibrateCamera`, `cv2.stereoCalibrate`,
  `cv2.stereoRectify`, `cv2.solvePnP`, `cv2.triangulatePoints`.
- Luxonis DepthAI documentation (OAK-D-PRO-AF pipeline).
