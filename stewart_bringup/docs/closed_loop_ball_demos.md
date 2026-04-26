# Closed-Loop Ball Demos — Vision-Based Control Plan

**Project:** Stable-Bot (6-DOF Stewart platform, ROS 2 Kilted)
**Scope:** Replace the open-loop rolling-ball demo with a vision-driven
closed loop. Three demos: orbit (Demo 1), click-to-position (Demo 2),
and path-drawing follow (Demo 3, stretch goal).
**Authors:** Luke McDevitt + Claude (Opus 4.7).
**Last revised:** 2026-04-25 (v4 — final spec lock: per-demo bag
schemas + dedicated plotter, training-frame capture helper, manual
re-arm only, UNO-reverse on Demos 1 & 3 only, strobe scoped to
off-platform fall + body-background only).

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
| Compute (current) | Bring-up host is a WSL2 Ubuntu 24.04 laptop. Phase 8 of the bring-up plan migrates to a Raspberry Pi. |

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

`scripts/calibrate_oak.py` runs in three independent stages so that
moving the camera only invalidates the cheap stage.

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

The OAK gives onboard depth for free, but the prof wants to see *the
techniques*. Each item below is implemented explicitly somewhere in the
pipeline, even where the OAK could shortcut.

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

Two ball-localization paths run in parallel and are published on
separate topics:

1. **Mono + plane projection** (`/ball_xy_mono`) — ball pixel + ArUco
   plane → ray–plane intersection.
2. **Stereo triangulation** (`/ball_xy_stereo`) — rectified pair → ball
   pixel in each → DLT triangulation → into platform frame.

Disagreement between them is itself a useful diagnostic and a slide.

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

The graphical disk in the Demo-2 / Demo-3 GUI draws the dead-zone in
semi-opaque red so the operator can't accidentally click or draw into
it.

---

## 11. GUI design

The GUI remains a single `web/index.html` Tailwind + roslibjs page
served by `gui_server.py`. The page now uses a **three-column layout**
to keep all live information visible at once.

### 11.0 Three-column layout

```
┌──────────────┬──────────────────────────────┬──────────────┐
│   LEFT       │           MIDDLE              │   RIGHT      │
│              │                               │              │
│  Vision      │  Demo control panels:         │  Existing    │
│  Debug       │   • Demo 1 (Orbit)            │  controls    │
│   • Live     │   • Demo 2 (Goto)             │   • Homing   │
│     stereo   │   • Demo 3 (Path) [stretch]   │   • Jog      │
│     feed     │  Each with Start / Stop +     │   • Level    │
│   • Status   │  graphical SVG disk view.     │   • Reset    │
│   • Latency  │                               │     stack    │
│              │                               │              │
│  Encoder &   │                               │              │
│  current     │                               │              │
│  debug       │                               │              │
│  values      │                               │              │
│  (existing)  │                               │              │
└──────────────┴──────────────────────────────┴──────────────┘
```

All three columns scroll independently. On screens narrower than
1280 px the columns stack vertically (mobile-friendly fallback).

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
  - Mono-vs-stereo ball-position discrepancy (mm)

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
| `/ball_xy_mono` | geometry_msgs/PointStamped | 60 Hz | Platform frame, mono path |
| `/ball_xy_stereo` | geometry_msgs/PointStamped | 60 Hz | Platform frame, stereo path |
| `/ball_state` | custom (px, py, vx, vy, covariance) | 100 Hz | KF posterior, both frames |
| `/ball_ref` | geometry_msgs/PointStamped | event-driven | Set by ref_generator_node |

### New `/control_cmd` payloads (existing topic)

The GUI uses the existing `/control_cmd` String topic to switch modes
(rosbridge service-discovery workaround):

- `mode:LEVEL_HOLD`
- `mode:BALL_TRACK_TRAJECTORY` + JSON params (radius, period, dir, phase)
- `mode:BALL_TRACK_GOTO` + JSON target (x_mm, y_mm)
- `mode:BALL_TRACK_PATH` + JSON params (path_id, speed_mm_s, loop)
- `z_offset:<mm>` for the z-height slider

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

### Plotter interface

```bash
ros2 run stewart_vision plot_demo_run \
    --bag <bag_dir> --demo {1|2|3} --out <out_dir>
```

Reads the bag once, classifies events, emits PNG panels + an HTML
index that opens in a browser with the cover plot, summary stats,
and links to the per-panel images. Latency, fault, and detector-
agreement panels are common to all three demo types so the
boilerplate is shared in `_common_panels.py`.

The HTML index is the artifact you take into the review meeting.

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

### Still open

None — spec locked at v4 (2026-04-25).

---

## 16. References

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
