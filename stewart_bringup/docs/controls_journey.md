# Stable-Bot controls journey + lessons learned

A chronological record of debugging the closed-loop ball-on-platform
demos to a working state, with the lessons preserved so future-us
doesn't repeat the same mistakes. Spans 2026-04-30, the day vision
went from "0–8 Hz of intermittent ball detections" to "30 Hz of 100 %
detections feeding a controller that actually tracks targets."

This doc is intended to be read end-to-end by anyone (operator or
agent) who will work on the controls or vision pipeline and wants
context on *why* the system is shaped the way it is.

---

## TL;DR — top three lessons

1. **The pitch axis sign is opposite the roll axis sign in the IK
   math.** `_rot_rpy(0, p, 0) = Ry(p)` (right-hand rule about +Y)
   takes body `(1, 0, 0)` to world `(cos p, 0, -sin p)` — the +X edge
   moves DOWN for positive pitch, so the ball slides toward +X. The
   BALL_TRACK loop's docstring assumed the opposite. With ArUco-IMU
   alignment in place, the working sign combination is
   `pitch_sign = -1.0, roll_sign = +1.0`. Roll is symmetric to
   intuition; pitch is not. Locked into
   `stewart_bringup/config/ball_track_gains.yaml`.

2. **Frame mismatches between `/ball_ref` and `/ball_state` produce
   the symptom "click never matches tilt direction."** No PID gain
   set or sign flip can compensate for two different rotated frames.
   The controller subtracts ref from state — both must live in the
   same frame, or the result is garbage. `ball_localizer_node`
   applies an ArUco→IMU 2×2 rotation; pre-2026-04-30, `ref_generator`
   did not. They do now (commit `0f4efe9`).

3. **Vision rate problems are usually not the camera or USB — they
   are usually exposure or lighting.** `setFps(60)` is a *ceiling*,
   not a floor. The IMX378 auto-exposure controller is allowed to
   pick any integration time up to the frame interval; in dim rooms
   it picks 30–67 ms and the framerate floor-divides accordingly.
   `setManualExposure(8000, 800)` + a real light source took the
   detection rate from 4 Hz to 30 Hz with the same code.

---

## Timeline

### Morning: vision starvation, multiple suspects

V0 detection was at 0–8 Hz across most of the day. Hypotheses tested
and discarded:

- **USB-2 fallback.** `lsusb -t` showed `5000M Driver=xhci-hcd` —
  USB-3 SuperSpeed was negotiated. Not the bottleneck.
- **`usb_max_current_enable`.** Pi 5 config had `=1` — power was fine.
- **Old `depthai-python` version.** 2.32.0.0 was current.
- **ISP-scaler back-pressure.** Probed with `OAK_ISP_SCALE=full`
  (skip the 1080p→540p downscale). v0_arr went DOWN, not up — ruled
  out scaler as the cap. Bigger frames just took more USB time.
- **Frame pool starvation.** Bumped `setNumFramesPool(2,3,4,4,4)`
  on `cam_rgb`. No effect.
- **Camera FPS too low.** Bumped from 15 → 60. Symptom regressed
  *worse* until the actual cause (next item) was addressed.

The actual fix was **manual exposure + manual focus**:

```python
cam_rgb.initialControl.setAutoFocusMode(...)              # pre-2026-04-30
cam_rgb.initialControl.setAutoExposureLimit(16500)        # didn't help
# became:
cam_rgb.initialControl.setManualFocus(145)
cam_rgb.initialControl.setManualExposure(8000, 800)
```

`setAutoExposureLimit` should have worked but didn't on the OAK
firmware shipped with depthai 2.32; only the manual setter forced
the exposure to a value short enough to free up the framerate. Manual
focus also stopped the camera from stalling on continuous-AF refocus
events.

After this, `v0_arr` lifted to 15–22 Hz. Still not 60. Investigating
further showed USB-effective throughput is the real ceiling on
OAK-D Pro AF — practical sustained transfer is more like 30–50 MB/s
than the theoretical 400 MB/s of USB-3 SuperSpeed. At 540p × 1.5 MB =
1.5 MB/frame, that mathematically caps at 20–30 Hz. Confirmed by
monotonic v0_arr ≈ 20 across exposures.

**Lesson:** if `setFps(N)` is set but `[health]` shows v0_arr much
less than N, suspect (in order): exposure too long, autofocus
stalling, USB transfer ceiling. Don't trust `setAutoExposureLimit`
on this firmware — use `setManualExposure`.

### Midday: detection accuracy, after rate was solved

Even at 30 Hz, the V0 detector had two failure modes:

- **Off-platform false positives** — the platform mask radius was
  0.220 m (slightly larger than physical to avoid clipping the rim).
  After loosening HSV bounds (S 140 → 100, V 90 → 60) for motion-
  blur tolerance, the looser threshold started catching bench
  shadows at the rim. Tightened the mask to 0.190 m (commit
  `f963860`); operator's intuition was correct that loosening one
  needed tightening the other.

- **Motion-blur drops detection mid-trial.** When the ball started
  rolling, V0 success rate dropped from 100% to ~33%. Looser HSV
  fixed most of this, but the remaining ~5% of frames where the
  ball was at peak motion blur kept missing.

**Lesson:** HSV detection on a moving ball is brittle. Each ~2× of
exposure time roughly doubles the per-pixel light, but it also
doubles the motion smear. Trade-off: shorter exposure + brighter
ambient light is the dominant strategy. The IKEA panel that arrived
late in the day made the shorter-exposure path work.

### Afternoon: the actual control bug

This was the biggest finding of the day. The operator reported:
**"where I click on the SVG has never once been the direction
that the ball is tilted towards."** That symptom was present
through every PID and bang-bang attempt regardless of gain choices.

A code trace through the data flow revealed two compounding bugs:

#### Bug 1 — frame mismatch (§0)

```
ref_generator    → /ball_ref     in ArUco/platform frame
ball_localizer   → /ball_xy_mono in IMU frame (rotated 158°)
ball_kf          → /ball_state   in IMU frame
stewart_control  → err = state - ref     ← mixing frames
```

The 158° came from the IVA Procrustes fit between IMU readings and
ArUco-derived roll/pitch over a sweep — the IMU was simply mounted
at that yaw rotation relative to the markers. `ball_localizer`
applied the rotation correctly; `ref_generator` didn't. So the
controller's error vector was always rotated 158° from where it
should have been.

**Fix (commit `0f4efe9`):** `ref_generator_node` now loads the same
`stewart_vision/config/aruco_imu_alignment.yaml` and applies the
rotation to every published ref. Both topics live in IMU frame.

#### Bug 2 — IK sign asymmetry

After §0 the error vector was coherent, but the ball *still* went
the wrong direction. Tracing `_rot_rpy(0, p, 0) = Ry(p)`:

```
Ry(p) = [ cos(p)  0  sin(p)]
        [      0  1       0]
        [-sin(p)  0  cos(p)]

Ry(p) · (1, 0, 0)ᵀ = (cos p, 0, -sin p)
```

For `p > 0`, the platform's +X edge ends up at world Z = `-sin(p)`
which is **below** zero — the +X edge moves **DOWN**. Ball on the
inclined surface slides toward the lower edge, i.e., **+X**. The
BALL_TRACK loop's docstring claimed the opposite ("ball at +x
commands +pitch ... drives ball toward x=0"). For any positive ex,
`tilt_pitch = ps · ex` with `ps=+1` produced +pitch and the ball
moved away from target.

For roll, `Rx(r) · (0, 1, 0)ᵀ = (0, cos r, sin r)` — **+Y edge moves
UP** for positive r, ball slides −Y. Roll matches the loop's
docstring. The asymmetry is built into the right-hand-rule rotation
matrices and is invariant — there's no sane way to make them
symmetric short of negating one axis convention.

**Fix:** set `pitch_sign = -1.0`, `roll_sign = +1.0` in
`ball_track_gains.yaml`. Operator-validated against six trials at
different sign permutations:

| signs(p,r) | duration | err_x mean | err_y mean | err_total rms | verdict |
|---|---|---|---|---|---|
| (-1, -1) | 19.6s | -84  | +153 | 219 | y inverted |
| (-1, -1) | 28.5s | +98  | +127 | 187 | y inverted |
| (-1, -1) | 44.0s | +24  | -174 | 221 | y inverted |
| (-1, +1) | 23.9s | -43  | -84  | 171 | both close |
| (+1, -1) | 10.2s | +201 | +87  | 297 | x worst case |
| **(-1, +1)** | **45.9s** | **+11** | **+22** | **194** | **first working run** |

Defaults were locked in at commit `5fc5c42`.

**Lesson:** when sign conventions are documented in code,
**always test both axes independently against the actual rotation
math**. The right-hand rule cycles axes (X → Y → Z → X), and the
sign of off-diagonal terms in `Rx`/`Ry`/`Rz` is **not** symmetric.
A docstring saying "+pitch tilts +X edge UP" is wrong with the
standard math convention; it would be correct with aerospace
convention or NED. We use the math convention. Don't trust
docstrings against formulas.

### Evening: vision pipeline polish

After the controls bug was found, several pre-existing latency
optimizations finally paid off:

- **Capture-time stamps on /oak/ball/v0/rgb_pixel.** Previously
  stamped with `rclpy.now()`; now uses `rgb_raw.getTimestamp()`
  translated through the dai-clock-to-ros-clock offset. Downstream
  KF and controller see the actual capture moment, not "now."
- **MJPEG encoder queue depth = 1, non-blocking.** Stopped the
  7-frame backlog the encoder was building.
- **JPEG quality 70 → 50.** Smaller per-frame USB load.
- **V0 platform-mask bbox crop.** cv2 HSV+contour runs on ~25%
  of the frame area instead of the full 540p. ~10 ms → ~3 ms per
  detection.
- **`/oak/health` and `/oak/config` topics.** Per-stream Hz +
  per-path latency bagged every 5 s; full configuration snapshot
  bagged every 5 s. Demo bags now self-describe — the digest
  knows what the camera was set to during the run.

Net: jpeg_lat p50 dropped from 113 ms → 70 ms, v0_lat from 50 ms
to 35 ms (median), and the digest can correlate behavior changes
with config tweaks.

### Late evening: visualization

After §0, both `/ball_ref` and `/ball_state` were in IMU frame
(rotated 158°). The SVG render became visually disagreeable — the
ref_marker appeared at the rotated position rather than where the
operator clicked. The controller worked, but the operator couldn't
visually correlate click and target.

Fix (commit `bc7b9bf`): `gui_server.py` exposes `/iva/alignment` (an
HTTP endpoint that reads `aruco_imu_alignment.yaml` and returns the
matrix as JSON). `index.html` fetches it once on page load,
transposes it to invert (rotations are orthogonal), and applies the
inverse rotation in `onBallState` and `onBallRef` so the SVG renders
everything in platform/ArUco frame. The ref_marker now appears
under the click point, and the live ball dot tracks the ball's
actual physical position.

---

## Phase 2B detector-on-OAK: experimental, env-flagged

A simple linear-color-score NN model (1×1 conv + soft-argmax) runs
on the OAK's Myriad X via DepthAI's NeuralNetwork node. Built from
`stewart_vision/scripts/build_v0_blob.py` (PyTorch → ONNX →
blobconverter), produces `stewart_vision/blobs/v0_320x180.blob`.
Toggle at runtime via the GUI's Vision Debug panel; data path:
ImageManip(540p→320×180) → NN → NNData(3 FP16 floats: cx, cy, conf)
→ host. Per-detection USB load drops from ~1.5 MB to ~16 B.

Status: works in well-lit rooms; the simple color-score model loses
the ball under low light or grabs ArUco corners. Not yet trusted
for production demos — kept as an A/B option. The cv2 path remains
the default for tuning. See `oak_phase2b_on_device_v0.md` for the
full design.

---

## Auto-tuner — the next phase

Plan: `auto_tuning_plan.md`. Implementation in progress.

Phase 1 hill-climbing optimizer over:
- `kp`, `kd`, `ki`, `max_tilt_deg`
- `pitch_sign`, `roll_sign` (categorical ±1 search — locked at -1/+1
  by default but the optimizer is allowed to flip them in case some
  future hardware change requires re-discovery)
- `alignment_offset_deg` ∈ [−15°, +15°] — residual rotation on top
  of the IVA-fit alignment, lets the optimizer compensate for a
  bad IVA fit without needing a re-sweep

Fitness composition (weights tunable after first 5–10 sanity-check
trials):
- `f_err`     = `1 / (1 + rms_err / 50)`  — 1.0 at 0 mm, 0.5 at 50 mm
- `f_p95`     = `1 / (1 + p95_err / 100)` — penalize tail oscillations
- `f_settle`  = `max(0, 1 - settling_time / 15)`
- `f_hold`    = on-target fraction (err < 25 mm tolerance)
- **`f_calm`**= `1 / (1 + mean_speed_mm_per_s / 100)` — punish a
  ball that's "tracking" by zooming in circles through the target.
  A ball that calmly settles and holds gets f_calm ≈ 1; one that
  loops at 100 mm/s gets ≈ 0.5; runaway oscillation at 200 mm/s
  drops to ~0.33. This stops the optimizer from rewarding gain
  combos that are technically "low rms" because the trajectory
  passes through the target frequently while otherwise being
  unstable.

`fitness = 0.25·f_err + 0.15·f_p95 + 0.15·f_settle + 0.25·f_hold + 0.20·f_calm`

Trial protocol: targets are generated at a **fixed distance D from
the current ball position** (D = 60 mm by default), with a random
angle θ. This standardizes the difficulty of every trial — without
this, a randomly-placed target close to the ball gets a high
fitness for free, polluting the comparison between gain sets.
Algorithm:

```
ball_xy = read_current_ball_state()
for attempt in 1..16:
    θ = uniform(0, 2π)
    target = ball_xy + D · (cos θ, sin θ)
    if |target| < 0.7 · R_platform:
        break
# else: target stays inside platform; if all 16 attempts off-platform
# (ball already near the rim), shrink D for this trial.
```

Apply candidate gains via existing `_do_ball_track_save_gains`
(hot-reload, no restart), start a demo bag, publish
`mode:BALL_TRACK_GOTO`, wait for either `err < 25 mm` for ≥ 1 s
(settled) or 25 s timeout. Stop bag, run digest, compute fitness,
append to JSONL log.

After 30 trials of hill climbing, switch to Bayesian Optimization
via scikit-optimize for the remaining 20 trials of the budget.

Operator can run 50 trials in ~25 minutes wall-clock. JSONL log +
fitness curve PNG let them inspect what the algorithm explored.

---

## Memory aids — things that have bitten us

- **`pip install` on Ubuntu 24.04 / Python 3.12** is "externally
  managed" (PEP 668). On the Pi this means you can't easily install
  Python packages. Either build artifacts on a dev machine and
  commit them (what we do for the V0 blob), use `pipx`, or
  `pip install --break-system-packages` if you really must.

- **`stewart_vision` is a separate ROS package from `stewart_bringup`.**
  When editing oak_driver_node.py / ball_localizer_node.py /
  ref_generator_node.py / ball_kf_node.py, you must include
  `stewart_vision` in the `colcon build --packages-select` line or
  the install/ tree won't reflect the source change. Multiple
  4-hour debug rounds traced to this. The deploy script
  `pi_deploy.sh` always rebuilds all three packages.

- **Two systemd services**, not one. `stable_bot.service` runs
  `ros2 launch ...`; `stable_bot_gui.service` runs `gui_server.py`
  directly. After editing `gui_server.py`, `digest_demo_bag.py`,
  or `web/index.html`, **both** services must be restarted (`pi_deploy.sh`
  does this).

- **`set -u` in bash trips ROS setup scripts.** They reference
  vars like `AMENT_TRACE_SETUP_FILES` without defaulting them.
  Use `set -eo pipefail`, drop the `-u`.

- **Right-hand-rule rotations are not symmetric across axes.**
  `Rx(+r)` and `Ry(+p)` produce off-diagonal sign patterns that
  flip on opposite sides of the diagonal. `Rx` matches the
  controller's "+roll = +Y edge UP" intuition; `Ry` doesn't match
  "+pitch = +X edge UP" — it does the opposite. Always derive sign
  conventions from the rotation math, not from intuition or a
  comment.

- **Vision rate symptoms can have many causes.** Diagnostic order:
  (1) check exposure with `setManualExposure` — does framerate jump?
  (2) check focus mode — is autofocus stalling?
  (3) check USB negotiation with `lsusb -t` — `5000M` or `480M`?
  (4) check depthai version — `< 2.21` has known FPS bugs.
  (5) check pipeline back-pressure (encoder queue depth, frame pool).
  (6) Light. Always more light.

- **YAML save in the GUI strips comments.** `yaml.safe_dump` emits
  alphabetized keys with no comments. After every GUI gain save,
  the YAML loses any documentation. Keep the canonical comments in
  this doc + commit them back to the YAML when convenient (commit
  `5fc5c42` is one such restoration).

---

## Where the system stands as of 2026-04-30 evening

- ✓ Vision: ~30 Hz of 100 % detections with adequate lighting.
- ✓ Frame consistency: `/ball_state` and `/ball_ref` both in IMU
  frame; SVG visualizes both back in platform frame.
- ✓ Sign convention: `pitch_sign = -1, roll_sign = +1` locked in.
- ✓ First demo run that actually closed the loop: bag
  `20260430T204648Z_demo2`, 46 s, err mean (+11, +22) mm.
- ⚠ RMS error ~190 mm — gain tuning needed. **This is exactly what
  the auto-tuner is for.**
- ⊘ Phase 2B (NN backend) needs the IKEA panel for reliable
  detection across the platform; flashlight-on-camera-arm produces
  sufficient light but blooms the ball center.

Onward.
