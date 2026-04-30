# Stable-Bot Vision + Control Tuning — Session Writeup (2026-04-30)

## What we set out to do

Bring up the three closed-loop ball demos (orbit, click-to-goto,
path-drawing) on the Pi-side Stewart platform. The vision stack was
nominally working; control loops weren't, and vision had reliability
issues we discovered along the way. By the end of the session the
vision pipeline is reliable, the controller architecture is correct,
and we have the diagnostic infrastructure to tune the demos
systematically rather than by guess-and-check.

---

## Architectural changes (what's different now)

### Vision pipeline

**Two parallel ball detectors instead of one.** V0 (HSV color
threshold inside the platform-mask region) and depth-blob
(above-platform-plane stereo blob inside the same mask). Both publish
independent pixel topics; ball_localizer projects both into
platform-frame mm and publishes `/ball_xy_mono` and `/ball_xy_depth`.
The GUI shows them as orange and cyan circles for direct comparison.

**Stereo close-range fix via ExtendedDisparity.** Default OAK stereo
cuts off at ~632 mm; the platform sits at ~600 mm. Setting
`setExtendedDisparity(True)` doubles the disparity range and halves
the minimum measurable distance to ~316 mm — the entire platform now
sits comfortably inside the stereo envelope.

**Self-calibrating depth-blob plane fit (then removed).** First
attempt computed `median(expected − measured)` across the eroded mask
as a per-frame plane offset; with smooth-surface stereo dropouts, the
median was driven by stereo edge artifacts at 1.5 m and the threshold
drifted. Removed in favor of trusting the ArUco-derived plane
directly with the threshold on raw `expected − measured`.

**ArUco→IMU alignment baked into ball_localizer.** The IVA sweep's
Procrustes fit measures a constant 2D rotation between ArUco-derived
camera RPY and IMU body RPY — this platform shows a 149-156° rotation
that no `±1` sign flip can compensate. The digest now writes a
standalone `aruco_imu_alignment.yaml`, and ball_localizer applies it
to ball xy in platform frame. Result: BALL_TRACK loop sees
IMU-aligned coordinates regardless of how the ArUco markers happen to
be oriented relative to the IMU body frame.

**Vision lag eliminated.** Three compounding bottlenecks:

- rosbridge JSON-array bloat (200 KB JPEG → 1.4 MB wire payload).
  Mitigated by `throttle_rate=100`, `queue_length=1`, and dropping
  OAK MJPEG quality from 95 to 70.
- Browser-side `String.fromCharCode + btoa` loop per frame
  (200K-iteration JS hot path). Replaced with
  `URL.createObjectURL(Blob)`, ~150x faster.
- `/oak/depth_blob/debug_image` overlay broadcast for visual
  diagnostics (annotated RGB with mask outlines + above-plane tint +
  detection circle).

### Controller

**Concurrent BALL_TRACK + level PI loop.** Previously these were
mutually exclusive: starting BALL_TRACK explicitly stopped the level
loop, leaving the controller open-loop on platform tilt. The fix
removes the mutual exclusion and auto-enables level when BALL_TRACK
starts. Now BALL_TRACK writes `current_rpy` as a desired-tilt
setpoint; the level loop's existing `target = level_ref + current_rpy`
formula treats it as a setpoint and uses IMU feedback to actually
achieve the commanded tilt. **IMU is the ground truth for platform
tilt**, no longer "commanded == actual" assumption.

**Bang-bang algorithm alternative to PID.** Selectable via
`algorithm: 'pid' | 'bangbang'` in `ball_track_gains.yaml`. Bang-bang
implements an explicit state machine: ACCELERATE (full tilt toward
target) → COAST (zero tilt while ball coasts) → BRAKE (full opposite
tilt) → SETTLE (zero tilt within tolerance). State machine is more
interpretable than continuous PID; each threshold has physical
meaning instead of being an interaction-laden gain.

**Bang-bang stiction breakthrough.** Foam-on-vinyl static friction
often pins the ball at `accel_tilt=3°`. New STICTION_BREAK phase: in
ACCEL with velocity below `stiction_v_threshold_mm_s` for more than
`stiction_break_s`, ramp tilt to `max_tilt_deg` (typically 6°) to
break the ball loose. Reverts to ACCEL once the ball moves.

**Asymmetric brake_tilt_deg.** Static friction (overcome at startup)
is much higher than kinetic friction (rolling ball). A fast-moving
ball needs *more* tilt to brake than a stationary ball needed to
start, because kinetic-friction-limited deceleration is
`g·sin(θ)·α`. Now `accel_tilt_deg` and `brake_tilt_deg` are
independent.

**Vision-latency lookahead.** End-to-end vision-to-command latency is
~100 ms (capture, USB transfer, KF processing, control loop). At
300 mm/s ball speed, this is 30+ mm of position drift; BRAKE phase
fires after the ball has already overshot the brake horizon. Now the
FSM extrapolates `(px, py)` forward by `control_latency_s` using KF
velocity before computing phase decisions.

**Z-compensation for off-center rotation.** Without compensation, a
tilt by θ with the ball at offset px adds `px·sin(θ)` of heave to the
ball's world-frame height. Now the BALL_TRACK loop commands
`z_cmd = z_hold + px·sin(pitch) − py·sin(roll)` so the platform
rotates *under* the ball — ball's world height invariant, only
surface gradient changes.

**Ball-fall plumbing fixed at three layers.** ball_localizer drops V0
cache after 0.4 s (no more 60 Hz publishing of stale-V0 positions).
ball_kf only publishes `/ball_state` when measurements are fresh
(predict still runs, but no broadcast on extrapolation). GUI hides
the SVG ball after 0.8 s of `/ball_state` silence and clears the
trail.

---

## Diagnostic infrastructure built

These are the workhorses for tuning. Each one mirrored an existing
pattern (the IVA bag pipeline) so the UX is consistent.

### Bag pipelines

**Vision-debug bags.** `/vision/start`, `/vision/stop`, list, digest,
push. Records RGB, depth_blob debug overlay, V0/depth pixel +
diagnostic, platform_pose, IMU. Digest extracts up to 9 keyframes
from the debug overlay and produces multi-panel plots (detection
rate, plane offset, mask occupancy, fail-reason breakdown). Push
commits **only** the digest artifacts; raw mcap stays on the Pi
(gitignored).

**Demo-run bags.** `/demo/start` with label dropdown
(`demo1` / `demo2` / `demo3`). Records ball state, ref, IMU,
control_cmd/result, status, leg encoders/currents, and the new
`/ball_track/diagnostic` topic (per-tick FSM phase + commanded
tilts). Digest produces a 7-row plot: 2D trajectory + error magnitude
+ x/y tracking + KF velocity + IMU tilt + leg currents + **phase
strip** (color-coded scatter showing which FSM phase the controller
was in at each tick, with commanded tilts overlaid).

### GUI panels

- **Gains & Control Parameters** (Demos column): live-editable PID
  gains + bang-bang thresholds + algorithm dropdown. Mirrors
  Level-PI Tuning panel exactly. Save writes to YAML and reloads
  in-memory; no service restart.
- **Demo Run Bag Recording**: same UX as IVA / vision-debug. Per-bag
  inline summary stats + digest/push/PNG/delete.
- **Vision Debug detector toggle**: V0 / depth-blob / both, with live
  readouts for each detector's ball position in mm.

### Telemetry

- `/ball_track/diagnostic` (Float32MultiArray, 11 fields) — FSM
  phase, commanded tilts, error/velocity/unit-error vector. Bagged
  by demo recorder. **Color-coded phase strip** in the digest is what
  makes bang-bang tuneable.
- `/oak/ball/depth/diagnostic` (Float32MultiArray, 22 fields) — algo
  internals + frame-wide depth distribution + probe-style pose vs
  measured. Lets the digest answer "why didn't depth-blob detect?"
  without raw images.
- `[health]` log lines from oak_driver + ball_localizer (5 s cadence)
  — pose count, V0/depth detection rates, mask state, intrinsics
  state, gate accept/reject counts.

### Pose persistence

`~/.stewart_control_state.json` saves last-commanded
`(current_xyz, current_rpy)` on every mutation. GUI Z sliders read
from `/status` on connect, so they show the last setpoint instead of
`0` after a service restart.

---

## Specific bugs found and fixed

A non-exhaustive list of the more painful ones, in roughly
chronological order:

- **OAK USB power browned out the Pi 5** — solved with a separate
  wall charger via splitter into the Spotpear hub.
- **OAK depth aligned to CAM_A required `setLeftRightCheck(True)`** —
  pipeline halted at error 180 otherwise.
- **IR projector contaminating mono cameras broke ArUco corner
  detection** — moved ArUco to RGB stream (memory
  `feedback_oak_ir_vs_aruco`).
- **stereoRectify K diverged from DepthAI's internal K by 13.58 px**
  — solved by publishing raw mono left + raw factory K + all 14
  distortion coefficients (rational model).
- **stewart_control_node startup deadlock** —
  `_open_bus_and_start_threads` held bus_lock then called read_sdo
  which acquired the same lock. Fix: release lock before SDO reads.
- **CBOR-raw broke the GUI entirely** — rosbridge on Pi/Kilted didn't
  honor it, zero frames arrived. Reverted; kept Blob URL +
  JPEG-quality drop.
- **Double-const SyntaxError** — added `const now = performance.now()`
  for a diagnostic log when the same name was already declared lower
  in the same function. JS refuses to parse the entire script,
  killing the rosbridge connection.
- **Save-hang on gains panel** — `ball_track_save_gains` reply
  handler nested inside `if (r.cmd.startsWith('level_'))` gate.
  Pulled out into a sibling block.
- **Demo 2 auto-targeted center on Start** — the bolt at center made
  this dangerous. Fixed: `_compute_ref` returns None when
  `x_mm`/`y_mm` not in params; ball-fall recovery exempts ref-stale
  (only counts state-stale).
- **Phantom-ball propagation through three caching layers** —
  ball_localizer cached V0 forever, KF predicted forever, GUI showed
  the dot forever. Fixed at all three layers with freshness gates.

---

## Lessons learned (the hard-won ones)

These are insights that would have saved hours had we known them up
front:

1. **Frame mismatches are silent.** When the ArUco board's frame is
   rotated relative to the IMU body frame, no combination of
   `pitch_sign / roll_sign` can compensate. The signs only mirror,
   not rotate. The Procrustes fit from an IVA sweep is the only way
   to know — a small `det=±1` and a `rotation_deg` near a multiple of
   90° is the smoking gun.

2. **Stereo on smooth surfaces fails predictably.** Carbon fiber +
   vinyl + foam ball + IR projector = "stereo gives correct depth at
   marker edges but invalid (or wrong-matched to background) on the
   platform interior." The pattern manifests as 80%+ "valid" pixels
   in the mask but a median height pegged at the wrong sign. The fix
   is either to add texture or to abandon stereo for the
   platform-tracking task.

3. **Camera close-range limits aren't documented prominently.** The
   OAK's default stereo cutoff of ~632 mm cost us multiple bags
   before we noticed `depth_min=632` constant — a giveaway that the
   platform was below the close-range limit. `ExtendedDisparity`
   doubles the range, costs only a small compute increase.

4. **Vision-to-command latency dominates fast control.** ~100 ms from
   photon to motor is enough to BRAKE 30 mm late at 300 mm/s ball
   speed. Lookahead by `velocity × latency` is a simple, big-win
   compensation. The user's instinct that "the platform can't
   compensate fast enough" was correct — the fix is computing phase
   decisions on the lead-extrapolated position.

5. **Static vs kinetic friction asymmetry matters.** A foam ball at
   rest on vinyl needs more tilt to start moving than the same ball
   moving at 500 mm/s needs to brake — but only because the ball at
   rest is *immobile* below the static-friction tilt threshold. Once
   moving, kinetic friction is much lower. This means ACCEL needs
   *high* tilt to break stiction, while BRAKE on a fast-rolling ball
   *also* needs high tilt to overcome the ball's kinetic energy.
   Symmetric bang-bang ignores this asymmetry; separating
   `accel_tilt_deg` from `brake_tilt_deg` (with stiction breakthrough
   as a third level) captures the actual physics.

6. **Closing the inner tilt loop on IMU is essential.** Without it,
   BALL_TRACK is open-loop on platform tilt — commanded tilt and
   actual tilt diverge by static plant bias (asymmetric leg backlash,
   gravity sag, IK approximations). The ball drifts to the corner
   where commanded == actual − bias, and the controller can't escape
   because it doesn't know about the bias.

7. **Phase telemetry is mandatory for FSM debugging.** Without
   `/ball_track/diagnostic`, "stuck in ACCEL forever (stiction)" and
   "rapidly chattering ACCEL↔COAST↔BRAKE (thresholds wrong)" both
   look like "ball doesn't track" in the bag. The phase strip
   distinguishes them in one glance.

8. **Bag-driven tuning beats guess-and-check.** Live-editable gains
   via the GUI + auto-recorded demo bags + auto-digest produces a
   tuning timeline in `git log`. Every gain change becomes a diff,
   every run becomes a digest commit. Far more disciplined than
   tweaking gains and watching by eye.

9. **rosbridge image streaming is bandwidth-heavy.** JSON-array
   serialization of `uint8[]` data inflates 7×; JPEG decoding via
   `String.fromCharCode + btoa` blocks the JS event loop. Both
   compound. Fixes: lower JPEG quality at the publisher, throttle at
   the bridge, Blob URLs at the browser.

10. **The ball-on-plate problem is fundamentally 2D + planar-
    constraint.** Solving it via 3D depth perception is overkill (and
    brittle on smooth surfaces). V0 (color) + ArUco (plane) +
    ray-plane intersection gives sub-mm position with a single
    camera. The depth-blob detector is a research artifact, not a
    production necessity.

---

## What's still open

- **Demo 1 (orbit) tuning** with the new infrastructure — pending
  another round with the alignment, latency lookahead, brake_tilt,
  and z-compensation all live.
- **Demo 3 (path) ref_generator** — `BALL_TRACK_PATH` is still
  stubbed, returns the first waypoint only. Arc-length traversal at
  speed (with the same closest-waypoint snap as Demo 1) is a
  self-contained ~30-line implementation.
- **End-to-end latency measurement** — a "ball-tap" digest analyzer
  that times from V0 detection to motor response. Enables tuning
  `control_latency_s` empirically instead of using the 100 ms
  estimate.
- **Stereo on the deck (research)** — whether adding fine-pitched
  texture (printed dot pattern) makes depth-blob useful. Not
  currently a priority.

---

This represents about a day's worth of architecture changes plus a
roughly equivalent amount of bug investigation. The diagnostic
infrastructure (bag pipelines + phase telemetry + alignment auto-cal)
is the most durable contribution — the controller-side knobs (PID vs
bang-bang, brake_tilt, z_comp) are tuneable in 5-10 bags now that we
know what we're looking at.
