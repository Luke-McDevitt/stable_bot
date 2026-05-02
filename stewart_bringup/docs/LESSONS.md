# Stable-Bot — compiled lessons learned

Everything I learned getting this thing from "scripted open-loop
rolling ball" to "click-to-goto with 0.5 mm tracking error." Terse
on purpose. Each bullet is a rule that prevents a specific mistake
I made or saw made; the parenthetical is the doc with the longer
story if you want it.

---

## ODrive / motors / CAN

- Every parameter the loop depends on must be (a) persisted to flash
  via the configurator AND (b) re-asserted at runtime on every
  level-on. Neither alone is enough — flash gets overwritten by
  recalibration, runtime gets reset by disarm.
  (level_loop_lessons_learned.md §1)
- `wL_FF_enable`, `control_mode`, `current_soft_max`, `vel_limit` all
  silently revert to factory defaults via the WebGUI's "Run
  Configuration Script." Same recurring bug shape.
- All 6 ODrives must be on the **same firmware build**. Different
  builds have different endpoint tables; CAN SDO writes to one
  endpoint id can be the wrong field on a sibling drive.
  (memory: feedback_odrive_firmware_uniformity)
- Don't `time.sleep()` between `Set_Axis_State=8` and the first
  motion command. The watchdog disarms silently with
  `active_errors=0x01000000`. (memory: feedback_odrive_watchdog)
- Drive showing `0483:df11` in dmesg is in STM ROM DFU, not bricked.
  Flip the physical RUN/DFU switch on hw 4.4 + power-cycle.
  (memory: feedback_odrive_pro_dfu_run_switch)
- `gs_usb` USB-CAN adapter (VID:PID 1d50:606f) is in the WSL2 kernel
  built-in. Use the netlink bring-up path, not slcand.
  (memory: project_jugglebot_can_adapter)
- Set `vel_limit` at three independent layers (firmware, Python CAN,
  GUI sliders) — the redundancy is the point.
- ODrive's Python lib (0.6.10) has no CAN transport. Use Tx_SDO
  via python-can. Endpoint metadata is at
  `parent._<name>_property._info` in `sync_tree`, not on the leaf.
  (memory: feedback_odrive_can_param_access)

## ROS 2 / WSL2 / Pi

- Don't set `ROS_LOCALHOST_ONLY=1` or
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in launches — silently
  hides topics from CLI tools that don't set the same vars.
  (memory: feedback_ros2_discovery_env)
- Pi build env: bashrc only sources the overlay → PYTHONPATH empty
  → `ament_package` import fails. Source `/opt/ros/kilted/setup.bash`
  first. ntrip won't build (broken upstream); apt name is
  `ros-kilted-ament-package` not `python3-ament-package`.
  (memory: feedback_pi_build_env)
- `pi_deploy.sh` is the only deploy path that works:
  `--packages-select jugglebot_interfaces stewart_bringup` (skip
  ntrip), then restart **both** `stable_bot.service` AND
  `stable_bot_gui.service`. Pull alone is not enough — the install/
  tree won't reflect symlink-source changes without colcon build.
  (memory: feedback_pi_pull_restart_recipe)
- USB devices reach WSL via `usbipd-win`, not natively. `attach
  --wsl --busid <X>` from a Windows admin shell per session;
  `bind` is one-time.
  (CLAUDE.md)
- SQLite databases belong on the Linux side (`~/...`), not under
  `/mnt/c/...` — FTS5 trips over Windows-mount I/O.
  (CLAUDE.md)
- Pi 5 USB-A 3.0 is the *blue* port. USB-C is power-only.
  (closed_loop_ball_demos_usage.md)

## Vision pipeline

- `setFps(N)` is a *ceiling*, not a floor. Auto-exposure is allowed
  to pick any integration time up to the frame interval; in dim
  rooms it picks 30-67 ms and the framerate floor-divides. Manual
  exposure (`setManualExposure(8000, 800)`) + a real light source
  takes you from 4 Hz to 30 Hz with the same code.
  (controls_journey.md)
- `setAutoExposureLimit` should work but doesn't on depthai 2.32 +
  current OAK firmware. Use `setManualExposure`.
- USB sustained throughput on OAK-D Pro AF is ~30-50 MB/s, far
  below the theoretical 400 MB/s SuperSpeed. At 540p × 1.5 MB/frame
  this caps at 20-30 Hz — that's the real ceiling, not the FPS
  setting.
- IR projector dot-speckle on mono cameras breaks ArUco corner
  detection. Run ArUco on the RGB stream (IR-cut filter); mono is
  for stereo only when IR is on.
  (memory: feedback_oak_ir_vs_aruco)
- OAK's default stereo cutoff is ~632 mm. Platform sits at ~600 mm.
  `setExtendedDisparity(True)` doubles the disparity range and
  halves the minimum to ~316 mm.
  (vision_control_tuning_lessons.md)
- Stereo on smooth surfaces (carbon fiber + vinyl + foam ball + IR)
  fails predictably: 80%+ "valid" pixels but median height pegged
  at the wrong sign. Either add texture or abandon stereo for
  platform-tracking.
- HSV detection on a moving ball is brittle. Each ~2× of exposure
  time roughly doubles per-pixel light AND doubles motion smear.
  Trade-off: shorter exposure + brighter ambient is the dominant
  strategy.
- YOLOv8n had ~20% better stationary-frame benchmarks than cv2 HSV
  but lost the ball almost immediately during demos with motion.
  **Don't trust offline accuracy benchmarks for closed-loop
  control quality. Always A/B with the runtime backend selector.**
  (step_id_tuning_lessons.md TL;DR #3)
- rosbridge JSON-array serialization of `uint8[]` inflates 7×; JPEG
  decoding via `String.fromCharCode + btoa` blocks the JS event
  loop. Both compound. Fixes: lower JPEG quality at the publisher,
  throttle at the bridge, Blob URLs at the browser.
- End-to-end vision-to-command latency is ~100 ms (capture, USB,
  KF, control loop). At 300 mm/s ball speed that's 30+ mm of drift.
  Use latency lookahead in the controller (`ex_lead = ex + edot · Td`).

## Frames & IK

- The geometric IK in `_compute_motor_targets` is wrong by **45-62°
  rotation per leg** vs the empirical Jacobian. Mean magnitude
  error: 130% on roll, 450% on pitch. Tuning gains on top of a
  broken IK is wasted effort.
  (level_loop_lessons_learned.md §2,
   demo2_shipping_journey_2026-05-02.md TL;DR #1)
- ArUco frame ≠ IMU body frame. Procrustes fit between them on
  this hardware shows a ~155° rotation. Sign-flips on individual
  axes cannot compensate — only a 2D rotation can.
  (controls_journey.md, vision_control_tuning_lessons.md)
- The `aruco_imu_alignment.yaml` rotation must be applied to **both**
  `/ball_state` (in `ball_localizer_node`) AND `/ball_ref` (in
  `ref_generator_node`). The controller subtracts ref from state;
  if they're in different frames, the result is rotated garbage.
  (controls_journey.md)
- Right-hand-rule rotations are not symmetric across axes.
  `Ry(+p)` tilts +X edge DOWN (ball slides +X). `Rx(+r)` tilts +Y
  edge UP (ball slides −Y). `pitch_sign = -1, roll_sign = +1` is
  the correct combination given the standard math convention. Don't
  trust docstrings against formulas — derive from the rotation
  matrices.
  (controls_journey.md)
- `_do_set_pose` and the level loop must use the same IK path. If
  one uses `empirical_ik` and the other doesn't, every command
  through the wrong path is silently rotated by the platform's
  leg-mount frame error.
  (demo2_shipping_journey_2026-05-02.md TL;DR #1)
- For BALL_TRACK on a tilted plate: rotate the platform UNDER the
  ball, not about its geometric center. `z_cmd = z_hold + px·sin(p)
  − py·sin(r)` keeps the ball's world-frame height invariant under
  tilt commands.
  (vision_control_tuning_lessons.md)

## Control theory — PID, bang-bang, stiction

- **`max_tilt` is the in-flight damping cap, not the
  controller-can-ever-output cap.** Stiction breakaway needs higher
  authority than in-flight damping. Decouple via
  `stiction_ramp_max_deg`.
- **Two physically-distinct tilt thresholds, not one.** Stiction
  breakaway `θ_s` (~2.14° on this hw, hardware property) and
  orbital braking `θ_orb` (depends on instantaneous ball state).
  At rim with v=500 mm/s, `θ_orb ≈ 7.3°`. `max_tilt < θ_orb` means
  the controller is physically incapable of recovering from that
  orbit. Recommended floor: `max_tilt = max(θ_s + 2.5, 8.0)`.
  (step_id_tuning_lessons.md TL;DR #1)
- **PID Kd × v_noise dominates direction when Kd is non-trivial.**
  Math: Kd dominates Kp when `|v| > (Kp/Kd) · |err|`. With raw
  noisy KF velocity reading 60+ mm/s on a stationary ball, this
  threshold is hit during normal motion. Filter the velocity that
  Kd reads (`kd_v_tau_s` LPF) — orbital motion (~1 Hz) passes,
  frame-rate noise (>5 Hz) gets attenuated 20-30 dB.
  (demo2_shipping_journey_2026-05-02.md TL;DR #2)
- **Bang-bang has no tangential damping.** FSM phase logic is
  keyed on `v_toward` (radial). For an orbiting ball, `v_toward ≈ 0`
  → FSM picks ACCELERATE → tilts toward target → adds inward
  force only → tangential velocity is conserved → orbit persists.
  PID's `Kd × v` opposes all directions; that's why PID kills
  orbits and bang-bang doesn't.
  (demo2_shipping_journey_2026-05-02.md TL;DR #3)
- **Slam-to-max stiction relief is wrong on low-friction
  hardware.** Gives the ball more energy than the controller can
  damp. Use a slow ramp from `θ_s + small_margin` upward instead;
  ball breaks loose at near-zero velocity.
- **`ramp_rate × timeout` must exceed `ramp_max - ramp_start`.**
  Otherwise the stiction ramp times out before reaching its
  configured ceiling. Operator's `0.35 × 8 = 2.8°` couldn't reach
  `5.0°`. Hours of tuning vanished into this single equation.
  (demo2_shipping_journey_2026-05-02.md TL;DR #5)
- **Position-delta is far cleaner than velocity-threshold for
  binary detection.** A stationary ball jitters by 1-2 mm in
  position; velocity is the derivative and amplifies that to
  30-60 mm/s. Use 5-10 mm position delta from a snapshot for
  "did the ball actually move." (TL;DR #4)
- **Hysteresis on motion detection: LPF + consecutive-tick
  counter both required.** Single-tick noise spikes are filtered
  by hysteresis; sustained noise events are filtered by LPF.
  Each on its own is insufficient on this hardware.
- **Cascade-bypass tradeoff**: `BALL_TRACK` commands tilts
  directly via `_do_set_pose` (no inner level loop). Faster
  transient response, but plant biases (leg backlash, IK
  approximations, gravity sag) that the level loop used to
  absorb now show up as ball-position bias. Compensated by the
  outer integrator — slower bias rejection, faster transient.
  (step_id_tuning_lessons.md)
- **Closing the inner tilt loop on IMU is essential UNLESS the
  outer loop is bandwidth-limited by it.** Original design used
  level-PI as the inner loop tracking `current_rpy`. Level-PI's
  500 ms step response capped BALL_TRACK at `ωn < 1 rad/s`. The
  analytic recommendation engine's preferred `ωn = 4 rad/s` was
  incompatible with the cascaded plant. Bypass was the right call.
- **Vision-latency lookahead** (`ex_lead = ex + edot · Td`) is
  mathematically equivalent to bumping Kd by `Kp · Td`. Restores
  the predicted closed-loop poles when an analytic Kp/Kd pair was
  computed assuming zero dead-time.
- **Plant ID has 30-50% CV** across replicates on this hardware,
  driven by vision noise. Don't trust single-trial recommendations.
  (step_id_tuning_lessons.md TL;DR #4)

## Stiction & friction (low-μ-specific)

- `θ_s` measured by STEP_ID's stiction phase varies across the
  deck. Set `stiction_ramp_start_deg` above the measurement and
  let the ramp climb from there.
- `μ_s = 0.037` (foam on vinyl) is too low for friction alone to
  decay tangential velocity. Bang-bang's COAST relies on friction
  decay; on this hardware it doesn't work.
- Static friction overcoming = high tilt; kinetic friction (rolling
  ball) = much lower. Asymmetric `accel_tilt_deg` vs
  `brake_tilt_deg` captures this — a moving ball needs MORE tilt
  to brake than a still ball needed to start.
- `max_tilt` must be **above** `θ_s` so PID can *maintain* motion
  between stiction-ramp pulses, not just below the runaway
  threshold. Sweet spot on this hw: 1.2-1.5°.
  (demo2_shipping_journey_2026-05-02.md "max_tilt = 1.0 killed PID")
- "Pulse-coast-pulse" walker pattern beats "one big push" on
  low-friction hardware. Each ramp pulse just barely breaks
  stiction → ball moves a small distance → PID damps → ball stops
  → next pulse fires. Tunable via `stiction_pos_delta_mm` and
  `stiction_ramp_timeout_s`.
- Don't blame friction for level-loop drift before checking
  controls. Operator has held ±0.1° on this hw before; track-loss
  on previously-working configs is a control regression, not
  mechanical stick-slip.
  (memory: feedback_dont_default_to_friction_diagnosis)

## Vision noise — specific patterns

- Single-frame V0 false-positives spike `vx, vy` to 4000+ mm/s on
  a foam ball that physically tops out at 800. `Kd × v_spike`
  saturates `max_tilt` in random directions. Cap `|v|` at the
  physical max before applying Kd
  (`BT_MAX_BALL_VEL_MM_S = 800.0`).
- Trajectory looks "flat then jumps" when V0 loses the ball during
  fast motion (HSV failure under motion blur) and re-acquires at
  a new position 50+ mm later.
- `/oak/health.v0_arr_hz ≈ 27`, `v0_pub_hz ≈ 12.5`. Publish rate is
  capped somewhere downstream; if you can lift it to match
  arrival, effective vision latency halves.
- Off-platform false positives: tightening HSV S/V bounds for
  motion-blur tolerance loosens the discrimination from off-deck
  shadows. Tighten the platform mask radius (0.220 → 0.190 m)
  to compensate.

## Tooling & methodology

- **Hand-tuning with good comparison tools beats auto-tuning.**
  Auto-tuner (Bayesian/hill-climb) failed: 27 trials, 0 acceptances
  because random-target fitness noise > gain effect. Operator hand-
  tuning + `compare_demo_bags.py` succeeded by sorting all runs
  by metric and reading the gain-vs-outcome correlation off the
  table.
- **Bag-driven tuning beats guess-and-check.** Live-editable
  gains via the GUI + auto-recorded demo bags + auto-digest
  produces a tuning timeline in `git log`. Every gain change
  becomes a diff, every run a digest commit.
- **Phase telemetry is mandatory for FSM debugging.** Without
  `/ball_track/diagnostic`, "stuck in ACCEL forever (stiction)"
  and "rapidly chattering ACCEL↔COAST↔BRAKE (thresholds wrong)"
  both look like "ball doesn't track" in the bag.
- **Find code-path inconsistencies before tuning gains.** Hours
  spent tuning while a frame-rotation IK bug silently corrupted
  every tilt command. **Always grep "X is used in N places — does
  it use the same auxiliary state in each?" before assuming the
  algorithm is correct.**
  (demo2_shipping_journey_2026-05-02.md durable lesson #1)
- **Velocity is noisier than position by `1/dt`.** For binary
  detection, use position. For continuous control, use filtered
  velocity. For lookahead, use either.
- **Live KF tuning is downstream**. No amount of Q/R tweaking
  helps if the underlying frame is wrong (IK fix) or the
  controller uses stale data (lookahead). Fix the path first.
- **Run the offline non-causal smoother on bags** for "what was
  the ball *actually* doing" — the live KF is causal-only and
  noise-smoothed only from the past. Sav-Gol with window 11,
  order 3 is enough.
- **Compare-script columns must be diff-mode**. Showing the 24
  fields that didn't change across a sweep alongside the 8 that
  did is unreadable. `--diff` mode or it's noise.
- **dates in planning docs absolute, not relative.** `2026-04-18`
  not "Thursday." Future-you doesn't remember which Thursday.
  (CLAUDE.md)

## Architectural decisions that paid off

- **GUI panel for live-editable gains.** Save writes YAML and
  reloads in-memory; no service restart. Iteration cycle drops
  from ~30 s (restart + warm-up) to ~3 s.
- **Two parallel ball detectors (V0 cv2-HSV + depth-blob).**
  Both publish independent topics; the GUI shows them as
  orange/cyan circles for direct comparison. Catches detector
  failures that single-detector A/B can't.
- **Demo-run bag recording with auto-digest.** `/demo/start` →
  rosbag2 → digest script → digest.summary.json + digest.png +
  next_step recommendation. The whole tuning loop is automated
  except for the gain choice.
- **`/oak/health` and `/oak/config` topics bagged every 5 s.**
  Demo bags self-describe — the digest knows what the camera was
  set to during the run. Lets the digest correlate behavior
  changes with config tweaks.
- **`~/.stewart_control_state.json` saves last pose on every
  mutation.** GUI Z sliders read from `/status` on connect, so
  they show the last setpoint instead of `0` after a service
  restart. Removes the "wait, what Z am I at?" class of confusion.
- **Phase code in the diagnostic topic** (`/ball_track/diagnostic`
  field 0). Color-coded phase strip in the digest is what makes
  bang-bang tuneable. Bang-bang's "stuck in ACCEL forever" vs
  "chattering ACCEL↔BRAKE" are the same gross outcome but the
  phase strip distinguishes them in one glance.

## Debugging discipline

- **Frame mismatches are silent.** No combination of `pitch_sign /
  roll_sign` can compensate for a rotated frame. The Procrustes
  fit from an IVA sweep is the only way to know — `det = ±1` and
  `rotation_deg` near a multiple of 90° is the smoking gun.
- **YAML save in the GUI strips comments.** `yaml.safe_dump` emits
  alphabetized keys with no comments. Keep canonical comments in
  this doc; commit them back to the YAML when convenient.
- **`set -u` in bash trips ROS setup scripts.** They reference
  `AMENT_TRACE_SETUP_FILES` etc without defaulting them. Use
  `set -eo pipefail`, drop the `-u`.
- **`pip install` on Ubuntu 24.04 / Python 3.12** is "externally
  managed" (PEP 668). Build artifacts on a dev machine and commit
  them, use `pipx`, or `pip install --break-system-packages` if
  you really must.
- **`stewart_vision` is a separate package from `stewart_bringup`.**
  When editing `oak_driver_node.py` / `ball_localizer_node.py` /
  `ref_generator_node.py` / `ball_kf_node.py`, include
  `stewart_vision` in the colcon build line or the install/ tree
  won't reflect the source change. Multiple 4-hour debug rounds
  traced to this.
- **Two systemd services**, not one. `stable_bot.service` runs
  the ROS launch; `stable_bot_gui.service` runs `gui_server.py`
  directly. After editing `gui_server.py`,
  `digest_demo_bag.py`, or `web/index.html`, **both** services
  must be restarted.
- **CBOR-raw broke rosbridge entirely** on Pi/Kilted. Zero frames
  arrived. Stuck with JSON + Blob URLs + JPEG quality drop.
- **Double-`const` SyntaxError** killed an entire JS bundle once.
  When the rosbridge connection mysteriously dies, check for
  parse errors in `index.html` first.

## Things that didn't work

- **Auto-tuner (hill climb + Bayesian)**: random-target placement
  produced fitness noise larger than gain effect. 27 trials, 0
  acceptances. (`auto_tuning_plan.md` shipped, never paid off.)
- **YOLOv8n as default vision backend**: 20% better stationary
  benchmarks but lost the ball "almost immediately" during demos.
  Reverted. Available via runtime selector.
- **CBOR-raw image transport**: rosbridge on Kilted didn't honor
  it. Reverted.
- **`OAK_FORCE_V1_ARCH=v8` as default**: same story as YOLOv8n.
- **Aggressive `Kp = 0.077`** from a single-replicate G_eff=224.
  Saturated max_tilt = 2.5° at any error > 32 mm. Drove the ball
  into orbital limit cycle every trial. **High Kp without high
  max_tilt is just bang-bang in disguise.**
- **Slam-to-max stiction relief**: kicked the ball at high
  velocity into orbital limit cycles on every restart. Replaced
  with the slow ramp.
- **`max_tilt = 1.0` for safety**: dropped PID below `θ_s` so PID
  couldn't *maintain* motion between ramp pulses. Demo became a
  130-second sequence of 1 mm hops.
- **`control_latency_s = 0.0`**: turned off the lookahead during
  one tuning pass. Phase margin collapsed; the controller saw
  100 ms-stale data and overshot every motion.
- **Self-calibrating depth-blob plane fit**: median of (expected −
  measured) was driven by stereo-edge artifacts at 1.5 m, not the
  platform interior. Removed; trusting ArUco-derived plane
  directly.
- **Setting per-axis hysteresis at 3 ticks (60 ms) only**: not
  enough for sustained vision-noise events of 100+ ms. Needed
  LPF + consecutive-tick combination.

## The two settled runs

- `20260502T025210Z_demo2`: median tail err **0.5 mm**, settled.
- `20260502T030936Z_demo2`: median tail err **4.2 mm**, settled.

Winning gains preserved verbatim in
`demo2_shipping_journey_2026-05-02.md` "The shipping commits"
section. Full configuration in `ball_track_gains.yaml`.

---

The single biggest unlock was finding that `_do_set_pose` and the
level loop used different IK paths (geometric vs empirical
Jacobian, ~55° rotated from each other). One line of code; half a
day to find. Everything else was iteration on top of getting that
right.
