# Level-loop PI Tuning Plan

**Date drafted:** 2026-04-26  
**Last updated:** 2026-04-27 (post ODrive recommutation; Z range corrected,
auto-sweep added, settle-detector replaced with Z-reached gate)  
**Owner:** Luke  
**Status:** Phase 1 + 2 ready to implement; ODrive recommutation done.

## Why

The level loop in `stewart_control_node._level_run` (KP=0.7, KI=0.2,
deadband=0.05°, rate-limit=0.2°/iter, MAX_CORR=5°, alpha=0.3) does not
settle inside the 0.1° error band. Symptom is a slow limit cycle around
the target, not high-frequency jitter, so IMU noise (sub-millidegree on
the MTi-630) is ruled out. Hand-tuning so far has not converged.
We want a data-driven tuning loop instead.

## Operating regime

The closed-loop ball demos run at **Z = 30–50 mm** (low-Z =
short legs = high leverage = stiffer plant). Tuning therefore targets
this regime, not mid-stroke. Characterization sweeps slightly outside
operating range: **Z = 25–55 mm** so the edges of the operating window
are also covered. Don't extrapolate tuning results from outside this
range — at high Z the platform behaves differently and the data isn't
relevant to the demos.

## Pre-conditions (cleared 2026-04-27)

- [x] **ODrive motor + encoder offset recommutation** done on every node.
  Tuning a controller against a broken plant gives gains that are wrong
  the moment the plant is fixed.
- [x] **Pi deploy path confirmed** (3 commands after SSH — see
  `PI_MIGRATION.md` "Syncing code changes to the Pi").

## Acceptance criteria

Stop tuning when **both** of these hold at every Z in the operating range:

- Settling time into the 0.1° band ≤ 0.5 s after a 1° step
- Steady-state RMS error ≤ 0.05° over 30 s with no input

If gains can't satisfy this at all of Z = 30, 40, 50 mm with one set,
the answer is gain-scheduling on Z (small lookup table), not better
tuning at one point.

## Phase 1 — Instrument the loop (control_node)

Publish a single dense `/level_diag` topic at 50 Hz with the full
controller state per tick. Don't try to recover this from existing
topics (sampled at different rates, can't align integrator state).

**Custom message:** `jugglebot_interfaces/msg/LevelDiag.msg`. Fields:

- `t_imu` (IMU receive monotonic), `t_tick` (tick start)
- `roll`, `pitch` (raw IMU)
- `target_roll`, `target_pitch` (commanded — `level_ref` + `current_rpy`
  + `step_offset`)
- `err_r`, `err_p`, `err_r_filt`, `err_p_filt`
- `integ_r`, `integ_p`
- `pi_out_r`, `pi_out_p` (= `-err*KP + integ` — *unclipped*)
- `corr_r`, `corr_p` (= `tilt_*_corr` after rate-limit + MAX_CORR clamp)
- `clip_flags` (bitfield: rate-limited / max-corr clipped / deadband /
  integrator clamped)
- `motor_targets[6]` (commanded leg pos turns), `leg_enc[6]` (actual
  pos turns), `leg_iq[6]` (measured Iq A)
- `dt_actual` (loop jitter — important if >> 20 ms)
- `target_xyzrpy[6]` (commanded platform pose this tick — captures Z so
  bags from different operating points are self-describing)

## Phase 2 — Bag recording in the GUI

**New panel** in `index.html`: "Level-PI Tuning" (do not repurpose the
existing "Active Stabilization (Ball-Hold)" scaffold — that's for the
v10 BALL_HOLD feature, a different thing). The new panel has three
subsections.

### 2a. Single test

Record/Stop toggle + name input + free-text notes input + size/duration
readout, plus a Step-test button (programmatic +1° roll for 3 s, back
to 0 — repeatable so step responses are comparable across runs).

`/control_cmd` JSON commands:

- `{cmd: 'level_record_start', name: <str>, notes: <str>}` → spawns
  `ros2 bag record -o ~/stable_bot_bags/<UTC>_<name>/ /level_diag
  /platform_rpy /leg_encoders /leg_currents /control_cmd`. Writes a
  sidecar `notes.json` next to the bag with: gains snapshot from
  `level_gains.yaml`, free-text note, git SHA of running build,
  start-time UTC.
- `{cmd: 'level_record_stop'}` → SIGINT the child, return path/duration/size.
- `{cmd: 'level_zero_integrator'}` → zero `integ_r` and `integ_p` in
  `_level_run` (cleaner than disarm-and-snap; no mechanical transient).
- `{cmd: 'level_step_test', amp_deg: 1.0, hold_s: 3.0, count: 1}` →
  drive `self.level_step_offset_roll` as a square wave.

### 2b. Auto-sweep

User-supplied Z-min, Z-max, **Z-step**, plus step_count, step_amp_deg,
step_hold_s, baseline_s, prefix. Estimated-duration readout. Big green
**Start** button, big red **Abort** button. Live progress display.

Per-Z-level routine:

1. Command pose `(0, 0, Z, level)` via existing `set_pose` mechanism.
2. **Z-reached gate** (replaces "wait for level" — that won't work
   because the platform currently doesn't settle and tuning is what fixes
   that). Pre-conditions:
   - Every leg encoder position within 0.05 turns of its IK target,
     **and**
   - max-min spread of each leg's position over the last 0.5 s < 0.05
     turns (i.e. legs are hovering around target, not slewing).
   - Hard timeout 5 s; abort sweep if it doesn't trip.
3. Issue `level_zero_integrator` (start every test from clean integrator
   state).
4. Start a bag named `<UTC>_<prefix>_z{Z:02d}/`.
5. **Baseline:** 30 s with no input. Yes — capture the limit-cycle
   behaviour. As tuning converges, the limit cycle shrinks; the variance
   of the baseline RMS across iterations is itself a progress signal.
6. **Step battery:** `step_count` step tests. Each: `level_step_offset_roll
   = +amp_deg` for `step_hold_s`, then `0` for `step_hold_s` rest.
   Default `step_count = 8` (up from plan v1's 5) — averaging across
   trials washes out the random limit-cycle phase at step start.
7. Stop bag.
8. Move to next Z, repeat.

Whole sweep is wrapped in a parent dir
`~/stable_bot_bags/sweep_<UTC>_<prefix>/` with a `manifest.json` listing
every child bag, all parameters, the running git SHA, start/end UTC.
The analyzer (Phase 3) reads the manifest to build the
Z-vs-{settling, RMS, overshoot} chart that decides gain-scheduling.

**Authority / safety:**

- Z range clamps come from `leg_limits.yaml` via the IK. GUI fields
  turn red and disable Start if a value would put any leg outside its
  safe stroke.
- `step_amp_deg` hard-capped at 2° in the dispatcher (typo guard).
- `/control_cmd` `level_auto_sweep_abort` polled by the sweep loop on
  a 100 ms timer.
- `/level_record_state` (JSON string topic) at 5 Hz with current state
  (`idle` / `single_recording` / `sweep_running` / `aborted`), test
  index, Z value, step index, elapsed time. If the topic stops
  publishing for >2 s the GUI shows a stale-state warning.
- First-sweep-this-session advisory modal in the GUI: "first sweep
  since page load — confirm you're at the bench". One-time, dismissible.

### 2c. Recent bags

`gui_server.py` GET `/bags` endpoint enumerates `~/stable_bot_bags/`
(per-test bags + sweep dirs, with size + duration from sidecar/manifest).
Sweep dirs are collapsible. Per-bag download (HTTP file serve) + delete
(POST with confirmation, path-traversal guarded).

## Phase 3 — Offline analysis script

`scripts/analyze_level_bag.py` using `rosbag2_py` + matplotlib + numpy:

**Plots:**
- `err_filt` vs time (roll + pitch overlaid)
- `integ` vs time
- `corr` vs time, with the unclipped `pi_out` overlaid (when they
  diverge, the controller is hitting saturation)
- FFT of `err_filt` to find the limit-cycle frequency

**Step metrics** (averaged across all step trials in a bag, with each
trial's pre-step mean subtracted before averaging):
- Rise time (10→90 %), peak overshoot (%), 5 %-settling time,
  1 %-settling time (= settling into the 0.1° band)
- Steady-state offset after settle

**Steady-state stats** (last 5 s of baseline):
- Mean, std-dev, peak-to-peak of `err_filt`

**Saturation count:**
- % of ticks where `clip_flags != 0`. High % = controller is fighting
  the rate-limiter / clamp; gains too aggressive.

**Sweep manifest mode** — given a `sweep_*/manifest.json`, plot Z vs.
each metric. This is the single chart that decides gain-scheduling.

## Phase 4 — Tuning protocol

1. **Baseline bag** at Z = 40 mm with current gains (KP=0.7, KI=0.2).
   Measure RMS error and dominant frequency from the FFT. This is the
   *zero point* — every later run is compared to it.
2. **Step bag** at Z = 40 mm: 8 step tests, averaged.
3. **From the step response:**
   - Overshoot > 10 % and oscillates: KP too high, or KP/KI ratio
     wrong. Try KI = KP/3 instead of current KP/3.5.
   - Sluggish with steady-state error: KI too low; raise toward KP/2.
   - High-freq oscillation (> 3 Hz): rate-limit can't catch the
     controller's bandwidth — lower KP first.
   - Steady-state offset > 0.05° but baseline RMS small: integrator
     clipped; check `clip_flags` and `integ` plot.
   - Iterate, re-record, compare. Stop when settling-time(0.1°) ≤ 0.5 s
     and steady-state RMS ≤ 0.05° at Z=40.
4. **Run an auto-sweep** at Z = 25–55, 5 mm step. If the same gains hold
   across the operating range (30–50), done. If not, decide between
   (a) one conservative gain set, (b) gain-scheduling on Z.
5. Lock final gains in `config/level_gains.yaml`.

## Build order

1. Phase 1 + 2 together → push → 3-command Pi sync → smoke-test panel
2. Phase 3 script (separate commit, no deploy needed)
3. Phase 4 (iterative, user-driven; I just maintain the analyzer)

## Rough size

- Phase 1: `LevelDiag.msg` + publisher + `level_gains.yaml` loader +
  step-offset hook in `_level_run` — ~120 lines control node.
- Phase 2a + 2b + 2c combined: `_control_cmd_cb` dispatcher additions +
  bag-recording subprocess management + auto-sweep thread +
  `/level_record_state` publisher + Z-reached gate — ~400 lines.
  GUI panel ~250 lines HTML/JS. `gui_server.py` `/bags` endpoint ~80 lines.
- Phase 3: ~300 lines analyzer.
- Custom msg requires rebuilding `jugglebot_interfaces` (one extra
  colcon step on the Pi).
