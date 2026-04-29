# Stable-Bot level-loop control architecture

This is the snapshot of the level-loop architecture as of
2026-04-29 (commit `c21f7af` — last sweep at RMS 0.110°). Companion
to `level_loop_lessons_learned.md`, which explains *why* we ended up
here. This doc explains *what's running* so future-you can come back
in 6 months and read this instead of re-deriving it.

---

## Overview

The level loop closes IMU error → leg-position commands. It runs in
a single thread on the Pi 5 at 200 Hz outer-loop rate. The ODrive
controllers handle their own inner loops at 8 kHz.

```
        ┌──────────────────────────────────────────────────────────┐
        │  IMU (MTi-630 @ 400 Hz)                                  │
        │  → roll, pitch, yaw                                      │
        └──────────────────────────────────────────────────────────┘
                               │ (50 µs latency)
                               ▼
   ┌────────────────────────────────────────────────────────────────┐
   │  _level_run thread (200 Hz outer loop)                         │
   │                                                                │
   │  err = rpy_imu - level_ref                                     │
   │  err_filt = α·err + (1-α)·err_filt    [IIR, scaled by dt]      │
   │                                                                │
   │  if |err_filt| > outer:                                        │
   │       integ += -err_filt · ki · dt    [outside decay zone]     │
   │  elif |err_filt| > deadband:                                   │
   │       graduated blend of integration + decay                   │
   │  else:                                                         │
   │       integ *= integ_decay            [in deadband, full decay]│
   │                                                                │
   │  pi_out = -err_filt · kp + integ                               │
   │  corr = clamp(rate_limit(pi_out), ±max_corr)                   │
   │                                                                │
   │  rpy_cmd = current_rpy + (corr_r, corr_p, 0)                   │
   │  legs = compute_motor_targets(xyz, rpy_cmd, empirical_ik)      │
   │                                                                │
   └────────────────────────────────────────────────────────────────┘
                               │
                               ▼
   ┌────────────────────────────────────────────────────────────────┐
   │  ODriveFeeder (50 Hz cyclic on CAN, runs in its own thread)    │
   │  → Set_Input_Pos to each of 6 drives                           │
   └────────────────────────────────────────────────────────────────┘
                               │
                               ▼
   ┌────────────────────────────────────────────────────────────────┐
   │  ODrive Pro × 6 (POSITION mode, 8 kHz inner loop)              │
   │  → motor commutation, current control, encoder feedback        │
   └────────────────────────────────────────────────────────────────┘
                               │
                               ▼
   ┌────────────────────────────────────────────────────────────────┐
   │  Stewart platform (6 legs, 3 corner-pairs)                     │
   │  → physical roll, pitch, yaw of the platform                   │
   └────────────────────────────────────────────────────────────────┘
                               │
                               ▼  (closes loop via IMU)
```

Total open-loop latency from IMU sample to platform motion is
~10-15 ms. Outer-loop bandwidth is set well below that to avoid
phase-lag oscillation — KP ≈ 0.5, KI ≈ 0.5, dominant closed-loop
period ~6-9 s.

---

## Outer-loop control law (in `_level_run`)

### Gains (in `level_gains.yaml`, GUI-editable)

| gain | typical | meaning |
|---|---|---|
| `kp` | 0.5 | proportional gain (corr/err per °) |
| `ki` | 0.5 | integral gain (corr/(err·s)) |
| `deadband_deg` | 0.005-0.02 | err magnitude below which integrator decays |
| `integ_decay_outer_deg` | 0.05-0.35 | err magnitude above which decay turns off |
| `integ_decay_per_tick_at_50hz` | 0.97-0.99 | decay strength per tick at the reference rate (auto-scaled to actual loop rate) |
| `filter_alpha` | 0.3-0.4 | err IIR low-pass coefficient (auto-scaled to actual rate) |
| `rate_limit_deg_per_iter` | 0.25 | max change in `corr` per tick |
| `max_corr_deg` | 5.0 | saturation cap on `corr` |
| `use_empirical_ik` | 1 | use measured Jacobian (1) or geometric IK (0) |

### Critical features

- **Reference-rate-scaled gains**: `filter_alpha` and
  `rate_limit_deg_per_iter` are interpreted at the rate
  `LEVEL_REF_HZ = 50`. The actual loop rate (set via
  `--level-loop-hz`, default 200) scales them so the *physical*
  closed-loop response is invariant to rate. Tuning a YAML at 50 Hz
  works at 200 Hz with no edits.

- **Graduated integrator decay**: a 3-zone integrator update:
  - `|err| > integ_decay_outer_deg`: full integration, no decay
  - `deadband ≤ |err| ≤ outer`: linear blend of integration ↓ and
    decay ↑
  - `|err| < deadband`: no integration, full decay

  This eliminated the 5-second limit cycle that pure binary
  integrate-or-decay had produced.

- **Anti-windup back-calculation**: if the rate-limit or saturation
  clamp engaged, the integrator is pulled back by the deficit. Keeps
  windup bounded during step disturbances.

- **`level_ref_roll/pitch`** captured from `level_cal.json`. The IMU
  reading at the operator-defined "level" pose is the target. Doesn't
  need recalibration on every run — only after a mechanical change.

---

## Inverse kinematics: empirical Jacobian (`EmpiricalIK`)

This is the single most important thing in this doc. **The geometric
IK in `_compute_motor_targets` was wrong by 100-450%** for this
build. Replaced (for the rotational part) with a measured Jacobian.

### Architecture

`tuning_data/system_id_<UTC>/jacobian.json` contains the empirical
Jacobian from a system-ID run:

```python
J[Z][δ] = 3×6 matrix of (∂rpy / ∂leg_n) at platform pose (0, 0, Z),
          measured by perturbing each leg ±δ and reading IMU.
```

At node startup, `EmpiricalIK._load`:

1. Picks the most recent `system_id_*/jacobian.json` under
   `tuning_data/`
2. For each measured Z, averages J across deltas ≥ 0.05 (skipping
   the small-δ measurements that are stiction-noise-dominated)
3. Computes `pinv(J_z)` — the 6×3 mapping from rpy → leg deltas

At runtime, `EmpiricalIK.rpy_to_leg_deltas(z, rpy)`:

1. Linearly interpolate `pinv(J)` between bracketing measured Z
   values (clamps to nearest endpoint outside the measured range)
2. Multiply: `leg_deltas = pinv_interp @ rpy`

`_compute_motor_targets` with `empirical_ik` provided:

1. **Translation (xyz)** from geometric IK at zero rpy. `xyz` was
   never measured for system-ID, so we still trust the geometric
   model for "where the platform sits" (Z, x, y centering).
2. **Rotation (rpy)** from `pinv(J) @ rpy`. The measured response.
3. Sum + re-clamp to leg soft limits.

This hybrid (geometric translation + empirical rotation) is the IK
the level loop uses by default.

### Why this works

The geometric IK assumes:

- All 6 legs have identical mechanical advantage at each corner
- Leg-to-corner mapping matches the IK's coordinate convention
- Per-leg sign convention (which way is "extension") is uniform
- Mounting angles are exactly as designed

Reality: none of these are exactly true. The empirical Jacobian
captures the actual response — including sign flips on individual
legs vs theory, magnitude scaling per leg, and the (roll ↔ pitch)
cross-coupling that the geometric IK doesn't model.

Cost per tick: ~50 floating-point ops (linear interp + 6×3 matmul).
Cheaper than the trig-based geometric IK it replaced.

---

## Inner loop (per-drive ODrive config)

Each ODrive Pro runs its own 8 kHz inner control loop. The level
loop is the outer envelope; the inner loop handles position-tracking
to the commanded leg positions.

### Critical persistent flash settings

These must be in flash on every drive:

| param | value | path | why |
|---|---|---|---|
| `axis0.controller.config.control_mode` | 3 (POSITION) | id 378 | level loop sends Set_Input_Pos |
| `axis0.controller.config.input_mode` | 1 (PASSTHROUGH) | id 379 | no filter, no traj generator |
| `axis0.config.motor.wL_FF_enable` | True | id 305 | torque feedforward — load-bearing for tracking |
| `axis0.config.motor.current_soft_max` | 12 A | id 315 | enough current to lift the platform under tilt |
| `axis0.controller.config.vel_integrator_gain` | 0.333 | id 382 | inner-velocity I gain |
| `axis0.config.can.encoder_msg_rate_ms` | 2 | id 275 | 500 Hz encoder broadcasts |

Set persistently via `set_odrive_feedforward_via_can.py --apply
--control-mode position --input-mode passthrough --vel-integrator-gain 0.333
--wl-ff true --encoder-rate-ms 2 --current-soft-max 12.0`. This
calls `save_configuration()` per drive so the settings persist to
flash.

### Runtime self-heal in `_prepare_for_level`

Called at every Level ON. SDO-writes (runtime, doesn't touch flash):

- `control_mode = POSITION` via standard CAN cmd 0x00B
- `input_mode = PASSTHROUGH` (same cmd)
- `wL_FF_enable = True` via SDO to id 305

Plus seeds the feeder's per-leg pos targets from the current encoder
readings, so the first commanded position doesn't slam the leg
toward a stale target.

The runtime self-heal is the safety net. Even if the WebGUI or a
recalibration reverts the flash defaults, the level loop self-corrects
on every Level ON.

---

## Bag recording + diagnostics

### Per-tick: `/level_diag` LevelDiag

Published at the loop rate (200 Hz). Carries:

- IMU `roll`, `pitch`, `yaw`
- `target_roll/pitch` (the commanded reference)
- Raw and filtered errors
- Integrator state, PI output, post-clip correction
- Clip flags bitfield (which clamps engaged this tick)
- Per-leg `motor_targets`, `leg_enc`, `leg_vel`, `leg_iq_sp`, `leg_iq`
- Per-leg `axis_state` (heartbeat-derived)
- Per-leg `active_errors` (decoded bits in the analyzer)
- Per-leg `feeder_mode` (idle/pos/vel)
- `dt_actual`, `target_xyzrpy`

Recorded into rosbags via `_lr_spawn_bag` (single bag) or per-Z child
bags (auto-sweep, system-id).

### At bag start: sidecar `<bag>_notes.json`

Captured once before bag spawn. Includes:

- Outer-loop gains in effect
- `git_sha` of the running code
- `level_loop_hz` and `ctrl_period_s`
- **`inner_loop_config`**: per-drive SDO snapshot of every parameter
  in `LR_SNAPSHOT_ENDPOINTS` (current_soft_max, wL_FF_enable, gains,
  watchdog, etc.). Surfaces the asymmetry-across-drives flag in the
  digest.
- Free-text notes from the GUI
- For sysid runs: rest_rpy, rest_encoders, delta list, Z

### Offline analysis: `analyze_level_bag.py`

Per-bag `<name>_summary.json`:

- `health` block: per-leg state %, active_errors decoded, feeder_mode
  uniformity, yaw drift
- `inner_loop_config` block: per-drive snapshot + asymmetry flag
- `baseline` block: RMS, p2p, FFT peak, saturation %
- **`settling` block**: settling time, time-in-band 005/010/020
  (combined both axes), ss_mean/std/p2p over last 5 s, integrator
  drift p2p, dominant oscillation frequency

Per-bag `<name>_plots.png`:

- 6-panel diagnostic: error vs target band, corr vs saturation,
  integrator state (the wind-up smoking gun), per-leg motor target
  vs encoder, error FFT, error magnitude histogram

Sweep-level `<sweep>_overview.png`: RMS / time-in-band / settling /
saturation vs Z.

### Cross-bag comparison: `compare_bags.py`

Sectioned table comparing N bags side-by-side: gains, inner-loop
config, results. Asymmetry warnings. Markdown export. Overlay PNG of
err traces with target bands shaded.

---

## System-ID runner

`stewart_control_node._sysid_run_inner` does the empirical Jacobian
data collection:

1. Calls `_prepare_for_level` (so the same inner-loop config that
   the level loop uses is active during measurement)
2. For each commanded Z in `z_mm_list`:
    - Move platform to `(0, 0, Z, 0, 0, 0)` via geometric IK
    - Wait for settle (`_lr_wait_z_reached`)
    - Snapshot rest IMU + encoders → linearization point
    - Spawn per-Z bag for raw data
    - For each leg × δ × rep: +δ → settle → sample IMU → return →
      −δ → settle → sample → return
    - Average rpy_pos and rpy_neg across reps
3. Compute empirical + theoretical Jacobians per (Z, δ)
4. Dump to `<sysid_dir>/jacobian.json` and mirror to
   `tuning_data/system_id_<UTC>/jacobian.json`

The level loop loads the most recent `jacobian.json` automatically.
`reload_empirical_ik` cmd refreshes without restarting the node.

---

## File structure

```
stewart_bringup/
├── stewart_bringup/
│   └── stewart_control_node.py        # everything above lives here
├── scripts/
│   ├── set_odrive_feedforward_via_can.py   # configurator (persistent flash)
│   ├── dump_odrive_endpoints.py            # one-time endpoint extraction
│   ├── analyze_level_bag.py                # digest writer
│   ├── compare_bags.py                     # side-by-side comparison
│   └── ...
├── config/
│   ├── level_gains.yaml                # outer-loop gains (GUI-editable)
│   ├── leg_limits.yaml                 # per-leg soft limits + rest pos
│   └── global_limits.yaml              # bus-wide caps
├── data/
│   ├── odrive_endpoints.json           # endpoint id → name (configurator's table)
│   ├── odrive_endpoints_reference.md   # human-readable controls reference
│   └── odrive_flat_endpoints_0.6.11-1.json   # full official table for cross-check
├── docs/
│   ├── level_loop_architecture.md      # this file
│   └── level_loop_lessons_learned.md   # the journey
└── web/
    └── index.html                      # GUI
tuning_data/
├── <bag_dirs>/                         # per-bag digests + plots
├── sweep_<UTC>_sweep/                  # per-sweep dirs with per-Z child bags
└── system_id_<UTC>/                    # per-sysid runs with jacobian.json
```

---

## Performance achieved (commit c21f7af)

| metric | value |
|---|---|
| RMS roll | 0.110° |
| RMS pitch | 0.107° |
| ss_std roll | 0.0155° |
| ss_std pitch | 0.0424° |
| settling time roll | 5.18 s |
| settling time pitch | 2.42 s |
| time in ±0.10° (both axes) | 46.1% |
| time in ±0.20° (both axes) | 94.1% |

Comparison to where we started (commit a5b717d, KP=0.7 KI=0.2):

- 27× tighter steady-state std on roll
- 16× tighter on pitch
- Dominant oscillation amplitude 25× smaller
- Integrator drift 19× less

Headroom for further improvement (smaller deadband, narrower decay
zone) exists but is into diminishing returns relative to the IMU
noise floor (~0.05°). Current performance is more than sufficient
for the vision work this stack supports.
