# Level-loop tuning saga — lessons learned

Compiled at end of the 2026-04-27 → 2026-04-29 controls journey, from
the iter-1 baseline (a5b717d, RMS 0.12°) through the wL_FF=False
regression that took us 30× backward, the firmware harmonization,
the empirical-Jacobian system-ID, to the final sub-0.1° steady-state
hold.

The lessons aren't ordered by importance — they're ordered by the
pattern of mistakes that each lesson prevents future-you from making.

---

## 1. Distinguish "runtime" from "persistent" state on every CAN write

**The single recurring bug all month.** ODrive's CAN protocol has two
paths for changing parameters:

| mechanism | scope |
|---|---|
| Standard CAN cmd 0x00B (`Set_Controller_Mode`), 0x00F (`Set_Limits`) | runtime only — reverts on disarm/reboot |
| SDO write to a config endpoint (e.g. id=315 `current_soft_max`) | runtime only too |
| `save_configuration()` (id=710) | persists current runtime to flash |
| Per-drive default values from cal / DFU | overwrite persistent state |

We hit the same class of bug at least four times:

- The GUI's "set leg current" slider sent `Set_Limits` (runtime), so
  any disarm or reboot reverted to the 6 A flash default. Fixed in
  e1c1c8d by also syncing `self.leg_current_a` from flash on bus open.
- `wL_FF_enable` got reset to False by the configurator, persisted to
  flash, and the next 5 sweeps regressed by 30×. Fixed by rolling
  back the default and adding a runtime self-heal in
  `_prepare_for_level`.
- `control_mode` got reset to VELOCITY (=2) by WebGUI's "Run
  Configuration Script" multiple times. Same pattern: WebGUI persists
  factory defaults, our runtime tries to use POSITION, watchdog
  disarms.
- `current_soft_max` reverted to 6 A whenever recalibration ran. Same
  pattern.

**Rule going forward:** every parameter the level loop depends on
must be (a) written persistently to flash via the configurator AND
(b) re-asserted at runtime via `_prepare_for_level` on every level-on.
Neither alone is enough. The runtime write is the safety net; the
flash write is the fast-boot path.

---

## 2. The IK was wrong — by 100-450%

This was the load-bearing discovery. The geometric IK in
`_compute_motor_targets` produces leg-length deltas assuming a
perfectly-built platform with idealized geometry. Reality has:

- Per-leg sign convention errors
- Asymmetric mounting angles
- Per-leg gain mismatch from belt tension / pulley alignment
- Z-dependent nonlinearity from leg-extension geometry

The empirical Jacobian measured on 2026-04-29 (system-ID run at 5 Z
heights × 4 δ values × 6 legs × 3 reps) showed the geometric IK had
**wrong signs on multiple legs** for the roll axis. Theoretical said
"leg 1 increases roll +7.2 °/turn," reality said "leg 1 barely affects
roll, −0.3 °/turn." Mean magnitude error was 130% on roll, 450% on
pitch.

**Bag-level effect**: with the geometric IK the level loop reached
RMS 0.218° at best. With the empirical Jacobian (commit 275ea77),
same gains, **RMS dropped to 0.110° on both axes** — and the
integrator drift went from 1° peak-to-peak (chasing the IK
mismatch) to 0.05° (loop sitting steady). 27× tighter steady-state
deviation, 19× less integrator wind-up.

**Rule going forward:** any time the loop fights a "systematic
offset" the integrator can't quite close, suspect the IK before
suspecting gains. Tuning gains on top of a broken model is wasted
effort.

---

## 3. "Tune the gains" is the WRONG default debugging instinct

When tracking degraded, the natural reflex was "lower KP" or "add
deadband" or "tune integrator decay." Sometimes those helped. More
often they masked a different root cause:

- Sweep on 2026-04-28 22:22 (post-wL_FF-regression-recovery): tuning
  felt limited by something. Was actually the inner-loop feedforward
  being off. No outer-gain tweak fixed it; only the wL_FF restore did.
- 2026-04-29 03:02 sticky-then-release: looked like classic stick-slip
  needing higher current. Was actually the IK gain mismatch — the
  loop was over-correcting because corr→tilt response wasn't 1:1.
- 2026-04-29 05:18 to 05:26 jacobian-vs-geometric A/B: same gains,
  swap IK only, **2× tighter RMS**. Tuning never could have produced
  this.

**Rule going forward:** before tuning gains, verify (1) inner-loop
state via `inner_loop_config` snapshot in the bag, (2) IK fidelity
via empirical Jacobian. If those are wrong, tune nothing — fix them.

---

## 4. Diagnostic infrastructure pays for itself within 2 bags

We added diagnostic features in this order, each prompted by a
specific frustration:

1. **`/level_diag` topic** with per-tick state, errors, integrator,
   correction. Made the loop's behavior visible at 50/200 Hz.
2. **`per_leg_state` + `active_errors` + `feeder_mode` + `yaw`** in
   LevelDiag. Caught the 1° rms regression as "platform actually
   running but at a 12° tilt the loop can't correct" instead of "
   loop is broken."
3. **`inner_loop_config` snapshot** at bag start, per Z. Caught the
   wL_FF=False regression in 5 minutes by diff'ing snapshots, when it
   would otherwise have taken hours of bisecting commits.
4. **Diagnostic plot grid** (6 panels per bag) and overview plot per
   sweep. Visual, not numerical, often surfaces the smoking gun
   faster — e.g., the integrator-drift panel showed obvious wind-up
   patterns that the JSON stats hid.
5. **`compare_bags.py`** for side-by-side digest tables across runs.
   The Jacobian-vs-geometric comparison was a 2-line invocation.
6. **System-ID** for the empirical Jacobian itself.

Every one of these was high-leverage AFTER it was built. Time spent
building diagnostics paid back many times over in time saved
debugging.

**Rule going forward:** when iterating on a control system, build
the next diagnostic the moment "I wish I could see X" comes up. Don't
delay-and-debug; build-first-and-debug.

---

## 5. Sticky-flash, runtime self-heal, and bag-start snapshots are the right pattern

The combination ended up being:

- **`set_odrive_feedforward_via_can.py --apply`** with the right flags
  → sets persistent flash values via SDO + `save_configuration()`
- **`_prepare_for_level()`** at every level-on → runtime SDO writes
  for `control_mode`, `input_mode`, `wL_FF_enable`, plus seeds feeder
  pos targets from current encoders
- **`_lr_capture_inner_loop_config()`** at bag-start → SDO snapshot
  of every relevant runtime config field, dumped to the sidecar
- **Analyzer's `inner_loop_config` block in summary.json** with
  `any_differ_across_drives` flag → asymmetry between drives jumps
  out instantly

This 4-layer pattern survives any one layer's failure. Future
parameters added to the level loop should go through all 4: persist
via configurator, self-heal in `_prepare_for_level`, snapshot at
bag-start, surface in the analyzer.

---

## 6. Run-once empirical models beat per-tick computation

Replacing `_compute_motor_targets`'s rotational IK with the empirical
Jacobian's pseudo-inverse:

- Adds ~5 ms of compute at node startup (load JSON, pinv 5 matrices)
- Removes per-tick trig from the geometric IK, replaces with one
  6×3 × 3×1 matrix-vector multiply (~50 ops)
- **Net: cheaper per tick than the original geometric IK**

The pattern generalizes: any time a per-tick computation depends on
quantities that don't change with state (geometry, calibration), it
should be precomputed at startup and looked up at runtime.

---

## 7. Don't recompute the bag-comparison spreadsheet by hand again

Manual comparison was ~80% of the friction. `compare_bags.py` does
in 5 seconds what was taking 20 minutes of cross-referencing JSON
files. Build it once, use it dozens of times.

The JSON output of every bag is structured for diff'ing on purpose:
nested `gains` block, nested `inner_loop_config`, nested `health`,
nested `settling`. Future analysis tools should rely on this
structure rather than custom regex'ing CSV columns.

---

## 8. The platform's Jacobian is a property of the robot, not a
control gain

This was the surprising-in-retrospect insight. Once we measured the
empirical Jacobian, it became clear: this isn't a knob we tune. It's
a measured property of the build. Run the system-ID once, save the
JSON, the IK is correct forever (until you change the mechanism).

That moves the level-loop tuning from "fight a moving target" to
"hold a known steady model" — which is why the same gains that gave
RMS 0.4° with geometric IK gave RMS 0.11° with empirical IK.

---

## 9. Specific bugs to never reintroduce

- **Don't use `current=6.0` as the arm-flow default.** Always use
  `self.leg_current_a` (the slider value) which is in turn synced
  from flash on bus open.
- **Don't write CMD_SET_LIMITS without also surfacing what was
  written in the GUI.** Runtime writes that aren't visible cause
  "phantom" parameter values.
- **Don't run a sweep with step_amp = 1° on a platform with 6 A
  current limit.** It can stall a leg against an endstop.
- **Don't trust the WebGUI's "Run Configuration Script" to leave
  `control_mode` and `input_mode` alone.** It sets them to factory
  defaults and persists. Always re-run the configurator after any
  WebGUI session.
- **Don't analyze a bag without checking the `inner_loop_config`
  asymmetry warning.** It's the smoking gun for any per-drive
  miscalibration.

---

## 10. Net score

Started: RMS ~5.4° / RMS ~12.1° (the "fucked error" run, post-
regression baseline before recovery).

Ended: RMS 0.110° on both axes, ss_std 0.015°/0.042°, **94% of the
bag inside ±0.20°, 46% inside ±0.10°.** Settles in 2-5 seconds.

100× tighter than where we started, fully repeatable, and the IK
is now a measured property of the robot rather than a guess.
