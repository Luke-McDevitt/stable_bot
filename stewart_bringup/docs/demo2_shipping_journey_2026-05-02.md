# Stable-Bot Demo 2 — shipping journey, 2026-05-02

A chronological + topical record of the day Demo 2 (click-to-goto
ball control) went from "every click degenerates into an orbital
limit cycle" to a settled trajectory with **median tracking error
of 0.5 mm**. Continues from the lessons in `controls_journey.md`
(2026-04-30) and `step_id_tuning_lessons.md` (2026-05-01); read
those first if you want the prior context on how the architecture
got to where it was at the start of this session.

---

## TL;DR — top eight lessons

1. **BALL_TRACK and the level loop must use the same IK path.** The
   level loop already used the empirical Jacobian (`use_empirical_ik`
   flag). `_do_set_pose` did not. After the cascade-bypass commit
   (`b9dee51`, 2026-05-01), BALL_TRACK called `_do_set_pose` directly
   and silently lost the empirical correction. The empirical Jacobian
   showed columns rotated **45-62° from the theoretical Jacobian** at
   every Z height — i.e. real platform-to-IMU frame mismatch that the
   geometric IK didn't model. With BALL_TRACK on geometric IK, an rpy
   command in IMU frame produced a physical tilt rotated ~55° from
   intended, and the ball moved at an angle to every click target.
   **Single biggest unlock of the day.**

2. **PID Kd × v_noise dominates direction when Kd is non-trivial.**
   With raw KF velocity reading 1500-2300 mm/s spikes on a foam ball
   that physically tops out around 800, even a modest `kd=0.010`
   rails the tilt magnitude in random directions. The "platform tilts
   away from the target" symptom that operator reported was real and
   reproducible — math: `Kd dominates Kp when |v| > (Kp/Kd) · |err|`,
   which for sane gains hits during normal motion.

3. **Bang-bang has no tangential damping. It cannot break orbits.**
   FSM phase logic is keyed on `v_toward` (radial component of
   velocity along the error vector). For a ball orbiting around the
   target, `v_toward ≈ 0` (motion is purely tangential) → FSM picks
   ACCELERATE → tilts toward target → adds inward force only →
   tangential velocity is conserved. Only PID's `Kd × v` term opposes
   *all* directions of motion, including tangential. This was a
   correct-by-construction failure mode that took several hours to
   diagnose because the digest's saturation/jitter metrics looked
   normal.

4. **Velocity is the derivative of position; it amplifies noise.**
   Position noise of 1-2 mm becomes velocity noise of 30-60 mm/s when
   you differentiate. Any "ball is moving" check based on velocity
   thresholding has the same noise floor problem. **Position-delta
   from a snapshot is far cleaner**: a stationary ball jitters by
   1-2 mm, a real movement displaces 10s of mm, so a 5-10 mm
   threshold cleanly separates them.

5. **`ramp_rate × timeout` must exceed `ramp_max − ramp_start`.**
   Otherwise the stiction ramp times out before reaching its
   commanded ceiling. Operator had `ramp_rate=0.35, timeout=8s,
   ramp_max=5°, ramp_start=0` — the ramp peaked at `0.35 × 8 = 2.8°`
   and reset, never reaching the 5° needed to break stiction. Hours
   of tuning vanished into this single equation.

6. **Stiction ramp parameters can legitimately exceed max_tilt.**
   `max_tilt` is the in-flight damping cap; the ramp may need higher
   tilt to reliably break stuck stiction (especially with friction
   variations across the deck). Decoupled the two: ramp uses
   `stiction_ramp_max_deg` only; the per-axis saturation uses
   `max(max_tilt, ramp_max)` while STICTION_RAMP is the active phase,
   then drops back to `max_tilt` once the ball is moving.

7. **Live KF + offline smoother is a powerful debugging combo.** The
   live ball_kf_node is causal-only and noise-smoothed only from the
   past. Running a Savitzky-Golay forward+backward smoother offline
   on the same raw vision data gives a much cleaner velocity signal
   that uses past AND future frames. The divergence between live KF
   and offline smoother quantifies how much the live filter is
   wrong, and the smoothed signal serves as ground truth for any
   future learned filter.

8. **Hand-tuning with good comparison tools beats auto-tuning.** The
   2026-04-30 auto-tuner attempt (random-target hill-climb +
   Bayesian) got 27 trials with 0 acceptances because random target
   placement produced fitness noise larger than gain effect.
   Replacing it with a `compare_demo_bags.py` table that diffs gains
   against outcome metrics across all the operator's hand-tuned runs
   immediately surfaced the winning configuration.

---

## Timeline

### Morning: orbital-limit-cycle diagnosis attempts

Operator reported "the ball quickly decays into an orbital motion
around the edge of the platform (since I put a lip to stop the ball
from rolling off)." Initial prescription: `max_tilt_deg` was set to
3°, well below the orbital-braking threshold `θ_orb` for any
meaningful tangential velocity at the rim. Bumped to 9° plus reduced
Kd. **No effect** — orbits persisted in cv2 and v1_yolo backends.

Operator reported "the platform doesn't even tilt in the correct
direction to get the ball from where it is to where the target is."
This was the symptom that finally pointed at the real bug.

### Midday: the IK frame discovery

Traced the call chain: BALL_TRACK loop → `_do_set_pose(...)` →
`_compute_motor_targets((x,y,z), (r,p,yw), self.geom, self.limits)` —
**no `empirical_ik` argument passed**, so geometric IK only. The
level loop, by contrast, had this:

```python
use_emp = bool(g.get('use_empirical_ik', 0)) and \
    self.empirical_ik is not None and \
    self.empirical_ik.is_loaded()
ik = self.empirical_ik if use_emp else None
targets, _ = _compute_motor_targets(
    tuple(xyz), tuple(rpy_cmd), self.geom, self.limits,
    empirical_ik=ik)
```

Two different IK paths in the same node. Loaded the most recent
`jacobian.json` (`system_id_20260429T033751Z`) and compared empirical
vs theoretical column-by-column at every Z height:

| Z = 30 mm, δ = 0.05 | Direction error per leg |
|---|---|
| Leg 0 | 57.85° |
| Leg 1 | 50.17° |
| Leg 2 | 56.14° |
| Leg 3 | 45.42° |
| Leg 4 | 59.67° |
| Leg 5 | 54.41° |

**The empirical Jacobian was rotated 45-62° from theoretical at
every leg, at every Z height, at every delta.** Not gain mismatch —
a frame rotation. The level loop had been silently absorbing this
since 2026-04-29. The cascade-bypass commit moved BALL_TRACK off
that path and onto the unrotated geometric IK.

Fix in `_do_set_pose` (commit `938455d`): gate on the same
`use_empirical_ik` flag the level loop reads. After deploy, operator
confirmed: **"pitch sign set to -1 and the roll sign at +1 has it
correctly point towards the right target."** First click that drove
the ball cleanly toward where the operator clicked.

### Afternoon: the long tail of small bugs

After IK was fixed, the ball moved correctly but oscillated on
arrival. The remaining problems and fixes:

**Vision noise into Kd (commit `3b69332`).** Added `kd_v_tau_s`
first-order LPF on the velocity input to PID's Kd. With `tau=0.15-0.3`,
single-frame V0 noise spikes barely move the filtered value;
orbital-rate motion (3 s period = 0.33 Hz) passes through. Reduced
cmd_jitter from 1.4°/tick to 0.26°/tick.

**Long-distance speed buildup (commit `fba9297`).** Added
`tilt_slew_up_deg_per_s` to limit how fast the commanded tilt
magnitude can grow per tick. Default 100 (off); set to 3-5 for soft
startup. Decreases (braking response, return to zero) are
unconstrained — only growth is limited. Combined with the stiction
ramp, this gives the ball a gentle acceleration profile on
long-distance clicks instead of an immediate kick.

**Slam-to-max stiction was kicking too hard (commit `5f87cb5`).**
Replaced the prior "boost magnitude to max_tilt" stiction relief
with a slow ramp that starts at `stiction_ramp_start_deg` (just
above the measured `θ_s` from STEP_ID's stiction phase) and climbs
at `stiction_ramp_rate_deg_per_s`. The ball breaks loose at near-zero
velocity instead of getting kicked into a fast trajectory.

**Stiction ramp can exceed max_tilt (commit `6872d5b`).** Decoupled
the ramp's magnitude cap from `max_tilt`. The ramp uses
`stiction_ramp_max_deg` only, and the per-axis saturation uses
`max(max_tilt, ramp_max)` while STICTION_RAMP is the active phase.

**Stiction ramp re-triggering (commit `fcaba25`).** Added a
`stiction_ramp_timeout_s` for the case where the ball never moves
within the ramp window — resets the dwell timer and ramp restarts
from `ramp_start`. Creates a "pulse → coast → pulse" pattern: each
ramp pulse just barely breaks stiction, ball moves a small distance,
PID damps, ball stops, next pulse fires. Useful for low-friction
hardware where one ramp can't drive the ball all the way to target.

**Vision-noise spikes resetting the dwell (commit `6f8701d`).**
Even with the LPF on Kd input, the *stiction-check gate* was using
raw `vel_mag = math.hypot(vx, vy)`. With the operator's noise floor
producing 60+ mm/s readings on a stationary ball vs threshold of
26 mm/s, the dwell timer reset every time noise crossed threshold.
Added a consecutive-tick counter (`stiction_moving_hyst_ticks`)
requiring N ticks of sustained motion before declaring "moving".

**3-tick hysteresis wasn't enough for sustained noise
(commit `744cf6e`).** Routed the stiction check through the LPF'd
velocity instead of raw, so the check sees smooth velocity that
doesn't cross threshold from noise alone. Combined with the
3-tick consecutive counter, this is robust to both single-frame and
sustained vision noise.

**Position is the cleaner signal (commit `ed47f00`).** Added
`stiction_pos_delta_mm`: when the dwell timer starts, snapshot
the ball's position; when displacement from that snapshot exceeds
threshold, exit the ramp regardless of velocity reading. A
stationary ball jitters by 1-2 mm in position; a real movement
displaces 10s of mm. Threshold of 5-10 mm cleanly separates them.
**This was the change that made the ramp reliably exit at the
right moment.**

### Late afternoon: the ramp-rate-vs-timeout equation

Operator's gains had converged to:
- `stiction_ramp_start_deg: 0.0`
- `stiction_ramp_max_deg: 5.0`
- `stiction_ramp_rate_deg_per_s: 0.35`
- `stiction_ramp_timeout_s: 8.0`

IMU never showed pitch above ~2.8°. Wasn't a code bug — the math:

```
ramp tilt at time t = ramp_start + ramp_rate · t
                    = 0.0 + 0.35 · t
ramp reaches max when (ramp_max - ramp_start) / ramp_rate
                    = (5.0 - 0.0) / 0.35 = 14.3 s
timeout = 8.0 s   < 14.3 s
```

**Ramp peaks at `0.35 × 8 = 2.8°` and resets**, never reaching the 5°
configured ceiling. Operator had been chasing this for hours.

Fix: tune the gains so the ramp reaches `ramp_max` AND has time to
dwell there. With `ramp_start=2.5, ramp_rate=1.5, timeout=4.0`:
climbs 2.5° → 5.5° in 2.0 s, then sits at 5.5° for 2.0 s before
timeout. Total 4 s of progressive stiction-breaking force.

### Evening: settling

After all the above, operator ran ~30 small variations. Built
`compare_demo_bags.py` to extract gains + outcome metrics from every
`digest.summary.json` into a single sortable table. Sort by
`median_tail_err_mm` ascending. The top row was **bag
`20260502T025210Z` with median tail error 0.5 mm and `settled: Y`**.

Winning gains (preserve these):

```yaml
algorithm: pid

# PD core
kp: 0.016
kd: 0.025
ki: 0.0
max_tilt_deg: 1.2
control_latency_s: 0.15

# Velocity LPF for Kd input (kills vision-noise spikes)
kd_v_tau_s: 0.35

# Sign convention
pitch_sign: -1.0
roll_sign: 1.0

# Stiction ramp (the "pulse-coast-pulse" walker)
stiction_ramp_start_deg: 0.0
stiction_ramp_max_deg: 6.0
stiction_ramp_rate_deg_per_s: 0.3
stiction_ramp_timeout_s: 25.0
stiction_v_threshold_mm_s: 61.0
stiction_break_s: 0.3
stiction_pos_delta_mm: 12.0
stiction_moving_hyst_ticks: 3

# Slew-rate limit on tilt magnitude growth (off — not needed here)
tilt_slew_up_deg_per_s: 100.0

err_tol_mm: 15.0
```

A second settled run (`030936Z`) confirmed it wasn't a fluke:
4.2 mm median tail error with similar gains.

---

## Specific bugs found and fixed

A non-exhaustive list of the more painful ones, in roughly
chronological order:

### IK frame mismatch (the day's biggest finding)

`_do_set_pose` always used geometric IK; level loop used empirical
Jacobian. Cascade-bypass moved BALL_TRACK to the wrong path.
Empirical Jacobian's columns are rotated 45-62° from theoretical
at every Z and every leg. **Fix: pass `empirical_ik=self.empirical_ik`
to `_compute_motor_targets` from `_do_set_pose`** when the same
`use_empirical_ik` flag is set that the level loop reads.

### Bang-bang vs PID for orbital recovery

Initial prescription was bang-bang because it sidesteps Kd-noise
issues. Failed because bang-bang's phase logic uses `v_toward` only
and has no tangential damping. **Lesson: bang-bang for
chatter-near-target failures (where Kd amplifies noise on a
stable target); PID for orbital limit cycles (where Kd's
tangential damping is what kills the orbit).**

### `control_latency_s = 0` regressed phase margin

Operator reduced `control_latency_s` from 0.10 to 0.0 during one
tuning pass, breaking the lookahead compensation. The Kp term
started reacting to where the ball was 110 ms ago instead of where
it would be when the tilt arrived. Visible in the bag as a sudden
amplitude jump in the tracking-error oscillation. **Fix: restored
to 0.10 (the actual end-to-end latency from `oak_latency_ms.p50`).
Vetted that PID does use the lookahead** (it does — line 4452-4454,
`ex_lead = ex + edot_x * latency_s`).

### `max_tilt = 1.0` killed PID's ability to maintain motion

Operator dropped `max_tilt` low to prevent in-flight runaway. With
`max_tilt = 1.0`, PID acceleration was 122 mm/s² — well below
friction's 363 mm/s² — so as soon as a stiction ramp pulse handed
off to PID, friction stopped the ball within ~1 mm. Demo became a
sequence of 1 mm hops at ~1.3 s cycle time = 130 s to traverse
100 mm. **Lesson: `max_tilt` must be above `θ_s` so PID can
*maintain* motion between ramp pulses, not just below the runaway
threshold. Sweet spot: 1.2-1.5° on this hardware.**

### Vision noise into Kd → wrong direction

Documented in detail in lesson #2 above. **Fix: LPF on Kd input
(`kd_v_tau_s`) — a first-order filter with τ=0.2-0.35 attenuates
frame-rate noise (>5 Hz) by 20-30 dB while passing orbital
motion (<1 Hz) cleanly.**

### Stiction-check gate using raw velocity

Vision noise sustained above threshold for many consecutive ticks.
Hysteresis (consecutive-tick counter) wasn't enough. **Fix: route
the stiction check through the same LPF'd velocity that Kd reads.
Then the consecutive-tick counter and the LPF reinforce each
other.**

### Position-delta exit is the cleanest stiction signal

After all the velocity-based filtering, position-delta is *still*
the most reliable "did the ball actually move" check. **Lesson:
when you can use position instead of velocity for a binary
detection, do — it has 30-60× less noise.**

---

## Architectural changes shipped

In rough order of impact:

### Empirical IK in `_do_set_pose` (commit `938455d`)

Single biggest fix. BALL_TRACK now uses the same empirical Jacobian
the level loop has been using since 2026-04-29.

### Velocity LPF for Kd (commit `3b69332`)

`kd_v_tau_s` in the gains panel. Default 0 (off, prior behavior).
Set to 0.2-0.35 to attenuate vision-noise spikes that Kd otherwise
amplifies into the tilt command.

### Stiction RAMP replaces slam-to-max (commit `5f87cb5`)

Three new gains: `stiction_ramp_start_deg`, `stiction_ramp_max_deg`,
`stiction_ramp_rate_deg_per_s`. Ball breaks loose at near-zero
velocity instead of getting kicked into a fast trajectory.

### Stiction ramp can exceed max_tilt (commit `6872d5b`)

Decouples in-flight damping cap (`max_tilt`) from stiction-relief
authority (`stiction_ramp_max_deg`).

### Stiction ramp timeout (commit `fcaba25`)

`stiction_ramp_timeout_s` — safety reset if the ball never moves.
Creates the "pulse-coast-pulse" walker pattern that worked.

### Hysteresis on "ball is moving" (commits `6f8701d`, `744cf6e`)

`stiction_moving_hyst_ticks`: require N consecutive ticks of
sustained motion before declaring the ball moving. Then routed
through the LPF'd velocity for double-filtering.

### Position-delta exit (commit `ed47f00`)

`stiction_pos_delta_mm`: snapshot position at dwell start, exit
ramp when displacement exceeds threshold. The check that finally
made the ramp reliably exit at the right moment.

### Tilt slew-rate limit (commit `fba9297`)

`tilt_slew_up_deg_per_s`: caps how fast Kp·err can grow the
commanded magnitude. Default 100 (off). Useful as backup for
long-distance soft-start; not needed for the winning configuration.

### Always-editable gains panel (commit `5f87cb5`)

Removed the opacity-dimming on the bang-bang threshold block
when PID is selected. All inputs editable regardless of algorithm
selection.

### Default vision backend = cv2 (commit `fba9297`)

`stable_bot.service` boot env changed from `OAK_V0_BACKEND=v1_yolo`
to `cv2`. Empirically more reliable for the demo motion profile
on this hardware/lighting combination.

---

## Tooling shipped

These are the scripts that turned hand-tuning from "guess and check"
into "diff and analyze."

### `compare_demo_bags.py` (commit `ecc01ba`)

Walks every `digest.summary.json` in a glob, pulls gains snapshot +
outcome metrics, prints a sortable table. `--diff` mode hides
columns that don't vary across the set, collapsing 24 columns down
to ~10 for a typical sweep. `--csv` for spreadsheet workflows.
`--chronological` for "what did I try and in what order".

This script is what surfaced the winning configuration. Without it,
finding `025210Z` in the noise of 30+ runs would've taken another
hour of digest-by-digest reading.

### `smooth_demo_bag.py` (commit `629e601`)

Offline non-causal Savitzky-Golay smoother. Outputs:
- `smoothed.json`: full time-series with raw, smoothed, and
  live-KF position+velocity. Suitable as ML training labels.
- `smoothed.summary.json`: KF-vs-smoother divergence per axis,
  plus a heuristic Q/R re-tune hint.
- `smoothed.png`: 4-panel comparison plot.

Three downstream uses: (1) cleaner digest analysis, (2) live KF Q/R
re-tuning baseline, (3) training labels for a future learned filter.

### `repo_loc.py` (existed earlier — minor docs improvement)

Lines-of-code counter that walks `git ls-files`. Used to gauge how
much of the new control logic was added today.

---

## Lessons learned (the durable ones)

1. **Find code-path inconsistencies before tuning gains.** Hours
   were spent tuning gains while a frame-rotation IK bug was
   silently corrupting every tilt command. The fix was 1 line of
   code; finding it took half a day. **Always grep for "X is used
   in N places — does it use the same auxiliary state in each?"
   before assuming the algorithm is correct.**

2. **Velocity is noisier than position by a factor of dt.** For
   any binary detection ("is the ball moving"), use position
   delta, not velocity threshold. For continuous control, use
   filtered velocity. For lookahead, use either.

3. **The stiction ramp's parameters are coupled.** `ramp_rate ×
   timeout` must exceed `ramp_max - ramp_start` or the ramp never
   reaches its ceiling. Document this in the gain comments and
   surface as a digest warning.

4. **A hand-tuner with good comparison tools beats an auto-tuner
   without.** The auto-tuner (Bayesian/hill-climb) failed because
   the fitness signal was buried under random-target noise. The
   operator hand-tuning + `compare_demo_bags.py` succeeded because
   the operator has a coherent mental model of which knob does
   what, and the tool surfaces gain-vs-outcome correlations
   immediately.

5. **Friction varies. Empirical θ_s is a starting point, not a
   guarantee.** STEP_ID's stiction phase measured θ_s = 2.14° at
   one position. Real friction varied across the deck enough that
   2.5° wasn't always sufficient. Set `stiction_ramp_start_deg`
   above θ_s and let the ramp climb from there.

6. **Live KF tuning is a downstream optimization.** No amount of
   live-KF Q/R tweaking helps if the underlying frame is wrong
   (lesson 1) or if the controller is using stale data (lookahead
   bug). Fix the control path first; then improve the estimator.

7. **Slam-to-max stiction relief was wrong from the start.** It
   gave the ball more energy than it could damp — every time it
   broke loose, it broke loose at high velocity, which fed
   orbital limit cycles. A slow ramp that stops the moment the
   ball *starts* moving is the correct shape for low-friction
   hardware. The original slam-to-max made sense for high-
   friction hardware where the ball *needs* a kick to overcome
   stiction — but on foam-on-vinyl with μ_s = 0.037, the kick is
   the problem.

8. **The "ball is moving" check needs both LPF and hysteresis.**
   Single-tick noise spikes are filtered by hysteresis; sustained
   noise events are filtered by LPF. Each on its own is
   insufficient on this hardware. Together they're robust.

9. **`max_tilt` is the in-flight cap, not the maximum
   controller-can-ever-output cap.** Stiction breakaway needs
   higher authority than in-flight damping. Decoupling them via
   `stiction_ramp_max_deg` is correct hardware abstraction.

10. **Bang-bang and PID solve different control problems on this
    hardware.** PID with `Kd` damps tangential motion (kills
    orbits). Bang-bang doesn't (FSM logic only sees radial
    velocity). For a Demo 2 click-to-goto trajectory that has both
    transient motion and orbital recovery, PID is the right tool.

---

## Where the system stands as of 2026-05-02 evening

- ✓ **Demo 2 settled at 0.5 mm tracking error** (bag
  `20260502T025210Z`). Reproducible (bag `030936Z` settled at
  4.2 mm with similar gains).
- ✓ Empirical IK is loaded and active in BALL_TRACK as well as the
  level loop.
- ✓ Velocity LPF (`kd_v_tau_s`) keeps Kd-on-noise out of the tilt
  command.
- ✓ Stiction ramp is the soft-start path; position-delta is the
  exit signal; timeout is the safety net.
- ✓ Comparison tooling (`compare_demo_bags.py`) makes
  gain-vs-outcome diffs instant.
- ✓ Offline smoother (`smooth_demo_bag.py`) provides ground-truth
  velocity for analysis and future learned filters.
- ⊘ Demo 3 (path drawing) — `BALL_TRACK_PATH` is still stubbed,
  returns the first waypoint only. Open work.
- ⊘ Live KF Q/R re-tuning using offline-smoother truth — the data
  is there; the optimizer isn't yet built.
- ⊘ Learned ball-state filter (LSTM trained on raw vision +
  smoothed labels) — research direction, ~1-3 days of work.
- ⊘ Vision-noise mitigation at the source (better lighting,
  shorter exposure, different ball surface) — physical
  improvements that would benefit every algorithm without code.

---

## The shipping commits

In order:

| commit  | summary |
|---------|---------|
| `acb9b9a` | fix: align err_t_s/err_xy with err_mag in digest |
| `938455d` | **fix: BALL_TRACK uses empirical IK like level loop** |
| `3b69332` | add: LPF on velocity into PID Kd term (kd_v_tau_s) |
| `786035d` | gui: expose kd_v_tau_s in gains panel |
| `5f87cb5` | PID stiction RAMP + always-editable gains panel |
| `fba9297` | default cv2 backend + tilt magnitude slew-rate limit |
| `6872d5b` | PID stiction ramp can exceed max_tilt cap |
| `fcaba25` | add stiction_ramp_timeout_s for pulse-coast-pulse tuning |
| `6f8701d` | hysteresis on "ball moving" detection |
| `744cf6e` | stiction hysteresis uses LPF'd velocity, not raw |
| `ed47f00` | add stiction_pos_delta_mm — position-delta exit |
| `ecc01ba` | add compare_demo_bags.py |
| `629e601` | add smooth_demo_bag.py |

About a day's worth of code changes plus a comparable amount of
diagnostic work. The `938455d` IK fix is the highest-leverage
single commit; the rest are the long tail of "now that the math is
right, make the loop robust to the hardware's actual physics."

---

## Onward

Demo 2 is shipped. Demo 3 (path drawing) is the next milestone, and
the controls foundation built today (empirical IK in BALL_TRACK,
soft-start stiction handling, noise-robust state detection) is the
right starting point for it. The same `compare_demo_bags.py` workflow
will scale to Demo 3 tuning. The smoother script's outputs become
training labels for a learned ball-state filter when there's time
for it.

Sleep well.
