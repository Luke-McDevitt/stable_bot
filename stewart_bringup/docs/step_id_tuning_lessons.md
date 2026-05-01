# STEP_ID + Demo 2 controller tuning — lessons learned

A chronological + topical record of the analytic-gains tuning work
done 2026-05-01 to get Demo 2 (click-to-goto closed-loop ball
control) into demo-shippable shape. Continues from the lessons in
`controls_journey.md` (2026-04-30) and `vision_control_tuning_lessons.md`
(2026-04-30 architectural changes). Read end-to-end if you're going
to touch the BALL_TRACK PID, the open-loop plant-ID code, or
anything that subscribes to `/ball_state` for control.

---

## TL;DR — top six lessons

1. **There are two physically-distinct tilt thresholds, and the
   controller's `max_tilt_deg` must satisfy BOTH.**
   - **Stiction breakaway** `θ_s` (≈ 4° on this hardware): the
     minimum tilt to overcome static friction on a stationary ball.
     `tan(θ_s) = μ_s` — pure hardware property.
   - **Orbital braking** `θ_orb`: the minimum tilt to brake a ball
     in a circular orbit at radius `r` and tangential velocity `v`.
     `sin(θ_orb) = v² / (r · g)` — depends on instantaneous ball
     state, NOT friction.
   - At rim with `v=500 mm/s`, `θ_orb ≈ 7.3°`. `max_tilt < 7.3°`
     means the controller cannot recover from that orbit.
   - Recommended floor: `max_tilt_deg = max(θ_s + 2.5, 8.0)`.

2. **Vision noise dominates everything when Kd is non-trivial.**
   Single-frame V0 false-positives can spike `vx, vy` to 4000+
   mm/s on a foam ball that physically tops out around 800 mm/s.
   `Kd × v_spike` saturates `max_tilt` in random directions — this
   is energy injection, not damping. Symptoms: orbital limit
   cycle around the platform rim, peak speeds in the digest CSV
   far above physical limits.
   - Mitigation in PID: hard cap `|edot|` at 800 mm/s before
     applying Kd (`stewart_control_node._ball_track_run`,
     `BT_MAX_BALL_VEL_MM_S`).
   - Mitigation in plant-ID fits: median-filter ball position
     before computing velocity (kernel=5); use 95th-percentile
     instead of `np.max` for peak velocity; linear regression
     of `v(t)` vs `∫θ(τ)dτ` instead of `peak_v / integrated_tilt`.

3. **YOLO is not always better than cv2 HSV — depends on the
   training set.** On this hardware (foam ball, ~600 hand-labelled
   frames, YOLOv6n vs YOLOv8n on Myriad X) the `~20%` benchmark
   accuracy improvement of v8 over v6 *did not* translate to
   reliable detection during fast motion — V1 YOLO completely lost
   the ball "almost right away" during an actual demo. cv2 HSV with
   the platform-mask crop was empirically more robust *for this
   use case* despite being more brittle to lighting in benchmarks.
   - Lesson: don't trust offline accuracy benchmarks for closed-
     loop control quality. Always A/B with the runtime backend
     selector in the GUI before locking in `OAK_V0_BACKEND`.

4. **Single-trial plant ID is not reliable enough on this hardware.**
   Six consecutive STEP_ID sessions on the same hardware in one
   evening produced G_eff = 35, 224, 92, 64, 148±58 (n=3),
   564 (n=1, suspect). 5–10× spread on an in-principle-deterministic
   measurement.
   - Causes (in order of impact): vision dropouts mid-trial,
     stiction breakaway timing varying, fit-window degeneracy when
     motion onset is late, safety cutoff firing inconsistently
     due to vision noise (now fixed via multi-frame consensus).
   - Mitigation: replicated trials with a coefficient-of-variation
     (CV = std/mean) gate. Refuse to recommend gains when CV >
     0.30 across n≥2 replicates. Single-trial sessions surface
     a "single-trial, do not trust" warning instead of silently
     passing.

5. **The cascade architecture caps outer-loop bandwidth at the
   inner loop's bandwidth.** Pre-2026-05-01 design: BALL_TRACK
   commanded `current_rpy`, level-PI tracked it via IMU feedback.
   On this hardware level-PI takes ~500 ms to track a 3° step;
   that fundamentally caps achievable BALL_TRACK `ωn` at < 1 rad/s.
   The analytic recommendation engine's preferred `ωn = 4` rad/s
   was incompatible with the cascaded plant.
   - Fix (2026-05-01): BALL_TRACK now disables level-PI on entry
     and commands tilts directly via `_do_set_pose`. Plant biases
     that level-PI used to absorb (asymmetric leg backlash, IK
     approximations, gravity sag) are now compensated by the
     outer-loop integrator instead. Slower bias rejection but
     faster transient response — net win for ball control.
   - Restore behaviour: `_start_ball_track_loop` saves the level
     state and re-enables it on exit IF it was on before
     BALL_TRACK started. Operator's "level off" choice survives.

6. **`STEP_ID` is a useful methodology even when it can't produce
   a clean recommendation.** Even when CV gates fire and the
   analytic recommendation is refused, the diagnostic outputs
   (`ball_paths_combined.png`, `phase_plane.png`,
   `tilt_timeseries.png`, `plant_gain_fit.png`,
   `step_id_summary.json`) are the right view of the system. The
   per-phase Run buttons (stiction / open-loop / verification)
   let the operator iterate on a single piece without paying for
   the full session. Stiction characterisation alone gives
   `θ_s + μ_s` in 10–25 s — useful even when the operator skips
   plant ID entirely.

---

## The two-tilt-thresholds insight (read this first)

This was the single biggest conceptual confusion during today's
debugging. Several hours wasted before the distinction was clear.

### Stiction breakaway (`θ_s`)

A ball at rest on a tilted plane is held by static friction until
the gravity component along the surface exceeds the maximum
static friction force:

```
m·g·sin(θ_s) > μ_s · m·g·cos(θ_s)
→ tan(θ_s) > μ_s
```

For foam-on-vinyl on this platform, `μ_s ≈ 0.07`, so `θ_s ≈ 4°`.
Below 4° tilt, the ball will not move regardless of how long the
tilt is applied.

**Diagnostic**: STEP_ID's "Stiction characterisation" phase
(2026-05-01, commit `1d49f1d`) measures this directly. Ramp tilt
from 0° at 0.5°/s, watch for first ball motion, record IMU-actual
tilt at motion onset.

**Where it matters**:
- The PID's `commanded tilt = Kp · err`. If `Kp · err < θ_s`, ball
  doesn't move. Either the integrator slowly winds up to break
  through (overshoot, limit cycle), or the stiction-relief PID
  branch (commit `2eb6235`) boosts to `max_tilt` after
  `stiction_break_s` of stuck-ball detection.
- Open-loop plant-ID tilt magnitude must be > `θ_s + 1°` or the
  ball won't move during the trial → fit produces near-zero G_eff.
- Closed-loop `max_tilt_deg` floor must be `θ_s + 2.5°` minimum,
  otherwise the recommended Kp at small errors produces tilts
  below stiction breakthrough → stuck ball.

### Orbital braking (`θ_orb`)

A ball moving in a circular orbit at tangential velocity `v` and
radius `r` experiences centripetal acceleration `v²/r`. To brake
it via gravity-along-surface:

```
g·sin(θ_orb) > v² / r
→ sin(θ_orb) > v² / (r · g)
```

For a ball at rim (`r = 200 mm`) at `v = 500 mm/s`:
`sin(θ_orb) > 250000 / (200 · 9810) = 0.127` → `θ_orb > 7.3°`.

This depends on the ball's *instantaneous* state. A faster ball
needs more tilt; a ball not in orbit needs none.

**Where it matters**:
- If `max_tilt_deg < θ_orb` for the orbit the controller has
  driven the ball into, you are physically incapable of braking
  it. Symptom: orbital limit cycle that persists indefinitely
  regardless of gains.
- Recommended floor: `max_tilt_deg ≥ 8°` to handle up to
  `v ≈ 530 mm/s` orbital velocity. Combined with stiction floor:
  `max_tilt_deg = max(θ_s + 2.5, 8.0)`.

### These two are NOT the same number

Stiction characterisation gives `θ_s` only. It tells you nothing
about `θ_orb`. Don't conflate them. If your stiction phase says
`θ_s = 3°` and you set `max_tilt = 5.5°`, the ball can break
stiction fine but cannot recover from a moderate orbit.

---

## Plant ID methodology — what we tried, what stuck

We went through three generations of G_eff fitting in one evening.

### Gen 1: parabolic position fit on a fixed window (initial design)

Fit `x(t) = c0 + c1·t + ½·a·t²` over `[ramp_end, end_of_tilt]`.
G_eff = a / θ_commanded.

**Failure mode**: when stiction holds the ball through most of
the tilt window, the fit captures a near-flat trajectory and
returns `G_eff ≈ 0`. The ball moves *after* the fit window — too
late.

### Gen 2: parabolic + peak-velocity fallback

Added `_fit_velocity_based_g_eff` = `peak_v / ∫|θ_imu(τ)| dτ` over
`[0, peak_v_t]`. Use this when parabolic returns implausibly low.

**Failure modes**:
- `np.max(v_along)` picks up vision-glitch frames (peak_v = 2505
  in one session, 5500 in another — physically impossible).
- `peak_v_t` happens at variable times relative to the tilt-phase
  boundary, so `integrated_tilt` covers different windows
  → variable G_eff.

Mitigations applied: 95th-percentile instead of np.max; multi-
frame consensus for the safety cutoff. Improved consistency from
51% → 40% CV but still not enough.

### Gen 3 (current): linear regression v(t) vs ∫θ(τ)dτ

The physics: `a = G·θ` integrates to `v = G·∫θ + C`. The slope of
`v(t)` plotted against `∫θ(τ)dτ` IS the plant gain.

```python
# 1. Median-filter ball position (kernel=5) — eats single-frame
#    V0 dropouts.
xs = _median_filter(xs_raw, 5)
ys = _median_filter(ys_raw, 5)
# 2. Velocity from smoothed position (central differences).
vx = np.gradient(xs, ts); vy = np.gradient(ys, ts)
v_along = vx · ax_dir + vy · ay_dir
# 3. Cumulative integral of |IMU tilt| from t=0.
cum_int_tilt = trapezoidal_integral(|imu_dominant|, ts)
# 4. Detect motion onset = first t where smoothed v > 30 mm/s
#    for ≥3 consecutive frames.
# 5. Linear regression v(t) vs cum_int_tilt(t) over
#    [motion_onset, end_of_samples]. slope = G_eff.
# 6. Drop residuals > 3σ, refit once.
```

**Why this is more robust**:
- All samples in motion window contribute (least-squares),
  not one extremum.
- Vision-glitch outlier residuals get dropped at 3σ.
- Cumulative integral handles non-constant tilt naturally
  (post-tilt-end ramp-down doesn't bias the slope).

**Status** (2026-05-01 evening): not enough sessions to fully
characterise, but appears to give significantly tighter CV than
peak-velocity. Still subject to fundamental vision-noise limits.

---

## Vision noise — the underlying constraint

Across all three generations of plant-ID and across both Demo 2
verification trials, vision noise was the dominant source of
trial-to-trial variation. Some specific manifestations:

- **Peak velocity 4500–5500 mm/s in verification trials** with a
  ball that physically tops out around 800. These were single-
  frame V0 false-positives where the detector locked onto a
  shadow / glare / hand briefly.
- **Trajectory looks "flat then jumps"** in the open-loop ball
  position record. V0 lost the ball during fast motion (the V0
  HSV failure mode documented in `vision_control_tuning_lessons.md`)
  and re-acquired at a new position 50+ mm later.
- **`/oak/health.v0_arr_hz`** typically 25–28 Hz across these
  sessions — 30 Hz target met. Detection *rate* is fine; it's
  detection *accuracy under motion blur* that fails.

**Mitigations applied (controller-side)**:
- Velocity sanity gate (`BT_MAX_BALL_VEL_MM_S = 800`) caps the
  velocity used by Kd. Removes the dominant energy-injection
  mechanism. Active by default.

**Mitigations applied (digest-side)**:
- Median-filter on ball position before velocity calculation in
  open-loop fit.
- 95th percentile instead of np.max for peak velocity.
- Linear regression with 3σ outlier rejection.

**What we did NOT try (but probably should before next attempt)**:
- Tuning the KF measurement noise R higher to dampen vision
  spikes at the source.
- Hampel-filter on /ball_state position with a 5-frame window.
- Switching V0 backend to "depth blob" (the parallel detector
  documented in `vision_control_tuning_lessons.md` — if it ever
  works on this surface).
- Increasing rgb_fps to 90 if the OAK supports it (more samples
  per dt → smaller per-frame outlier impact on velocity).

---

## Architectural changes shipped

In rough order of impact:

### Cascade-bypass for BALL_TRACK (commit `b9dee51`)

`_start_ball_track_loop` now stops the level-PI loop on entry and
re-enables it on exit (only if it was on before). BALL_TRACK
commands tilts directly via `_do_set_pose` / IK with no inner-loop
transient. Reverses the architectural decision in `controls_journey.md`
lesson #6 ("closing the inner tilt loop on IMU is essential").

**Tradeoff**: plant biases (leg backlash, IK approximations,
gravity sag) used to be absorbed by level-PI's IMU feedback. Now
they show up as ball-position bias which the outer integrator
must compensate over time. Slower bias rejection in exchange for
much faster transient response.

### Velocity sanity gate in PID (commit `f542b64`)

Cap `|edot|` at 800 mm/s before applying Kd in `_ball_track_run`.
Removes vision-noise-driven Kd saturation events. Single biggest
fix for the orbital limit cycle problem.

### Stiction relief in PID (commit `2eb6235`)

Borrows the bang-bang STICTION_BREAK pattern. When `vel_mag <
stiction_v_threshold AND err_mag > err_tol AND held > stiction_break_s`,
the PID's commanded tilt is scaled up to `max_tilt` while
preserving direction. Phase reports `STICTION_BREAK` (code 4).
Targets the "Kp·err < θ_s, ball stuck, integrator winds, eventual
overshoot" failure mode.

### PID latency lookahead (commit `a1fbfdb`)

`ex_lead = ex + edot_x · control_latency_s`. The PID's P term
reacts to where the ball will be when our tilt command actually
takes effect (~100 ms ahead), not where it was. Mathematically
equivalent to bumping Kd by `Kp·Td` — keeps the analytic
recommendations' predicted closed-loop poles intact in the
presence of cascade dead-time.

### Multi-frame consensus safety cutoff (commit `49fb700`)

The open-loop trial's safety cutoff (end tilt early if ball
exceeds velocity limit) used single-frame velocity, which
vision-noise spikes (1234 mm/s in one trial) tripped
inconsistently. Now requires 3 of last 5 samples over the limit
before cutting. Limit raised from 350 to 1500 mm/s — foam ring
handles real escape cases.

### Z-compensation cleanup on ball_track exit (commit `f91f49c`)

`_ball_track_run` does `z_cmd = z_hold + px·sin(pitch) − py·sin(roll)`
per tick to rotate platform UNDER the ball. Last tick before exit
left the platform at `z_hold ± ~10 mm` displacement. Level loop
then tracked that displaced pose forever — operator observed the
"~10 mm Z dip after every Demo 2 run" symptom.

Fix at loop exit: send `_do_set_pose(0, 0, z_hold, 0, 0, 0)`.
Whatever picks up after BALL_TRACK starts from a known clean
state.

---

## STEP_ID phase-by-phase: when to run what

The 2026-05-01 split into per-phase Run buttons (commit `1d49f1d`)
gives you these independently-runnable phases:

### 1. Stiction characterisation (`/step_id/run_stiction`)

**Run when**: starting a new tuning session on this hardware,
after physical changes (clean ball, replace platform surface,
new lighting), or to diagnose "ball doesn't break loose"
behaviour.

**Outputs**: `θ_s` (breakaway tilt), `μ_s` (friction coef),
recommended `max_tilt_deg ≥ θ_s + 2.5°`, recommended open-loop
tilt magnitude `≥ θ_s + 1°`.

**Time**: ~10–25 s wall-clock (ramp at 0.5°/s up to 8° max).

### 2. Open-loop plant ID (`/step_id/run_open_loop`)

**Run when**: stiction known, want analytic Kp/Kd recommendation.
Set the open-loop tilt magnitude in the GUI to `θ_s + 1°` first.

**Outputs**: G_eff (mean ± std across n replicates), recommended
Kp/Kd/Ki/max_tilt_deg in `step_id_recommendation.json`. Refuses
to recommend if CV > 0.30 across n≥2 replicates.

**Time**: ~3 minutes for n=3 default.

### 3. Closed-loop verification (`/step_id/run_verification`)

**Run when**: gains have been applied, want to see whether they
actually settle the ball before shipping. The 4-trial protocol
exercises both 92 mm and 170 mm step distances.

**Outputs**: per-trial settled/not, `verification_summary` in
`step_id_recommendation.json`, post-hoc `refined_gains` suggestion
if the trials reveal failure patterns (orbital, undershoot,
saturation, vision-noise jitter).

**Time**: ~3–6 minutes (each trial 25 s timeout + 3 s settle +
operator placement time).

### Full session (`/step_id/start`)

Runs all three in sequence. Use when starting completely fresh.

---

## What we tried that didn't work

- **YOLO v8 vs cv2 HSV.** Operator A/B test showed v8 had ~20%
  better detection accuracy on stationary balls under controlled
  lighting, but during actual demos v8 lost the ball almost
  immediately. Reverted boot default to cv2 HSV. v8 still
  available via runtime backend selector.
- **`OAK_FORCE_V1_ARCH=v8` then back to cv2.** Briefly committed
  to enable v8 by default; reverted within 30 minutes when v8
  proved worse for our specific motion profile.
- **Aggressive `Kp = 0.077`** from one analytic recommendation
  (G_eff=224 from a single replicate). Saturated max_tilt = 2.5°
  at any error > 32 mm. Drove the ball into orbital limit cycle
  every trial. Lesson: high Kp without high max_tilt is just
  bang-bang in disguise.
- **`OAK_FORCE_V1_ARCH=v8` with the existing recommendation.**
  Wanted to compare clean v8 trials against cv2 trials. Couldn't
  complete a single open-loop trial with v8 — vision lost ball
  every time.

---

## Open questions / what we never characterised

These would be worth doing on a future tuning round:

1. **Direct measurement of `θ_orb` / orbital velocity.** We have
   `θ_s` from the stiction phase. We never directly measured the
   orbital velocity that develops on this platform with the
   current vision noise + gain regime. Could add a 4th STEP_ID
   phase: drive ball into a known orbit at known tilt, observe
   steady-state orbital v, compute required braking tilt.
2. **KF measurement noise tuning.** We never adjusted the Kalman
   filter's R parameter to match measured vision noise variance.
   Would dampen velocity spikes at the source rather than at
   the controller's input.
3. **Inner-loop level-PI bandwidth measurement.** We disabled
   level-PI for BALL_TRACK because it was too slow; we never
   characterised exactly how slow. A short tilt-step trial with
   no ball + IMU recording would give us a 2nd-order fit on the
   inner loop.
4. **Vision exposure/gain trade-off under motion.** `controls_journey.md`
   covered exposure tuning for stationary detection. We didn't
   re-tune for motion — shorter exposure + brighter lighting +
   higher ISO might cut motion blur at the cost of noise.
5. **Detection rate vs control rate budget.** /oak/health shows
   `v0_arr_hz ≈ 27`, `v0_pub_hz ≈ 12.5`. The publish rate is
   capped somewhere; if we could get publish to match arrival
   we'd halve the effective vision latency.
6. **Plant gain at different Z heights.** All STEP_ID runs were
   at start_z = 80 mm. Foam ball normal force vs friction
   coefficient may vary with Z; closed-loop demos at the
   specified Z = 30–50 mm range may have different G_eff.

---

## Where the system stands as of 2026-05-01 evening

- ✓ Vision: cv2 HSV @ 27 Hz arrival, 12 Hz publish. v1_yolo
  available but worse for fast motion on this hardware.
- ✓ Cascade architecture: BALL_TRACK bypasses level-PI, commands
  tilts directly via IK.
- ✓ PID stiction relief active (boost to max_tilt when stuck).
- ✓ PID latency lookahead active (~100 ms phase compensation).
- ✓ Velocity sanity gate (Kd input capped at 800 mm/s).
- ✓ STEP_ID per-phase Run buttons in GUI.
- ✓ Stiction characterisation phase implemented and ready to use.
- ✓ Z-compensation cleanup on BALL_TRACK exit (no more 10 mm dip).
- ⚠ Demo 2 closed-loop tracking still showing orbital limit cycles
  in some configurations. Believed addressable with `max_tilt =
  max(θ_s + 2.5, 8)` + appropriate Kp/Kd combination — operator
  iteration in progress as of doc-write time.
- ⊘ Plant ID still produces 30–50% CV across replicates on this
  hardware. Vision noise is the bottleneck.

The Demo 2 tuning is operator-driven from here — typing values
into the Gains & Control Parameters panel, running Demo 2,
reading the next-step banner from the digest, iterating. The
infrastructure is in place; the limit is now physical (vision
noise, stiction variability) rather than software.

Onward.
