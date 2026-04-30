# Stable-Bot autonomous-tuning plan

**Author intent (operator):** "automated training schema where the program
autonomously runs trials, analyzes the data, then tweaks the control values,
tries again, uses something like gradient ascent to optimize. Plus a
performance-over-time graph. I have 1 day."

**Status:** plan only — no code yet. Approve before implementing.

---

## 0. Pre-flight investigation (before any auto-tuning runs)

The operator reports: **"where I click on the SVG has never once been the
direction that the ball is tilted towards"**, and even after the
`aruco_imu_alignment.yaml` refresh (rotation 161° → 158°, RMS 0.80° → 0.50°),
this persists with bang-bang or PID. That is **not a tuning problem** —
no gain set fixes a frame mismatch. Auto-tuning that runs on top of a wrong
frame just finds gains that minimize wrong-direction error, which is worse
than useless.

### What needs to be confirmed first

The pipeline (as I understand it from the code):

```
SVG click (platform-frame mm)
      │
      ▼
/control_cmd = "mode:BALL_TRACK_GOTO {x_mm, y_mm}"
      │
      ▼
ref_generator_node → /ball_ref (PointStamped, in ??? frame)
      │
      ▼          ┌─ ball detection → ball_localizer
      │          │     applies aruco_imu_alignment rotation
      │          │     → /ball_xy_mono (in IMU frame, mm)
      │          ▼
      │       ball_kf_node → /ball_state (in IMU frame)
      │          │
      ▼          ▼
stewart_control_node BALL_TRACK loop:
  err = ball_ref − ball_state
  command pitch/roll proportional to err
```

**The error subtraction must operate in a single frame.** If `/ball_ref`
carries platform-frame coordinates and `/ball_state` carries IMU-frame
coordinates, the controller computes garbage and tilts in arbitrary
directions. Symptom: "click never matches tilt direction." This is the
operator's exact observation.

### Investigation checklist (before tuning)

1. Read `stewart_vision/stewart_vision/ref_generator_node.py` — does it
   apply the same `aruco_imu_alignment.yaml` rotation that `ball_localizer`
   does? If not, the fix is **applying the rotation to the click target
   too** (or pulling both back to platform frame).
2. Read `stewart_bringup/stewart_bringup/stewart_control_node.py` BALL_TRACK
   loop — confirm that the error vector is computed in a consistent frame
   and that `pitch_sign`/`roll_sign` map IMU-frame error to the correct
   tilt axes.
3. Read `ball_localizer_node._apply_aruco_imu_rotation` — confirm the
   rotation direction (camera → IMU vs IMU → camera) matches what
   downstream nodes expect.
4. **Quick empirical test**: place ball at center, click target at
   `(+50, 0)` (purely +X). Watch `/platform_rpy` in journalctl while the
   controller runs. Pitch (or roll, depending on convention) should
   develop a sign matching the desired tilt. If not, that's the bug.

If any of items 1–3 reveal a frame mismatch, the fix is a 5-line
correction in one of those files. **Do this first.** Don't start
auto-tuning until item 4's empirical test passes — clicking +X tilts
the ball roughly +X.

---

## 1. Goals + budget

- **Goal:** find a controller configuration (gains + ArUco-IMU rotation
  offset) that minimizes a fitness function over a sample of random
  goto-targets.
- **Hard budget:** 1 calendar day, including all fixes from §0.
- **Soft budget:** 50 trials per tuning session (≈ 25 min wall-clock at
  30 s/trial); 2–3 sessions max. Anything longer needs operator
  oversight against ball escape / battery / OAK overheating.

---

## 2. Hardware setup

- **Paper / foam ring** around the platform rim — ball cannot fall off.
  Eliminates the most common trial-abort failure mode (operator chasing
  the ball) and lets the algorithm run to its timeout reliably.
- **IKEA panel light** (when it arrives) — eliminates the low-light
  detection regressions from earlier in the day.
- **Headlamp as backup** if the panel can't be in place yet.

---

## 3. Fitness function

Each trial = one goto. Targets are random within the safe region.
Fitness ∈ [0, 1], higher = better. Composition (operator can tune
weights after seeing the first 10–20 hand-picked trials):

| component | meaning | weight |
|---|---|---|
| `f_err` | `1 / (1 + rms_error_mm / 50)` — 1.0 at 0 mm, 0.5 at 50 mm | 0.25 |
| `f_p95` | `1 / (1 + p95_error_mm / 100)` — punishes tail oscillations | 0.15 |
| `f_settle` | `max(0, 1 − settling_time_s / 15)` — 1 if settled in 0 s, 0 if ≥ 15 s | 0.15 |
| `f_hold` | `on_target_fraction` (frac of run with err < 25 mm tolerance) | 0.25 |
| `f_calm` | `1 / (1 + mean_speed_mm_per_s / 100)` — punishes a "tracking" ball that's actually orbiting fast through target | 0.20 |

`fitness = 0.25·f_err + 0.15·f_p95 + 0.15·f_settle + 0.25·f_hold + 0.20·f_calm`

`on_target_fraction` and `mean_speed_mm_per_s` are new fields that
need to be computed from the bag during digest. Today's digest has
rms / p95 / settling already.

### Why `f_calm`?

Without a velocity penalty, an out-of-tune controller that drives
the ball in a tight circle that crosses the target 5 times per
second can score a misleadingly low rms / high hold-fraction. The
ball "looks like it's tracking" but the system is unstable. `f_calm`
breaks the tie by rewarding the slower, more deliberate trajectory.
A well-tuned controller settles at the target and stops, scoring
near 1.0 on `f_calm`. A spinning controller drops below 0.5 and
loses the optimization race even if its rms is low.

### Trial protocol — fixed-distance random targets

Each trial picks a target at a **fixed distance D from the current
ball position** (D = 60 mm default), with a uniform-random direction.
Standardizing the per-trial distance is critical:

- **Without it**: a target generated 10 mm from the ball gets a
  near-zero rms with almost any controller, while a 180 mm target is
  hard for any controller. Mixing both into the same fitness pool
  creates noise that no number of trials can average out.
- **With it**: every trial has the same nominal difficulty, fitness
  comparisons across trials are meaningful, and the optimizer
  converges from clean signal.

```
ball_xy = read_current_ball_state()       # via /ball_state subscriber
for attempt in 1..16:
    θ = uniform(0, 2π)
    target = ball_xy + D · (cos θ, sin θ)
    if |target| < 0.7·R_platform:
        break
# 16 attempts all off-platform → ball is near the rim. Shrink D
# for this trial only:  D_trial = D × 0.7  and retry once.
```

`D = 60 mm` is a sensible default — large enough that controller
quality matters (the ball has room to oscillate), small enough that
a reasonable PID can settle within the 25 s timeout.

### Sanity-check the fitness before unleashing the algorithm

Run **5 hand-picked gain sets** (e.g., the current best PID, a slightly-too-loose,
a slightly-too-tight, a clearly-bad-direction, and bang-bang). Compute
fitness for each. The ranking the function produces must match the
ranking your eyes produce. If it doesn't, reweight components before
auto-tuning starts.

---

## 4. Search space

### Gains being optimized

PID-only (no bang-bang in Phase 1 — its stiction-breakthrough makes
fitness too discontinuous for any gradient-style method to converge):

| variable | bounds | initial step | rationale |
|---|---|---|---|
| `kp` | [0.001, 0.050] | 0.002 | proportional gain |
| `kd` | [0.000, 0.100] | 0.005 | derivative gain (damping) |
| `ki` | [0.000, 0.050] | 0.002 | integral (steady-state offset) |
| `max_tilt_deg` | [1.0, 8.0] | 0.5 | absolute tilt clip |
| `pitch_sign` | {±1} | flip | sign-flip search (categorical) |
| `roll_sign` | {±1} | flip | sign-flip search (categorical) |
| **`alignment_offset_deg`** | [−15.0, +15.0] | 1.0 | residual rotation on top of `aruco_imu_alignment.yaml` (operator addition: in case the IVA fit is wrong) |

That's 6 continuous + 2 categorical = effectively 8 dimensions.

### Why alignment_offset is included

Even after re-running IVA, the operator's "click never matches tilt
direction" observation suggests the alignment may be off by enough that
no PID gains can compensate. Letting the algorithm search a small
±15° rotation offset on top of the IVA-fit alignment gives it a
degree of freedom to discover whether the alignment is the bottleneck.
The auto-tuner reads `aruco_imu_alignment.yaml`, applies an additional
rotation by `alignment_offset_deg` before passing to ball_localizer.

If the algorithm consistently pulls `alignment_offset_deg` to a
nonzero value to maximize fitness, that's a strong signal the IVA fit
was bad — and the operator should re-run IVA after auto-tuning,
applying the discovered offset, then re-tune.

### Bang-bang search (Phase 1.5, if Phase 1 isn't enough)

The operator notes bang-bang was "much less jittery." If PID's best
fitness after 50 trials is still under (say) 0.5, run a smaller search
in bang-bang space:
- `accel_tilt_deg`, `brake_tilt_deg`, `v_brake_mm_s`, `v_coast_mm_s`,
  `stiction_break_s`. Hold PID gains as a fallback. Same fitness function.

Do **not** mix the two algorithms in the same search — discontinuous
fitness landscape, optimizer can't reason about the boundary.

---

## 5. Algorithm

### Phase 1 — coordinate-descent hill climbing (trials 1–30)

```
gains = current_best
fitness_best = evaluate(gains)
for trial in 1..30:
    var = pick_variable()                      # round-robin or random
    direction = +1 or -1                       # alternate or random
    test = clamp(gains[var] + step[var]·direction, bounds[var])
    f = evaluate(test)                         # one full demo trial
    if f > fitness_best + noise_floor:
        gains[var] = test
        fitness_best = f
        consec_no_improve[var] = 0
    else:
        consec_no_improve[var] += 1
        if consec_no_improve[var] >= 3:
            step[var] *= 0.7                   # anneal step
```

`noise_floor`: 0.02. Below that, accept-or-reject is dominated by
trial-to-trial variation, not real gain difference.

### Phase 2 — Bayesian Optimization (trials 31–50)

Once Phase 1 has 30+ samples, switch to BO via `scikit-optimize`.
- Fit a Gaussian process to all `(gain_vector, fitness)` pairs so far.
- Pick the next gain vector by maximizing **expected improvement** under
  the GP posterior.
- BO is *sample-efficient* — typically halves the trials needed to
  converge from where hill-climbing leaves off, by exploring strategic
  rather than greedy directions.

### "Is this gradient ascent?" — short answer

No, not in the calculus sense. There's no analytical ∂fitness/∂gains
because each evaluation requires running a physical demo. We use
**derivative-free optimization**, which has the same goal (maximize
fitness) but uses different machinery:

- Hill climbing = greedy local search, accept-or-reject perturbations.
- BO = builds a probabilistic *model* of the fitness function from
  observations, queries the model to decide where to evaluate next.

Both *behave* like gradient ascent in well-behaved regions of the
parameter space, without ever computing a gradient. SPSA and CMA-ES
are alternatives that we'd consider at higher dimension counts (>15);
ours is small enough that hill climbing → BO is the right ladder.

---

## 6. Logging + visibility

Each trial appends a JSONL row to
`tuning_data/auto_tune_<UTC>/log.jsonl`:

```json
{
  "trial": 12,
  "ts": "2026-04-30T18:22:31Z",
  "gains": {"kp": 0.012, "kd": 0.015, "ki": 0.005,
            "max_tilt_deg": 3.0, "pitch_sign": 1.0, "roll_sign": 1.0,
            "alignment_offset_deg": 0.0},
  "target": {"x_mm": 73.4, "y_mm": -41.8},
  "ball_start": {"x_mm": -22.0, "y_mm": 5.5},
  "fitness": 0.612,
  "components": {"f_err": 0.45, "f_p95": 0.30,
                 "f_settle": 0.55, "f_hold": 0.85},
  "settled": true,
  "settling_time_s": 4.2,
  "duration_s": 12.4,
  "bag": "tuning_data/20260430T182231Z_auto_tune_trial_0012",
  "algo": "hill_climb",
  "step_taken": "kp +0.002 (accepted)"
}
```

A separate `summary.json` is updated each trial with running best:

```json
{
  "n_trials": 12,
  "best_fitness": 0.71,
  "best_gains": {...},
  "best_trial": 8,
  "elapsed_s": 374
}
```

### Plot artifacts (auto-generated each trial)

- `fitness_curve.png` — fitness vs. trial number, with running max
  overlaid. The training curve.
- `gain_trajectory.png` — each gain plotted vs. trial. Shows which
  knob the algorithm leaned on and when.
- `target_coverage.png` — scatter of trial targets on the platform,
  colored by trial fitness. Shows whether the algorithm saw a
  representative sample.

### GUI panel

New "Auto-Tune" panel (right column) with:

- **Start / Stop / Abort** buttons
- Live `n_trials / max_trials` + `best_fitness` + `elapsed_s`
- Embedded sparkline of fitness over time (read JSONL via fetch)
- Current-best gains display + **"Promote to active"** button (copies
  best gains into `ball_track_gains.yaml`, leaves auto-tuner running)
- ETA estimate

---

## 7. Safety

- **Hard gain bounds** clamped before any apply.
- **Sanity guard**: if 5 consecutive trials produce fitness < 0.10,
  halt the run and surface a notification — controller is broken
  (sign flip during search, alignment offset gone wrong, ball escaped
  the ring). Operator manually intervenes.
- **Inter-trial pause**: 2 s with `mode:LEVEL_HOLD` between trials so
  the ball settles. Optional "ferry" with a known-good PID set if
  ball can't settle on its own — only if needed (some random gain
  combos won't even hold position between trials).
- **Operator kill button** in the GUI: publishes `mode:LEVEL_HOLD`
  and sets `auto_tune.abort = true` immediately.

---

## 8. Implementation order (build sequence)

In the order I'd ship if approved:

1. **§0 investigation** — read the three files, run the empirical
   click-direction test, fix any frame mismatch found. **Estimated:
   1–2 hours. Blocking everything else.**
2. **`on_target_fraction` and `mean_speed_mm_per_s` fields in
   `digest_demo_bag.py`.** Both derived from /ball_state — first is
   the fraction of state samples within `err_tol_mm` of the latest
   ref, second is the mean speed from the KF velocity field. ~30 min
   total.
3. **Fitness sanity check** — 5 hand-picked trials, manually compute
   fitness, confirm ranking. ~30 min.
4. **`auto_tune_node.py`** — new ROS node, trial loop, hill-climb
   algorithm, JSONL log writer. ~2 hours.
5. **Plotting script** — generates the three PNGs from the JSONL log.
   ~30 min.
6. **GUI panel** — start/stop/abort, sparkline, current-best display.
   ~1 hour.
7. **First 50-trial run** + analysis. ~30 min.
8. (Stretch) Bayesian Optimization swap-in. ~1.5 hours if Phase 1
   isn't enough.

Total at ~6–8 hours of build + iterate. Fits within the 1-day budget
*if* §0 doesn't reveal a deep architectural rework.

---

## 9. Risks

- **§0 reveals a deep architectural problem.** If `/ball_ref` and
  `/ball_state` are in different frames and the fix is non-trivial,
  pivot the day to fixing that and use the auto-tuner only at the
  end. Without correct frames, no tuning works.
- **Fitness function rewards wrong behavior.** Mitigation: sanity-check
  step §3.
- **Bang-bang turns out to be the only thing that works** at this
  hardware. Pivot to bang-bang search (§4 stretch) if PID's best is
  poor after Phase 1.
- **Ball escape despite ring** (rare with proper foam ring). Handle
  by detecting "ball lost" via /ball_state staleness, abort trial,
  prompt operator.
- **Vision instability** (low FPS, high latency, NN false-positives)
  poisons fitness scores. The auto-tuner can't separate "bad gains"
  from "bad vision" — make sure vision is healthy *before* starting
  a tuning session by inspecting `[health]` log lines in the GUI.

---

## 10. What's NOT in scope today

- Online Reinforcement Learning (model-based, actor-critic, etc.) —
  needs many more trials than we have time for.
- Cross-target generalization beyond goto — orbit / path tuning is
  separate.
- Bandit / contextual algorithms that pick targets based on what's
  most informative — fixed uniform-random sampling is fine for the
  budget.

---

## Decision points to resolve before I start coding

1. **§0 fix can come from me?** I can read the three files and
   propose the frame-mismatch fix as part of this work. Operator
   confirms before I push.
2. **Approve the fitness weights** — 0.30 / 0.20 / 0.20 / 0.30 are
   guesses; any prior intuition?
3. **Approve the search space + bounds.**
4. **Algorithm: hill climb only, or hill climb → BO?** I lean both,
   but BO needs `scikit-optimize` installed on the Pi (not currently
   there). Hill-climb-only fits cleanly within the day.
5. **GUI panel: minimal (start/stop + counter) or full (sparkline +
   current-best)?** Minimal can ship faster.

When you green-light these, I'll start with §0.
