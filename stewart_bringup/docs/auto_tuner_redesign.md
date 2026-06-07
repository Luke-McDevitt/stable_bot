# Auto-tuner redesign — post-mortem + ideal design

**Purpose.** We have two abandoned tuning systems (STEP-ID and the hill-climb
auto-tuner) and went back to hand-tuning. This document (1) explains what each
was, how it was used, and **why both failed**, (2) surveys what researchers
actually do, and (3) specifies the auto-tuner I'd build from scratch — with
every edge case, the integration, the accept/revert logic, the GUI sync, and
how it uses the digest to *diagnose* rather than blindly search.

Read alongside `step_id_tuning_lessons.md` and `vision_control_tuning_lessons.md`
(the primary sources) and `auto_tuning_plan.md` (the original hill-climb plan).

---

## Part 1 — Post-mortem: what they were and why both failed

### They are two fundamentally different philosophies

| | **STEP-ID** | **Hill-climb auto-tuner** |
|---|---|---|
| Philosophy | **Model-based** (white-box) | **Model-free** (black-box) |
| What it measures | Plant gain `G_eff` from open-loop tilt-step tests | A closed-loop *fitness* score per demo trial |
| How it picks gains | **Analytic** — pole-placement formula `Kp = ωn²/G_eff`, `Kd = 2ζωn/G_eff` | **Search** — perturb a gain, keep it if fitness improved |
| Targets | Repeatable fixed steps (92 mm / 170 mm) | Fixed-distance **random-direction** gotos |
| Output | One Kp/Kd/Ki recommendation | A best-found gain vector after N trials |

STEP-ID asks *"what is the plant, so I can compute the right gains?"* The
auto-tuner asks *"which gains score best, never mind why?"* They are the two
canonical families (system-identification vs derivative-free optimization).

### Why STEP-ID failed: the plant measurement isn't repeatable

`G_eff` measured **6 times in one evening on identical hardware came out 35,
224, 92, 64, 148, 564** — a 5–10× spread on an in-principle-deterministic
number. The analytic gains are a *direct function* of `G_eff`, so garbage in →
garbage out: one fit's `Kp=0.077` "drove the ball into orbital limit cycle every
trial." Three generations of the fit (parabolic → peak-velocity → regression of
`v` vs `∫θ`) only reached **30–50% CV**. The root cause was **vision noise** —
single-frame detector false-positives spiking velocity to **4,500–5,500 mm/s**
on a ball that physically tops out ~800. A single-point identification can't
average that out.

### Why the auto-tuner failed: it can't tell bad gains from bad vision

The hill-climb ran (the `auto_tune_*` sessions exist) but never beat hand-tuning.
Its documented core risk: *"Vision instability poisons fitness scores. The
auto-tuner can't separate 'bad gains' from 'bad vision'."* Concretely:
- **Random-direction targets** mean every trial has different difficulty and a
  different interaction with the vision noise → the fitness signal is buried in
  trial-to-trial variance.
- **Greedy accept/reject on a single noisy trial** (a `noise_floor` band-aid)
  means the search wanders — it accepts lucky-vision trials and rejects good
  gains that drew an unlucky trial.
- **A scalar fitness with no diagnosis** — it knows trial 12 scored 0.6, but not
  *why* (over-damped? orbiting? steady-state offset?), so it can't move
  intelligently; it perturbs blindly.
- **A pre-flight frame/alignment blocker** (§0 of the plan: "click never matches
  tilt direction") — tuning on a wrong frame just finds gains that minimize
  wrong-direction error.

### The single shared root cause

**Both failed at the *evaluation*, not the *optimizer*.** A noisy, non-repeatable,
single-shot evaluation defeats *any* tuning method — model-based or model-free.
STEP-ID's noisy evaluation was `G_eff`; the auto-tuner's was the fitness score.
Fix the evaluation and either method can work; leave it broken and neither can.

**What changed since (the board is different now):**
- **Pi under-voltage throttling is fixed** (the BEC back-feed — see
  `pi_power_throttling.md`). The Pi was likely throttling *mid-trial* during
  those sessions → dropped vision frames + timing jitter = exactly the
  non-repeatability that poisoned both. The old sessions never had this fixed.
- **The #1 documented "should-try-next" was never done: tune the KF `R`.** The
  meas-noise data now exists (`R ≈ 0.2 mm`) to dampen the velocity spikes at the
  source.
- **We now have rich step-response metrics** (overshoot / ITAE / settle / path)
  and a **validated physics model** that can serve as a digital twin.

---

## Part 2 — What researchers actually do

Three families, in rough order of sophistication:

1. **Relay feedback (Åström–Hägglund).** Replace the controller with an on/off
   relay; the loop limit-cycles at the plant's critical frequency; read the
   ultimate gain `Ku` and period `Tu` from the oscillation and apply tuning
   rules. *"One of the simplest and most robust auto-tuning techniques… applied
   in industry for 15+ years… no preliminary experiments, modeling, or external
   tools."* Strength: one short, self-exciting experiment. Weakness for us: it
   measures *one* operating point and assumes a fairly linear plant — our plant
   has stiction + the two-tilt-threshold nonlinearity.

2. **Bayesian Optimization (the modern default for expensive/noisy tuning).**
   Fit a **Gaussian-process surrogate** to the (gains → score) data seen so far,
   then pick the next gains by maximizing an **acquisition function** (expected
   improvement). It is built for *"expensive and noisy black-box optimization"* —
   the GP explicitly models observation noise, so it doesn't chase lucky trials.
   Key variants directly relevant to us:
   - **Safe / constrained BO (SafeOpt)** — restrict evaluations to a region
     that's probabilistically *stable/safe*, so the optimizer never proposes
     gains likely to send the ball orbiting.
   - **Risk-averse BO under heteroscedastic noise** (e.g. RaGoOSE) — for when
     the noise itself varies with the gains (ours does: bad gains → fast ball →
     worse vision).
   - **Early-stopping BO** — abort a clearly-bad trial early to save time.
   - **Digital-twin BO** — explore on a *model* first, spend real trials only
     where the model is uncertain. **We have a model.**

3. **Iterative/data-driven tuning (IFT, VRFT).** Estimate the gradient of a
   closed-loop cost directly from experiment data and step the gains. Powerful
   but needs cleaner signals than we have today.

**The unanimous theme:** the optimizer matters less than (a) **replicated,
low-variance evaluation**, (b) **a noise-aware surrogate** so single trials don't
mislead, and (c) **explicit safety constraints**.

Sources: [Safe contextual BO for PID tuning](https://arxiv.org/pdf/1906.12086) ·
[Safe risk-averse BO for controller tuning](https://www.researchgate.net/publication/371855392_Safe_Risk-averse_Bayesian_Optimization_for_Controller_Tuning) ·
[Early-stopping BO for controller tuning](https://arxiv.org/pdf/2501.11532) ·
[Constrained BO for cascade controller tuning](https://www.researchgate.net/publication/359250332_Safety-Aware_Cascade_Controller_Tuning_Using_Constrained_Bayesian_Optimization) ·
[Relay feedback auto-tuning — tutorial review](https://www.sciencedirect.com/science/article/abs/pii/S0959152401000257) ·
[Relay auto-tuning via iterative feedback tuning](https://www.sciencedirect.com/science/article/abs/pii/S0005109802002017)

---

## Part 3 — The ideal auto-tuner

Seven principles, each fixing a specific failure above.

### P1. Repeatable evaluation — auto-ferry to a fixed start, fixed target
*(fixes the random-target variance; finishes what STEP-ID started with "repeatable
start/end points")*

The single biggest lever. Every trial must be **identical** so the only variable
is the gains:
- **Auto-ferry**: between trials, switch to the **champion** gain set and drive
  the ball to a canonical **start** position; confirm it's settled within a
  tight band (`|err| < 8 mm`, `|v| < 20 mm/s` for 1 s) before the trial begins.
  This removes manual ball placement — the thing the operator was doing by hand
  for the model A/B — as a variance source.
- **Fixed target**: send to the **same** target each trial (the operator's
  place-and-send move, automated). Use a **small panel of 2–4 canned moves**
  (e.g. 90 mm and 170 mm, two directions) so we tune for more than one operating
  point, but each *specific* move is repeated identically.
- **Verify the start** before scoring; if the ferry didn't land the ball in the
  band, re-ferry once, else mark the trial **invalid** (not "bad").

### P2. Statistical decision — replicate, gate on CV, accept only beyond noise
*(fixes single-shot accept/reject)*

- Each gain set is evaluated with **N≥3 replicates** of each canned move.
- Compute the objective's **median and spread** across replicates. If the
  **coefficient of variation exceeds a gate (CV > 0.30)** — STEP-ID's own rule —
  **refuse to decide**: collect more replicates or flag the run as
  vision-limited. A noisy evaluation is a *measurement failure*, not a gain
  verdict.
- A challenger only beats the champion if its objective is better by **more than
  the pooled noise** (a one-sided test on the replicate distributions, not a
  single number). This is exactly what BO's noise model does automatically.

### P3. A diagnostic objective, not a blind scalar — *use the digest to know what to change*
*(fixes "can't tell why a gain set is bad"; answers "use bag digest data to be
more educated")*

We now have per-goto **step metrics** (`overshoot_pct`, `rise_s`, `settle_s`,
`steady_state_err_mm`, `settled_rms_mm`, lateral/path, `reversals`, `IAE/ITAE`).
Use them two ways:

1. **As the objective** — minimize **ITAE** (the textbook single-number step
   quality, penalizes lingering error) with **overshoot and settle as
   constraints** (e.g. "minimize ITAE subject to overshoot ≤ 20%"). Far better
   signal than the old rms/hold/calm blend.
2. **As a diagnosis that *guides* the search** — map symptoms → moves the way a
   human expert does, to give the optimizer an informed prior:

   | digest symptom | likely cause | suggested move |
   |---|---|---|
   | high overshoot, oscillatory, ITAE high | under-damped | ↑ Kd, or ↓ Kp |
   | slow rise, sluggish, never overshoots | over-damped / weak | ↑ Kp |
   | settles with a constant offset (`steady_state_err`) | no integral authority | ↑ Ki |
   | orbital limit cycle, persists regardless | `max_tilt < θ_orb` **or** vision noise | ↑ max_tilt to `max(θ_s+2.5, 8)`; check vision |
   | ball never leaves start | `Kp·err < θ_s` (stiction) | ↑ Kp / ↑ max_tilt / stiction relief |
   | large lateral wander, wrong-direction error | frame/sign/alignment | **stop** — not a gain problem |
   | `settled_rms` high but mean ~0 | vision velocity noise | tune KF `R`, not gains |

   The optimizer (BO) does the fine search; the diagnosis table provides the
   *initial direction* and catches non-gain problems (frame, vision) so we don't
   waste trials "tuning" something gains can't fix.

### P4. Noise-aware, sample-efficient optimizer — Bayesian optimization
*(replaces greedy hill-climb)*

Fit a GP to `(gain_vector → objective)` with an explicit **noise term** equal to
the measured replicate variance, and choose the next gains by **expected
improvement**. This is the documented modern standard for noisy black-box
controller tuning and is far more sample-efficient than hill-climbing (which we
can't afford many trials of). Seed it with the diagnosis-table direction from P3
and the current champion. `scikit-optimize`/`BoTorch` provide this; if we can't
put them on the Pi, run the BO step **off-board** (the Pi only runs trials and
streams results) — the optimization is cheap, the trials are the expense.

### P5. Safety — bounded, stability-screened, vision-gated, always-revertible
*(fixes "could try unstable gains"; the operator's safety bar)*

- **Hard gain bounds**, clamped before any apply.
- **Constrained/safe BO**: never *propose* gains the surrogate predicts are
  unstable (orbital). Treat "ball escaped / violent oscillation" as a **safety
  constraint violation**, recorded so that region is avoided — not just a low
  score.
- **Vision-health gate** before/within each trial: check `vcgencmd
  get_throttled == 0x0`, `/oak/health` rate, and `/ball_state` freshness. A
  trial run under throttling or a vision dropout is **invalid**, not scored.
- **Early-abort** a trial the instant the ball escapes the ring, the error
  *grows* after the goto (sign/instability), or a time budget is hit → command
  LEVEL_HOLD, ferry back.
- **Foam ring + operator kill button** (instant LEVEL_HOLD + `abort=true`) +
  **auto-halt** on K consecutive invalid/sub-floor trials.

### P6. The physics model as a digital twin — pre-screen in sim
*(uniquely available to us now)*

Before spending a *real* trial on a proposed gain set, **simulate** the move with
the fitted `ball_model.yaml` + the controller, and reject anything that's clearly
unstable or much worse than the champion in sim. This (a) keeps the ball safe
(never run a predicted-orbital gain on hardware), and (b) multiplies sample
efficiency — the documented "digital-twin BO" idea. The twin won't be exact
(that's why we still validate on hardware), but it's an excellent **safety and
triage filter**.

### P7. Champion–challenger with full provenance and atomic apply
*(answers accept / revert / GUI-update)*

- A **champion** gain set is always the current best *validated* set. A
  **challenger** must beat it on a **held-out verification panel** (canned moves
  *not* used during the search), replicated, beyond noise, **and** pass the sim
  screen, to be **promoted**.
- **Apply is atomic**: promotion calls the existing `set_control_gains` (writes
  `ball_track_gains.yaml` + updates the live dict), which already rides
  `/status` so **the GUI Gains panel inputs auto-populate** — no second code
  path. (Same mechanism that now syncs the control-method dropdown.)
- **Revert is trivial and total**: every gain set + its full metric distribution
  is logged to `tuning_data/auto_tune_<UTC>/log.jsonl`; the champion before the
  session is saved; a **"Revert to champion"** GUI button re-applies it; and
  because the yaml is git-tracked, `git checkout` is the ultimate undo. Nothing
  is overwritten without the prior value preserved.

---

## Part 4 — Every edge case

| # | Edge case | Detection | Handling |
|---|---|---|---|
| 1 | Ball escapes the ring | `/ball_state` stale > 0.5 s, or `|pos| > R` | abort trial → **invalid**, LEVEL_HOLD, ferry, retry once |
| 2 | Vision dropout mid-trial | `/ball_state` gap, `/oak/health` rate low | mark trial **invalid** (not scored); pause if persistent |
| 3 | Pi throttling during trial | `get_throttled != 0x0` | **invalid**; pause session, surface alert (should be rare post-BEC-fix) |
| 4 | Ferry fails to seat the ball at start | start band not met after re-ferry | **invalid** trial; if repeated, halt — likely champion can't hold |
| 5 | Ball never leaves start (stiction) | `max(progress) < min_travel` over the move | score = worst-feasible **with** diagnosis "below stiction"; informs ↑Kp/↑max_tilt |
| 6 | Orbital limit cycle | sustained `reversals`/speed, never settles | early-abort (safety); record region as **unsafe** for BO; check `max_tilt` vs θ_orb and vision |
| 7 | Sign / frame flip | error *grows* after goto; large lateral | **stop the session**, alert "not a gain problem" (P3 table) |
| 8 | Unstable gain proposed | sim screen (P6) or constrained BO predicts orbital | **never run on hardware**; BO avoids the region |
| 9 | High replicate variance | CV > 0.30 | **refuse to decide**; more replicates or flag vision-limited |
| 10 | Degenerate metrics (no move → no overshoot/settle) | `step_metrics` returns None | handle as invalid/worst-feasible, never NaN into the objective |
| 11 | Champion drifts (hardware change) | periodic champion re-validation worse than logged | re-baseline; warn operator |
| 12 | Optimizer proposes out-of-bounds | always | clamp to hard bounds before apply |
| 13 | Conflicting objectives (faster but more overshoot) | constraint violated | constrained objective (min ITAE s.t. overshoot ≤ cap) resolves it deterministically |
| 14 | Operator kill mid-trial | kill button | immediate LEVEL_HOLD, `abort=true`, champion preserved, state saved |
| 15 | Tuner/process crash | watchdog | champion is atomic + on disk; resume or auto-revert; never leave a half-applied gain set |
| 16 | Time / thermal budget | elapsed, `measure_temp` | graceful stop at budget; champion already safe |
| 17 | Steady-state offset vs oscillation ambiguity | `steady_state_err` vs `reversals` distinguish | diagnosis routes to Ki vs Kd — not conflated |
| 18 | Vision-noise jitter masquerading as instability | `settled_rms` high, mean ≈ 0, low reversals after smoothing | flagged as **measurement**, routes to KF-`R`, not a gain change |

---

## Part 5 — How we'll *know* it actually helped (validation)

Decisions are only trusted against a **held-out verification panel** — fixed
canned moves reserved from the search (so we measure generalization, not
overfitting to the trials we optimized on):
- Run the **old champion** and the **new challenger** back-to-back on the
  verification panel, **N replicates each, interleaved**, on a vision-healthy,
  un-throttled Pi.
- Compare the **step-metric distributions** (ITAE, overshoot, settle), not single
  runs, and require the improvement to **exceed the pooled noise**.
- This is the rigorous version of STEP-ID's verification phase + the manual model
  A/B we just did — same identical-move discipline, now replicated and
  statistically gated.

A change is "real" only if it (a) wins on the held-out panel, (b) beyond noise,
(c) without violating a constraint (overshoot/safety), and (d) the win survives a
re-test (no champion drift). Otherwise → revert to champion.

---

## Part 6 — Prerequisites before any of this runs

In strict order (each is a thing that *defeated* the old tuners):
1. **Tune the KF `R` / smooth the velocity** — kill the vision-velocity spikes at
   the source (the documented #1 unfinished item; also cuts the ~11 mm
   steady-state jitter). *Without this, every evaluation is still noisy.*
2. **Confirm vision health + click→tilt direction** on the un-throttled Pi
   (the §0 frame test).
3. **Confirm `max_tilt ≥ max(θ_s + 2.5, 8)`** so the search can't be sabotaged by
   the orbital-braking floor.
4. Only then build the tuner above — **auto-ferry + replicated + diagnostic-ITAE
   + constrained/safe BO + sim-screen + champion/challenger**.

The order matters: the optimizer is the *last* and least important piece. The old
tuners had decent optimizers and still failed, because the evaluation underneath
them was broken. Fix the evaluation first.
