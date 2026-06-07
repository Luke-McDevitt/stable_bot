# Physics-model control — implementation plan & interaction map

**Companion to `ball_physics_modeling_plan.md`** (which has the physics, the
data campaigns §5, the §15 GUI sketch, and the §13 logging). *That* doc says
*what physics to capture*; *this* doc says *exactly what code changes wire it
into the live control schema*, phase by phase, with every interaction point
so we can flip PID ⇄ model and know which ran in every bag.

Driven by the 2026-06 latency work (`latency_headroom_journey.md`): the loop
is ~264 ms, decoupled from CPU, and the controller already predicts the ball
`Td` ahead — so the model is a *better predictor*, not a latency fix.

## Decisions resolved (operator + recommendation, 2026-06-06)

- **Z dependence → one model, validate across Z; don't pre-build per-Z.**
  The ball *rolling* dynamics (`a = (5/7)g·sinθ`, friction ∝ `mg·cosθ`) are
  Z-independent at steady Z — Z is the heave, not the tilt. The Jacobian's
  Z-dependence is on the *actuation* side (leg-motion→tilt), which the model
  doesn't touch (it predicts ball-given-*measured*-tilt). So fit one model at
  the demo Z, then run a held-out check across Z; only add a Z term if the
  horizon error grows at the extremes.
- **Platform regions (center/middle/edge) → one model, but DO collect
  coverage across regions.** For a rigid, uniform plate the physics is
  position-independent. You don't need a model *per* region — but you *do*
  want data spanning regions so the fit (a) is validated everywhere and (b)
  surfaces any surface-friction inhomogeneity as a region-dependent residual.
  If that residual shows up, add a position-dependent friction term (or a GP
  residual) — let the data decide. The coverage-grid campaign (§5G) drives
  this for free.
- **Excitation → chirp above the stiction breakaway + the dedicated
  breakaway campaign (§5B).** Our system is stiction-dominated (needs the
  startup ramp to break loose), so a small-amplitude chirp won't move the
  ball. Use chirps at amplitudes that actually roll it for the rolling/
  frequency content, and campaign B for the static/rate/dwell breakaway that
  the LuGre (Tier-3) friction term needs. PRBS only if the spectra need it.
- **IMU → platform IMU is primary; log both (free).** The platform IMU gives
  tilt + ω + plate accel (the Tier-2 coupling terms). A base IMU only matters
  if the *rig* moves; for a fixed base it's redundant. `imu_dual` already
  records both, so keep logging both and use the base one only if a coupling
  residual demands it.
- **Tune KF `R` first → yes, cheap, do it in Phase 0/1.** Damping vision
  spikes at the source may matter more than model form (plan §10/§19). It's a
  `ball_kf_node` measurement-noise parameter; raising `R` (or gating outlier
  jumps) trades responsiveness for smoothness. Low-risk A/B.

## Phase 0 — control-method switch — SHIPPED ✅

The infrastructure to flip PID ⇄ model, behind a flag, default-off and
**provably behaviour-neutral** until a model exists. Deploy is zero-risk.

**Interaction map (what touches what):**

| piece | file:symbol | role |
|---|---|---|
| predictor | `_ball_predictor.py:predict_lead()` | pure lead-position fn; `const_vel` = `x+v·Td` (exact), `model` = `model.integrate(...)` with a const-vel fallback (a model bug can't command a wild tilt). |
| model loader | `_ball_predictor.py:load_ball_model()` | returns None today → const-velocity everywhere. Phase 1 returns a fitted model. |
| the flag | `ball_track_gains['use_model_predictor']` (default 0) | rides the gains dict → `/status` → digest. |
| read site | `stewart_control_node._predict_lead()` | picks method from the flag + `self._ball_model`. |
| swap site 1 (bang-bang) | `_ball_track_run` ~L4627 | `if use_model: px_lead,py_lead = _predict_lead(...) else: <legacy inline>` |
| swap site 2 (PID) | `_ball_track_run` ~L4707 | same branch; `ex_lead = ex + (px_lead-px)` (== legacy for a static ref) |
| set command | `/control_cmd` `set_control_method:{value: pid\|model}` | targeted single-flag set (doesn't touch other gains), mirrors `set_speed_cap`. |
| GUI | demos panel `#demo-control-method` dropdown | publishes `set_control_method`. |
| label | `digest_demo_bag.summary.control_method` + stdout | `'model'` iff `gains_at_record.use_model_predictor`. **Zero new bag topics.** |
| artifact path | `config/ball_model.yaml` (`BALL_MODEL_PATH`) | absent today; Phase-1 fitter writes it. |
| tests | `test/test_ball_predictor.py` (6, pass) | pins const-vel exactness + stub neutrality + the broken-model fallback. |

**To use it now:** deploy + flip the demos "control method" dropdown to
*Physics model* — it currently does nothing (const-velocity fallback), which
is the point: it proves the plumbing (dropdown → `/status` → digest label)
before any model risk.

## Phase 1 — the modeling pipeline (the real build)

**Build (plan §9), in dependency order:**
1. `scripts/fit_ball_forward_model.py` — offline gray-box fitter (Tier-1→2→3,
   optional GP residual), writes `config/ball_model.yaml` + a fit report.
2. `_ball_physics.py` — Tier-2 coupling + Tier-3 LuGre/rolling-resistance.
3. The model class with `.integrate(px,py,vx,vy,td_s,tilt_history)` — drop it
   into `load_ball_model()`; `predict_lead`'s `'model'` branch already calls it.
4. **Control-node Phase-1 adds:** a commanded-tilt **ring buffer**
   (`self._tilt_history`, append each tick) so the model integrates through
   the known input; and **per-sample `Td`** (`Td = /oak/latency_ms + fixed
   actuation_offset` instead of the fixed `control_latency_s` — plan §19.1,
   kills the jitter term; gate behind its own flag for an A/B).
5. Excitation modes in the control node, driven by `/control_cmd`
   `data_collect:<campaign>:<json>` (§15).

**The "Physics Modeler" GUI panel — top of the right column** (3rd
`<section class="lg:col-span-1">`; auto-tune / step-id / latency-bench shift
down — auto-tune is already env-disabled so it's inert). Clone the STEP_ID
endpoints (`/data_collect/bags`, `/.../digest`, `/.../png`, `/.../push`):
- per-campaign **Run** buttons (meas-noise, breakaway, coasting, step-ID,
  sweep, coupling, coverage-grid),
- a **live coverage heatmap** (state-space bins → fill the next under-sampled
  cell, incl. the platform-region coverage above),
- a **run-QA strip** (axis_state / feeder_mode / errors / latency / det-rate;
  auto-abort + `_fault` tag — plan §14),
- a **session list** (digest/PNG/push) and a **model panel** (the §18
  sim-vs-real error viz so you watch the fit close on reality).

**Logging (plan §13, manifest v2)** — for the *data-collection* bags only
(the demo digest already has what it needs): expand the allowlist to the
dropped-but-needed topics (`/oak/latency_ms`, `/oak/health`,
`/ball_track/diagnostic`, `/level_diag`, `/leg_currents`, per-motor
`RobotState`, CAN traffic) + a per-bag metadata sidecar.

## Robust tests (the report card)

1. **Unit (pytest, pure):** Tier-1/2/3 integration vs analytic cases;
   `predict_lead` const-vel exactness + model-stub neutrality (done);
   the **fitter recovers known params** from synthetic data (inject θ_s/μ_s/
   G_eff → fit → assert recovery); §14 QA gates fire on synthetic bad runs.
2. **Validation:** input-replay **sim-vs-real**; the **horizon-error curve**
   (predicted-vs-actual at lookahead 0→264 ms on *held-out* bags) — this is
   the "is 264 ms inside the trustworthy horizon" answer; learning curve
   (error vs data quantity → §17 stopping criteria).
3. **A/B (the payoff):** PID vs model over the same click-to-goto set —
   tracking rms/p95/settling **and** horizon-error, side-by-side in
   `compare_demo_bags`, auto-labeled by `summary.control_method`. **Instant
   fallback = flip the dropdown to PID.**
4. **Safety / HIL:** the model path inherits PID's `max_tilt` clamp and the
   const-velocity fallback (a model exception can't escape it); audit the
   lost-ball → hold-level/disarm watchdog *before* trusting the model online.

## Sequencing / open items

- Phase 0 is shipped + safe to deploy now (validates the switch end-to-end).
- Phase 1 minimal core = campaigns **B/C/D + coverage grid** → a Tier-1/2
  predictor + the horizon-error curve. Resist building all 7 campaign
  buttons before the first model proves out. Sim/MPC (plan §8.3) waits.
- Phase-0 gaps to close in Phase 1: reflect the dropdown from `/status`
  (cosmetic); the tilt-history ring buffer; per-sample `Td`.
- The **`R`-tune** and **per-sample `Td`** are independent quick wins worth
  doing alongside, since the *jitter* (not the mean) drives the limit cycle.
