# Ball Physics Modeling & Data-Collection Plan

**Added 2026-06-03.** Research-grounded plan for replacing the
constant-velocity "look-forward" (and ultimately the PID + stiction-break
loop) with a **data-driven physics model** of the ball + platform,
fit from ROS bags of the real hardware. This is the deep-dive companion
to [NEXT_STEPS.md → Phase 2](NEXT_STEPS.md#phase-2--data-driven-ball--platform-forward-model-look-forward-upgrade).

The goal: a simulation accurate enough that (a) the controller's
forward-prediction is right on transients, and (b) we can develop and
tune a better-than-PID controller in sim before touching hardware.

> **Part I** (§1–11) is the physics + data-collection theory.
> **[Part II](#part-ii--execution-gui-run-procedure-data-sufficiency-robust-logging) (§12–18)**
> is the execution layer added after a deeper repo audit: what telemetry
> we can actually log, the robust-logging gap, GUI integration, the
> per-run operator procedure, the "when do we have enough data?"
> stopping criteria, auto-QA, and the digest error visualizations.

---

## 1. Why the current model is not enough

Today the controller predicts the ball with `x̂(t+Td) = x + v·Td` — a
straight line — and the KF behind it (`ball_kf_node.py`) is also
constant-velocity. That model is blind to every force actually acting on
the ball:

- the gravity-projection acceleration `α·g·sinθ` from the tilt the plate
  is *currently* commanded to (the one term we already wrote down in
  `_ball_physics.py`, but only use for BALL_HOLD feedforward);
- friction in all its forms (breakaway, Stribeck, rolling resistance);
- the coupling from the **platform itself moving** (a Stewart platform
  doesn't just tilt — it translates, and can rotate, under the ball).

The straight-line model is right only when the ball is coasting at
constant speed on a level, still plate — i.e. almost never during a
demo. Everything below is about measuring the missing physics.

---

## 2. The physics — model hierarchy

We build the model in tiers, each adding physics the previous one
ignored. Fit only as deep as the validation error requires.

### Tier 1 — Rolling core (no-slip), the textbook ball-on-plate

A solid sphere rolling without slipping on a plane tilted by `θ`:

```
(1 + I/(m R²)) · ẍ = g · sin θ
→  ẍ = α · g · sin θ ,   α = 1/(1 + I/(mR²))
   α = 5/7 (solid foam),  3/5 (hollow ping-pong)     [matches _ball_physics.py]
```

The rotational inertia is why the ball accelerates at `5/7 g sinθ`, not
`g sinθ`. This is the standard ball-and-plate benchmark model
([Cal Poly thesis](https://digitalcommons.calpoly.edu/theses/),
[Springer 2024](https://link.springer.com/article/10.1007/s11768-024-00208-8)).
**What's missing: everything dissipative and everything from plate motion.**

### Tier 2 — Rotating / translating plate coupling (Stewart-specific)

The benchmark literature assumes a plate that only tilts about a fixed
point. Our platform is a 6-DOF Stewart mechanism: the plate **translates
and can yaw** under the ball. In the plate frame, a ball at position
`r = (px, py)` on a plate with angular velocity `ω` and the base
accelerating at `a_base` sees extra terms:

```
m_eff · r̈ = m g·(in-plane gravity)        ← Tier 1
            − m a_base                      ← base translational accel (the BALL_HOLD term)
            − 2 m (ω × ṙ)                   ← Coriolis  (plate spinning while ball moves)
            − m (ω × (ω × r))               ← centrifugal (pushes ball to rim)
            − m (ω̇ × r)                     ← Euler / angular-accel term
            − F_friction(ṙ, N, …)           ← Tier 3
```

These are negligible for slow, pure-tilt motion but grow at the rim and
during fast maneuvers — exactly the regime where the current controller
limit-cycles. `ω`, `ω̇`, and `a_base` are all directly measurable from
the IMU; this is cheap physics to add once the data is logged.
(Lagrangian-in-rotating-frame background:
[Physics LibreTexts](https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Graduate_Classical_Mechanics_(Fowler)/29:_Non-Inertial_Frame_and_Coriolis_Effect/29.01:_The_Lagrangian_in_Accelerating_and_Rotating_Frames).)

### Tier 3 — The friction stack (this is where the real behavior lives)

Friction is the dominant un-modeled effect and the answer to most of your
questions. It has several physically-distinct pieces, each separately
measurable:

| Component | Physics | What it governs | Already known? |
|---|---|---|---|
| **Stiction breakaway** `θ_s` | `tan θ_s = μ_s`. Below it the ball never moves. | The "won't start" dead-zone | ✅ `θ_s≈4°`, `μ_s≈0.07` (foam/vinyl) from STEP_ID |
| **Rate-&-state / dwell aging** | Static friction grows ~**logarithmically with how long the ball has sat still**, and breakaway depends on how fast you ramp the tilt. | *"Rate of change before the ball breaks free"* — your exact question | ❌ never measured |
| **Stribeck effect** | Friction *dips* as velocity rises from zero → negative damping → stick-slip limit cycles | A likely contributor to the orbital limit cycle | ❌ never measured |
| **Rolling resistance** | Opposes rolling; for a sphere on **foam it is velocity-dependent** (sharp rise at low v, flat at mid v, ∝v² at high v) | Coasting deceleration; where the ball settles | ❌ never measured |
| **Viscous** | `∝ v` | High-speed damping | ❌ never measured |
| **Slip onset** | No-slip holds only while required friction `< μ_s N`; fails at high tilt + high speed → ball *slides*, dynamics change | Whether Tier 1 is even valid in a given maneuver | ❌ never measured |

**On "rate of change before break-free":** this is real and
well-studied. Static friction is not a single number — it rises
logarithmically with contact dwell time (rate-and-state friction,
[Dieterich–Ruina](https://academic.oup.com/gji/article/161/1/179/621855)),
and the breakaway force depends on the loading rate. There's a direct
recipe for folding **dwell-time into the LuGre model**
([Inclusion of the dwell time effect in the LuGre friction model](https://www.sciencedirect.com/science/article/abs/pii/S0957415820300258)).
So yes — we should ramp the tilt at several different rates and after
several different rest durations, and measure breakaway as a function of
both. (See campaign B below.)

### The unifying dynamic friction model: LuGre

Rather than bolt on each component separately, the state of the art for
capturing breakaway + Stribeck + presliding + frictional lag in one
model is **LuGre** (a bristle-deflection internal state `z`):

- 4 static params: Coulomb `F_c`, stiction `F_s`, viscous `σ₂`, Stribeck
  velocity `v_s`;
- 2 dynamic params: bristle stiffness `σ₀`, damping `σ₁`;
- identification recipe is standard: **static params from constant-velocity
  sliding experiments; dynamic params from presliding/breakaway experiments**
  ([Wang 2016](https://onlinelibrary.wiley.com/doi/10.1155/2016/6929457),
  [Revisiting the LuGre model](https://hal.science/hal-00394988/document)).

LuGre + the dwell-time extension + a velocity-dependent rolling-resistance
term is the target friction model. It is exactly fittable from the
campaigns in §5.

---

## 3. The modeling approach — gray-box, then hybrid residual

State of the art for robot dynamics is **not** pure black-box. It's a
**hybrid / gray-box** model: an analytical physics core plus a *learned
residual* for whatever the physics misses
([Data-driven Interpretable Hybrid Robot Dynamics](https://arxiv.org/pdf/2512.11900)).
That is the right target here.

| Approach | What it is | Use it for | Caveats |
|---|---|---|---|
| **Gray-box NLS/MLE fit** | Fit the params of Tiers 1–3 (α_eff, ω-coupling, LuGre, rolling-resistance) by nonlinear least-squares / max-likelihood to logged trajectories | **Primary.** Interpretable, few params, generalizes, and STEP_ID already fits a subset | Needs good state estimates (→ offline smoothing) |
| **SINDy** | Sparse regression picks interpretable ODE terms from a candidate library | Discover missing terms / cross-check the gray-box; **weak-form SINDy** tolerates noise | Library must contain the right terms ([SINDy-PI](https://royalsocietypublishing.org/doi/10.1098/rspa.2020.0279)) |
| **Learned residual (NN / GP)** | Model only `f_real − f_graybox` | Mop up leg backlash, IK error, surface inhomogeneity | Data-hungry/opaque; GP gives uncertainty for free ([GP-NODE](https://arxiv.org/pdf/2103.03385)) |
| **Neural ODE** | Learn the whole vector field | Last resort if structured models plateau | Most data-hungry, least interpretable |

**Recommendation:** gray-box first (it reuses STEP_ID and is enough for
the predictor), then a **GP residual** if structured error remains — the
GP's uncertainty bound is directly useful as a "prediction confidence"
the controller can act on. Given vision noise is our bottleneck (§6),
prefer **integral / weak-form** fitting methods, which don't require
differentiating noisy position twice.

---

## 4. Every parameter to capture — the ROS-bag manifest

The discipline: **log raw streams, not just derived/filtered values**, so
state (velocity, acceleration, tilt-rate) can be re-derived offline with
non-causal smoothing. Log the **commanded input** too — closed-loop data
is only identifiable if you recorded what the controller asked for.

### Ball (the thing we're modeling)
| Signal | Topic (verify names) | Why it matters |
|---|---|---|
| Raw detection pixel `(cx, cy)` + confidence | `/oak/ball/diagnostic` | Model the *measurement noise* itself (R), pre-projection |
| Ball position, platform frame `(px, py)` | `/ball_xy_mono` | The core observable; ground state for the fit |
| Ball position, world frame | derive from `(px,py)` + ArUco pose | Needed to separate ball motion from plate motion (coupling terms) |
| KF state `(px,py,vx,vy)` + covariance | `/ball_state`, `/ball_state/cov` | What the controller actually sees; baseline to beat |
| Ball spin (if ever observable) | — | No-slip lets us infer it; only measure if we add markings |

### Platform (the input + the coupling source)
| Signal | Source | Why it matters |
|---|---|---|
| **Commanded** pose `(x,y,z,roll,pitch,yaw)` | controller `/control_cmd` / set_pose | The identifiable input; feedforward reference |
| IMU orientation `(roll,pitch,yaw)` | Xsens MTi-630 | **Actual** tilt — the real driver of `α·g·sinθ` |
| IMU angular velocity `ω` | Xsens | Coriolis/Euler terms; **plate "rate of change"** for breakaway |
| IMU linear acceleration `a_base` | Xsens | Base-motion / BALL_HOLD coupling term |
| Plate angular acceleration `ω̇` | derive from `ω` | Euler term |
| Z height | commanded + FK | Normal force → friction & rolling resistance scale with it |
| 6× motor encoder positions | ODrive / CAN | Independent platform pose via forward kinematics (no IMU drift) |
| 6× motor velocities | ODrive / CAN | Plate velocity ground-truth; backlash/compliance signature |
| 6× motor currents (torque proxy) | ODrive / CAN | Actual force applied; detects saturation, models the *plate's* own dynamics & lag |
| ArUco board pose + per-marker residual | `platform_pose_node` | Vision pose vs IMU vs FK cross-check; occlusion flags |

### Timing (dynamics ID is destroyed by time skew)
| Signal | Source | Why it matters |
|---|---|---|
| Per-message hardware timestamps | all topics | Offline time-alignment; a few ms skew corrupts accel/velocity correlation |
| Vision pipeline latency | `/oak/latency_ms` | The actual `Td` to predict over — **measure, don't assume 0.10 s** |
| Detection arrival vs publish rate | `/oak/health` | Effective sample interval; outlier weighting |
| Control-loop tick time | controller diag | Align command to its effect |

### Session metadata (one sidecar YAML per bag)
Ball type + measured mass/diameter · platform surface material & cleanliness ·
Z setpoint · lighting · camera exposure/gain · vision backend (HSV vs YOLO) ·
ambient temperature (friction is temperature-sensitive) · date/operator ·
git SHA of the running code.

---

## 5. Data-collection campaigns (the experiment battery)

Run these roughly in order; each isolates one piece of the model. Your
two instincts — **place the ball on known ArUco positions and drive it to
a set point** (campaign G) and **vary the platform rate before break-free**
(campaign B) — are both in here, plus the pieces needed for a *dynamic*
(not just static-gain) model.

### A. Measurement-noise characterization — **do this first**
Vision noise is the documented bottleneck (`step_id_tuning_lessons.md`).
You cannot fit dynamics through noise you haven't characterized.
- **Method:** park the ball at rest at a known position (use an ArUco
  marker center as the ground-truth fiducial). Log ≥200 frames. Fit a 2-D
  Gaussian → the KF's `R`. Repeat at center / mid / rim, both ball
  colors, both lighting conditions, and a sweep of exposure/gain.
- **Identifies:** `R(position, lighting, exposure)`, motion-blur-vs-noise
  trade-off, the on-platform detection-dropout rate.

### B. Stiction breakaway — static, rate-dependent, and dwell-dependent
This is your "rate of change before the ball breaks free" campaign.
- **Method:** ball at rest. Ramp tilt at a **fixed rate**; record
  IMU-actual tilt at first motion onset. Sweep the ramp rate
  (≈0.1, 0.25, 0.5, 1, 2, 5 °/s) → breakaway-vs-rate curve. Separately,
  vary **rest dwell time** before the ramp (1, 5, 30, 120 s) →
  breakaway-vs-dwell (aging) curve. Repeat per direction (+x,−x,+y,−y,
  diagonal — checks leg-backlash anisotropy), per position, per Z, per
  ball.
- **Identifies:** `θ_s`, `μ_s`, the rate-and-state aging coefficient, and
  the LuGre dynamic params σ₀/σ₁ (presliding).

### C. Rolling resistance / coasting deceleration
- **Method:** kick the ball to a known speed (brief tilt impulse), then
  snap the plate **dead level** and let it coast. Track the deceleration
  vs speed. Sweep initial speeds (the foam curve has 3 velocity regimes),
  per ball, per surface, per Z.
- **Identifies:** rolling-resistance `c(v)`, the Stribeck dip, viscous σ₂.

### D. Step-response plant ID (extend existing STEP_ID)
- **Method:** step tilt to a fixed angle above breakaway; record the
  trajectory; fit `G_eff` via the Gen-3 regression `v(t)` vs `∫θ dτ`.
  **Extend:** multiple tilt magnitudes (tests `sinθ` linearity + slip
  onset) and multiple Z heights. n≥3–5 replicates for the CV gate.
- **Identifies:** effective `α·g` (= `G_eff`), slip-onset tilt, Z-dependence.

### E. Frequency-domain / persistently-exciting sweeps — **the key upgrade**
Steps barely excite the dynamics; a proper *dynamic* model needs
**persistent excitation**
([MIT 6.435 input design](https://ocw.mit.edu/courses/6-435-system-identification-spring-2005/ffdd1299a755458f8b990bf3f8de8d20_lec4_6_435.pdf)).
- **Method:** drive plate tilt with a **chirp** (sine sweep ≈0.1–3 Hz),
  **multisine**, or **PRBS** — in roll alone, pitch alone, then both —
  keeping the ball on the plate. Run small-amplitude (ball near center,
  linear regime) and large-amplitude (ball explores the rim, nonlinear)
  versions.
- **Identifies:** the full input→ball transfer behavior, frictional lag,
  and validates Tiers 1–3 across frequency. PRBS has the best
  signal-to-excitation (lowest crest factor); chirp is easiest to design.

### F. Plate-rotation / coupling probe (Stewart-specific)
- **Method:** with the ball parked off-center, yaw the plate (and run
  combined roll+pitch oscillations). Record ball drift.
- **Identifies:** whether the Coriolis/centrifugal/Euler terms (Tier 2)
  are significant on this hardware — decides if we keep or drop them.

### G. State-space coverage grid (your "ArUco markers → set point" idea)
The model must be accurate across the **whole** (position × velocity ×
tilt) space, not just near center. The ArUco ring gives exact,
repeatable ground-truth start positions.
- **Method:** place the ball on marker *i*, command a goto to marker *j*
  (or a fixed tilt sequence), record. Sweep many (*i*, *j*) pairs and
  radii. This blankets position space and produces a variety of
  transient trajectories for the fit.
- **Identifies:** position-dependence of all params (surface
  inhomogeneity, IK error vs location); supplies the bulk of the
  training set.

### H. Closed-loop hold-out (validation only — never fit on these)
- **Method:** record ordinary PID + stiction-break Demo-2 runs (goto and
  rolling-trajectory).
- **Use:** the held-out test set. The model wins if it predicts these
  runs better than constant-velocity (§7).

---

## 6. ROS-bag mechanics & best practices

- **Record raw, derive offline.** Log raw pixel, raw IMU, raw encoder.
  Re-derive velocity/acceleration with the **non-causal smoother**
  (`smooth_demo_bag.py`) so fits aren't poisoned by causal-filter lag or
  single-frame vision spikes.
- **Time-sync is non-negotiable.** Camera, IMU, and ODrive streams must
  share a clock or have a characterized offset. A few ms of skew
  decorrelates tilt from acceleration and biases every dynamic param.
  Use `message_filters` approximate-time alignment offline; consider a
  one-time sync-pulse calibration.
- **Replicate for the CV gate.** Vision noise → fit per-condition with
  n≥3–5 and refuse params when `CV = std/mean > 0.30` (existing STEP_ID
  policy).
- **Naming + metadata.** Encode campaign + key params in the bag name
  (you already do this for routines), and drop a sidecar YAML (§4).
- **Train / validate / test discipline.** Campaigns A–G are train/validate;
  campaign H is the locked test set. Never tune on H.
- **Log the input.** For every closed-loop bag, record the commanded
  tilt stream — an input you didn't record is an unidentifiable system.

---

## 7. Offline fitting → model artifact → validation

1. **Ingest** bags → time-align → non-causal smooth → derive
   `(p, v, a, θ, ω, ω̇, N)` per timestep.
2. **Gray-box fit** (NLS/MLE, integral form) of Tiers 1–3 → params.
3. **Residual fit** (GP or weak-form SINDy) on `f_real − f_graybox`.
4. **Export** `config/ball_forward_model.yaml` (analog of
   `step_id_recommendation.json`) — params + fit covariance + valid
   ranges (Z, speed) + the conditions it was fit under.
5. **Validate three ways:**
   - **Horizon error:** predicted position at `Td` vs smoothed-actual, on
     held-out campaign H. Beat constant-velocity — that's the win
     condition, not fit residual.
   - **Open-loop rollout:** integrate the model from an initial state
     through a recorded input sequence; compare full trajectory to the
     recorded ball (sim-vs-real overlay). This is what makes the
     simulation "built on real data."
   - **Multi-step / divergence:** how many ms before predicted and actual
     diverge past a threshold — sets the trustworthy prediction horizon.

---

## 8. From model → simulation → better-than-PID controller

The fitted model feeds three downstream payoffs, in increasing ambition:

1. **Predictor swap** (immediate): replace `x_lead = x + v·Td` in
   `_ball_track_run` with forward-integration of the fitted model over
   the *measured* `Td` and the *commanded-tilt history* (keep a ring
   buffer of issued tilts). Behind a flag, A/B vs constant-velocity.
2. **Model-based feedforward / EKF**: fold the model into `ball_kf_node`
   as an EKF motion model (spec §8) and add an inverse-model feedforward
   so the controller stops relying on the integrator to discover the
   plant.
3. **Sim-based controller development**: stand up the simulation (the
   existing C++ `SimCore` or a Python twin) on the fitted model, validate
   sim-vs-real by input-replay (§7), then design/tune an **MPC** (or a
   learned policy) that plans over the model — the thing that finally
   beats "PID + stiction-break." Develop it in sim, deploy with the same
   flag-gated A/B.

---

## 9. What we already have vs. what to build

| Have | Build |
|---|---|
| `_ball_physics.py` (Tier 1 α, presets) | Tier 2 coupling + Tier 3 LuGre/rolling-resistance terms |
| STEP_ID: `θ_s`, `μ_s`, `G_eff` regression | Rate/dwell breakaway (B), coasting (C), sweeps (E), coverage grid (G) |
| `bag_recorder_node.py`, digest scripts | `fit_ball_forward_model.py`; chirp/PRBS tilt generator |
| `smooth_demo_bag.py` (non-causal smoother) | Time-sync calibration across cam/IMU/ODrive |
| `/oak/latency_ms`, `measure_detector_latency.py` | Per-tick horizon use of measured latency |
| `use_empirical_ik` flag pattern | `use_model_predictor` flag for the A/B |
| C++ `SimCore`, wave-sim GUI | Data-grounded ball-on-platform sim + replay validation |

---

## 10. Open decisions

- **How deep to fit?** Stop at the tier where held-out horizon error
  plateaus. Don't fit Tier 2 coupling if campaign F says it's negligible.
- **Gray-box vs +residual** — start gray-box; add GP residual only if
  structured error remains.
- **Fit at demo Z (30–50 mm) or parameterize on Z?** `G_eff` and friction
  may both scale with normal force (`step_id_tuning_lessons.md` Q6).
- **Tune KF `R` first?** Damping vision spikes at the source may matter
  more than model form (`step_id_tuning_lessons.md` Q2). Likely do A
  before everything.
- **Excitation: chirp vs PRBS vs multisine** for campaign E — start chirp
  (easy), move to PRBS if we need cleaner spectra.
- **One IMU or base+platform pair?** A base-mounted IMU cleanly separates
  `a_base` (coupling) from plate tilt; confirm what `imu_dual` gives us.

---

# Part II — Execution: GUI, run procedure, data sufficiency, robust logging

*Added 2026-06-03 after auditing the live telemetry in the repo.* Part I
says what physics to capture; Part II says how to capture it robustly,
drive it from the GUI, know when to stop, and see the model approach
reality.

## 12. Repo telemetry audit — what we can log vs. what we record today

Every signal below is already published by a running node. The auto-bag
allowlist (`bag_recorder_node.TOPICS_DEFAULT`) records only the ✅ rows —
the ❌ rows are published but **dropped on the floor**, including several
that either drive the model or flag a bad run. This is the concrete
version of "capture everything that could affect the situation."

| Signal | Topic / msg | In bag today? | Why we need it |
|---|---|---|---|
| Ball KF state + cov | `/ball_state`, `/ball_state/cov` | ✅ | The observable + uncertainty |
| Raw detection + per-detector | `/ball_xy_mono`, `/oak/ball/diagnostic` | ✅ | Measurement-noise model (R) |
| IMU (orient, **ω**, **a_base**) | `/imu/data` (`sensor_msgs/Imu`) | ✅ | Tilt + the Tier-2 coupling terms |
| Commanded tilt / mode | `/control_cmd`, `/control_result` | ✅ | The identifiable input |
| Leg encoder positions | `/encoders` | ✅ | FK platform pose (IMU-independent) |
| **Vision latency** | **`/oak/latency_ms`** | ❌ | **The `Td` we predict over — measure, don't assume** |
| **Vision health** | **`/oak/health`** `[v0_arr_hz, v0_pub_hz, jpeg_hz, depth_hz,…]` | ❌ | Detection rate + dropout — the dominant noise source |
| **Controller per-tick** | **`/ball_track/diagnostic`** (FSM phase, commanded tilt, ex/ey, edot, lead) | ❌ | The controller's own view — the input side of the fit |
| **Level/feeder per-tick** | **`/level_diag`** (`LevelDiag`) | ❌ | `feeder_mode`, `axis_state`, per-leg `active_errors`, `leg_iq/vel/enc`, **`target_xyzrpy` incl. Z**, `dt_actual` |
| **Motor currents** | **`/leg_currents`** | ❌ | Applied-torque proxy; saturation / backlash signature |
| **Per-motor full state** | **`RobotState`/`MotorStateSingle`** (`active_errors`, `disarm_reason`, `current_state`, `procedure_result`, `iq_meas/sp`, `fet_temp`, `motor_temp`, `bus_voltage`, `bus_current`) | ❌ | Catches a leg silently in IDLE, **armed in the wrong mode**, thermal/voltage fault, watchdog disarm |
| **CAN utilization** | **`CanTrafficReportMessage`** (`received_count`, `report_interval`) | ❌ | Bus saturation → dropped frames → silent control gaps |

**On Z and motor mode specifically** (your callout): Z is logged inside
`LevelDiag.target_xyzrpy`, and the "is a leg accidentally in vel instead
of pos / not closed-loop" case is exactly `LevelDiag.feeder_mode` +
`axis_state` + `active_errors`. The data already exists — it just isn't
in the bag, and `LevelDiag` is only published while the *level* loop runs
(it's bypassed during BALL_TRACK), so BALL_TRACK runs currently capture
**none** of the per-leg health. Both are fixed in §13.

## 13. Robust logging — manifest v2

Two code changes, both small:

1. **Expand the allowlist** to add: `/oak/latency_ms`, `/oak/health`,
   `/ball_track/diagnostic`, `/level_diag`, `/leg_currents`, the
   per-motor `RobotState`, and the CAN-traffic topic. (Images stay
   opt-in.) Mirror them for the new data-collect modes (§15).
2. **Publish per-leg health during BALL_TRACK too.** Either keep
   `/level_diag` (or a slim `motor_health` topic) flowing while the
   BALL_TRACK loop owns the bus, so every run — not just level runs —
   records `axis_state`/`feeder_mode`/`active_errors`/temps.

Plus the discipline from §6: every message carries a usable timestamp;
add a one-time sync-pulse to align camera/IMU/ODrive clocks; **log raw,
derive offline**; write the per-bag metadata sidecar (§4).

## 14. Auto-QA — reject bad runs before they poison the fit

Once §13 lands, the digest can automatically mark a bag unusable and drop
it from the training set — your "identify an issue" ask, for free:

- any `axis_state ≠ 8 (CLOSED_LOOP)` mid-run → a leg dropped out;
- any `feeder_mode` unexpected (vel where pos was intended) → armed wrong;
- any `active_errors ≠ 0` (watchdog / vel-limit / current-limit) → fault;
- `motor_temp` / `fet_temp` over threshold or `bus_voltage` sag → thermal/power;
- `/oak/health.v0_pub_hz` below floor or a `/oak/latency_ms` spike → vision starvation;
- `CanTrafficReport.received_count` drop → bus saturation.

Faulty bags get the existing `_fault` suffix and the fitter skips them.
A clean run is one where none of these fired for its whole duration.

## 15. GUI integration — mirror the STEP_ID pattern

The GUI is an HTTP server (`gui_server.py`, `do_GET`/`do_POST`) +
`web/index.html`, with the controller driven over the `/control_cmd`
String bus. STEP_ID is already wired exactly this way — session
enumeration (`/step_id/bags`), digest (`/step_id/bags/digest`), plot
serving (`/step_id/bags/png`), push (`/step_id/bags/push`), per-phase Run
buttons. Clone that pattern for a **Data Collection panel**:

- **Per-campaign Run buttons** (Meas-noise, Breakaway, Coasting, Step-ID*,
  Sweep, Coupling, Coverage-grid) → publish
  `data_collect:<campaign>:<json-params>` on `/control_cmd`.
  (*Step-ID already exists — fold it in rather than duplicate.)
- **Live coverage map** — heatmap of state-space bins (r × |v| × θ × θ̇ ×
  Z × direction) with per-bin counts, so the operator fills the *next
  under-sampled* bin instead of collecting blindly. This is what makes
  the campaign systematic rather than a pile of random runs.
- **Run-QA strip** — `axis_state`/`feeder_mode`/`active_errors` lights +
  latency + detection-rate readouts; auto-abort and `_fault`-tag on
  disarm / dropout / latency spike (§14, live).
- **Session list** with digest / PNG / push — direct clones of the
  step_id endpoints: `/data_collect/bags`, `/.../digest`, `/.../png`,
  `/.../push`.
- **Model panel** — after a fit, shows the §18 error visualizations so
  you literally watch the sim get closer to reality between runs.

## 16. Operator runbook — the steps of one collection run

1. **Pre-flight** (auto-checked, shown in the QA strip): homed + levelled;
   all six axes `CLOSED_LOOP` in the intended `feeder_mode`; temps/bus
   nominal; vision backend + exposure chosen; ball type/mass/surface, Z,
   lighting, ambient temp written to the metadata sidecar; running git SHA
   logged.
2. **Pick campaign + params.** The coverage map highlights the next
   under-filled bin and (via §17.2) can recommend the most informative run.
3. **System opens the expanded-topic bag** automatically
   (`data_collect_<UTC>_<campaign>/`) and writes the sidecar.
4. **Place the ball** as prompted (ArUco markers as fiducials), or let the
   auto motion run (sweeps / steps / coasting).
5. **Execute** the campaign motion. QA strip live; auto-abort + fault-tag
   on disarm / dropout / latency spike.
6. **Stop** → bag flushed → auto-digest → coverage map, parameter
   convergence, and learning curve update.
7. **Repeat** until the §17 stopping criteria go green for every region.
8. Periodically run a held-out **click-to-goto** set (campaign H) —
   never fit on it — and watch the model panel close on reality.

## 17. "When do we have enough data?" — the stopping criteria

There is a principled answer, and it is **per-region, not global** — you
can be saturated at the center and starved at the rim at the same time.
Four tests; a region is *done* only when all four are green there.

1. **State-space coverage quota** — *answers center-vs-edge, fast-vs-slow,
   small-vs-large, quick-vs-slow-platform, start→end.* Bin the reachable
   space on the axes that matter: radius `r` (center→rim), speed `|v|`,
   tilt `θ`, **tilt-rate `θ̇`** (slow vs quick platform moves), `Z`, and
   the start→end direction pairing. Require ≥ `N` *post-QA* samples per
   reachable bin (e.g. `N ≈ 300`). The coverage map turns "is every case
   covered?" into a visual checklist; empty reachable bins *are* the
   to-do list.
2. **Per-parameter convergence via Fisher information** — *the rigorous
   bound.* The inverse Fisher Information Matrix lower-bounds the
   parameter covariance (Cramér–Rao). Track each fitted parameter's
   running estimate ± its CRLB standard deviation as bags accumulate; a
   parameter is "done" when its relative std drops below target
   (~10–15 %) **and** the estimate has stopped drifting. This generalizes
   your existing `CV < 0.30` gate to *every* model parameter, and
   **D-optimal design** (maximize `det(FIM)`) tells you which campaign
   adds the most remaining information — the GUI can recommend the next
   run ([optimal experiment design / FIM](https://link.springer.com/10.1007/978-1-4419-9863-7_1222),
   [D-optimal for nonlinear dynamics](https://www.hindawi.com/journals/mpe/2012/296701/)).
3. **Held-out learning-curve plateau** — *the model-agnostic answer.* Plot
   held-out horizon-error vs training-set size; when more bags stop
   lowering it, more data won't help
   ([learning-curve diagnosis](https://scikit-learn.org/stable/auto_examples/model_selection/plot_learning_curve.html)).
   If it's still falling, keep collecting; if it plateaus *high*, the fix
   is more physics, not more data → test 4.
4. **Residual whiteness** — *are we missing data or missing physics?* After
   fitting, check the prediction residuals: if they're white (no
   autocorrelation, uncorrelated with `r, v, θ, θ̇, Z`), the remaining
   error is just noise → done. If they're structured (error grows with
   speed, or near the rim), that region needs **more data** if its bin is
   sparse, or a **missing model term** if the bin is dense but still
   wrong. The §18 residual maps localize exactly which.

Worked example: rolling-resistance-at-high-speed is "enough" only when
the high-`|v|` bins are filled (test 1), its CRLB std is small (test 2),
the learning curve has flattened (test 3), and the high-`v` residuals are
white (test 4). All four, in that region.

> Why bother capturing this much: a recent ball-balancing study found
> RL's edge over PID came "not from improved parameter tuning... but from
> its ability to effectively utilize richer state observations"
> ([Rich State Observations Empower RL to Surpass PID](https://arxiv.org/abs/2509.21122)).
> Richer logging is the lever; the stopping criteria keep it from running
> forever.

## 18. Digest error visualizations — watching the sim approach reality

Every fit/digest emits (extending, not replacing, the existing STEP_ID
plots like `plant_gain_fit.png` / `phase_plane.png`):

- **Sim-vs-real overlay** — model rollout vs recorded ball on the XY plane
  + `px(t), py(t)` time series, per held-out bag. The headline picture.
- **Horizon-error curve** — prediction error vs lookahead (0–200 ms),
  model vs the constant-velocity baseline. Shows the win and the
  trustworthy horizon.
- **Residual heatmaps** — error binned over position, speed, tilt, and
  tilt-rate. Shows *where* we're wrong.
- **Residual whiteness** — autocorrelation + residual-vs-state scatter
  (the missing-physics detector).
- **Coverage map** — occupancy + per-bin counts over the §17 bins.
- **Parameter convergence** — each parameter ± CRLB std vs #bags.
- **Learning curve** — held-out horizon-error vs data quantity.
- **Per-bag QA timeline** — `axis_state`/`feeder_mode`/`active_errors`,
  latency, detection-rate, dropout markers, and the auto-exclude verdict.

These are what you watch, run over run, to confirm the work is actually
getting closer to reality — and to prove it for the click-to-goto demo.

---

## 19. Reality check (2026-06-03): allowlist safety + measured latency

Two questions answered before touching code.

### Expanding the bag allowlist is safe — verified

- **Two record paths already exist.** `bag_recorder_node.TOPICS_DEFAULT`
  (auto, on mode-transition, *narrow*) and `gui_server.DEMO_TOPICS`
  (manual `/demo/start`, *broad*). The Demo-2 tuning bags were recorded
  with `DEMO_TOPICS`, which **already** includes `/oak/latency_ms`,
  `/oak/health`, `/oak/config`, `/ball_track/diagnostic`,
  `/leg_currents`, `/leg_encoders`, `/odrive_errors`, `/status`. So
  expanding the auto-list is *not new behavior* — it aligns the
  auto-recorder with the manual recorder already in daily use.
- **Every bag consumer is topic-filtered.** `digest_demo_bag`,
  `digest_vision_bag`, `digest_iva_bag`, and `smooth_demo_bag` build a
  `{topic: type}` map and do `cls = _TYPE_CLASS.get(type); if cls is
  None: continue` — unknown/extra topics are silently skipped.
  `analyze_level_bag` reads only `LevelDiag`; `compare_demo_bags` reads
  digest JSON, not bags. **Adding topics cannot break any of them**, and
  `digest_demo_bag.py` already has handlers for the latency/health/diag/
  currents topics — they're coded but the *auto*-recorder starves them.
- **Nothing pins the list.** `TOPICS_DEFAULT` is referenced only inside
  `bag_recorder_node.py`; no test asserts it (the four tests cover
  aruco / ball-physics / marker-layout / safety-yaml).
- **Net:** additive and safe. The only genuinely new topics (`/level_diag`,
  per-motor `RobotState`, CAN traffic) are skipped by existing digests
  until one opts in — so just (a) flow `/level_diag` during BALL_TRACK
  (§13), and (b) confirm `RobotState`/CAN are actually published before
  relying on them (`ros2 bag record` only *warns* if a listed topic is
  absent, never errors). Consideration, not breakage: bag size / Pi load
  grows modestly — still no images, and `DEMO_TOPICS` already records at
  these rates without issue.

### Measured latency (Demo-2 run `20260502T031335Z`)

| Metric | Value |
|---|---|
| `/oak/latency_ms` (capture→Pi, **cv2** path) | mean **108 ms**, std **29 ms**, p95 **162 ms**, max 186 ms |
| on-device NN path (`v0_lat_ms`) | mean **56 ms**, p95 75 ms |
| detection arrival → publish | **27 Hz → 12.6 Hz** (publish-capped; ~79 ms between ball updates) |
| controller assumption (`control_latency_s`) | **0.2 s** (operator doubled it to fight the lag) |
| platform IMU rate | ~283 Hz (near-zero-latency *input* signal) |
| exposure / ISO | 8 ms / 1600 → ~6 mm motion blur at 800 mm/s |

### Is that latency reasonable for what we're planning?

- **Offline modeling — yes, with discipline.** 108 ms is fine to fit
  *through* provided every sample carries its capture timestamp and we
  time-align offline. The 29 ms jitter means a *fixed*-delay assumption
  would smear the input↔output correlation, so the fit must use the
  per-sample `/oak/latency_ms` (already in the plan, §4/§13).
- **Online control — no, not with the current predictor.** 108 ms ± 29 ms
  (tail to 186) is large for 0.5 mm-class control of an 800 mm/s ball
  (108 ms ⇒ up to 86 mm of travel), and a *fixed* `control_latency_s`
  cannot track the varying real delay — the jitter alone injects tens of
  mm of prediction error, enough to sustain the orbital limit cycle. It
  is only tolerable with a **model-based** predictor.

### Latency mitigations (priority order)

1. **Use the per-sample latency you already log.** Feed live
   `/oak/latency_ms` (+ capture timestamp) into the predictor instead of
   the fixed `control_latency_s`. Kills the ±29 ms jitter penalty;
   near-zero cost; also a modeling prerequisite.
2. **Model-based dead-time compensation (Smith predictor / known-input
   EKF).** The tilt is known at ~283 Hz, ~0 latency — forward-integrate
   the ball through the *known* tilt over the dead time with the fitted
   model. This *is* the modeling work; it's what makes 100 ms survivable.
   Smith predictors are the textbook tool for camera-delay visual
   tracking, and their benefit is bounded by model fidelity — which is
   exactly why the empirical model matters
   ([Smith predictor for CCD/optical tracking](https://www.mdpi.com/1424-8220/24/17/5546)).
3. **Close the 27 → 12.6 Hz publish gap** → roughly halve inter-sample
   staleness (the repo's own note: would "halve the effective vision
   latency").
4. **Cut motion blur** (exposure 8 ms → 2–3 ms + more light / IR) so the
   fast on-device YOLO path (56 ms) is reliable under motion and can
   replace the slow cv2 path (108 ms). One lever that both ~halves
   latency and fixes the detection-accuracy failure (`step_id_tuning_lessons.md` §3).
5. **Faster control tick** (50 → 100 Hz) to shave the control-stage delay.
6. **Tracking-gate ROI** around the model-predicted position → faster
   inference + fewer false positives.

**Headline:** latency is high and jittery, but the cheapest win — stop
assuming a fixed 0.2 s and use the per-sample latency already in the bag —
plus the model-based predictor (the planned work) is the path to a crisp,
real-to-life click-to-goto.

---

## 20. Throughput levers — publish gap, loop rates, CAN (2026-06-04)

Investigation of two proposed improvements: close the 27 → 12.6 Hz
vision publish gap, and raise the control tick to 100 Hz. Headline: the
bottlenecks are not where they first appear, and most of the machinery to
change rates safely already exists.

### 20.1 The publish gap (27 → 12.6 Hz) is two bottlenecks, not a throttle

Mechanism in `oak_driver_node`:
- The slow tick `_tick` runs at **60 Hz**, but `q_rgb_raw.tryGet()` only
  yields ~27 frames/s: the full-res RGB-raw stream (~1.5 MB/frame) is
  **USB-bandwidth-limited**, not tick-limited → `v0_arr ≈ 27 Hz`.
- `_n_v0` (→ `v0_pub`) increments **only on a successful detection**. The
  cv2 HSV detector misses the ball under motion blur, so ~27 attempts →
  ~12.6 successes. The code comment is explicit: *"v0_pub crash from 13 Hz
  → 5 Hz once the ball started moving."* → `v0_pub ≈ 12.6 Hz`.

So "closing the gap" is **not** removing a rate cap. It means:
1. **Kill the USB-raw bottleneck** by detecting on-device — the VPU/YOLO
   path sends only `(cx, cy)` over USB and hits 37–60 Hz
   (`oak_throughput_diagnosis.md`).
2. **Fix the detection misses** — reduce motion blur (exposure 8 ms →
   2–3 ms + light/IR), a better-trained on-device model, and a
   tracking-gate ROI. The blocker today is on-device detector robustness
   under fast motion (cv2 is robust but USB-capped; YOLO is fast but loses
   the ball — the `step_id_tuning_lessons.md` §3 tradeoff).

> **Deeper:** the root cause is that we detect on the rolling-shutter RGB
> with an 8 ms exposure while the global-shutter mono cameras (built for
> ping-pong-ball tracking) sit disabled. Full sensor-level analysis,
> what's been tried, and the high-speed-vision recipe (global shutter +
> sub-ms exposure + IR) are in
> [`oak_highspeed_detection_analysis.md`](oak_highspeed_detection_analysis.md).

Downstream of a higher ball-publish rate — all additive, no breakage:

| Consumer | Effect |
|---|---|
| `ball_localizer` / `ball_kf` | Subscription-based, rate-agnostic; more updates → tighter KF. **Re-tune KF `R`/`Q`** for the new measurement interval — the only required change. |
| BALL_TRACK controller | Fresher anchors for the predictor; strictly better. |
| digests / bags | More samples per topic; larger bags (still no images). Topic-filtered readers unaffected. |

### 20.2 "Control tick" is three distinct clocks — the outer loop is already 200 Hz

| Clock | Where | Current rate | Configurable? | On the CAN bus? |
|---|---|---|---|---|
| **Outer control loop** (level + BALL_TRACK) | `ctrl_period_s = 1/level_loop_hz` | **200 Hz** (default) | Yes — `--level-loop-hz` | No |
| **ODrive feeder** (`Set_Input_Pos/Vel`) | `ODriveFeeder.period` | **50 Hz** (hardcoded) | No (period not plumbed through) | **Yes** |
| **Encoder feedback** (RTR) | `EncoderListener.rtr_period` | **10 Hz** enc / 2 Hz err | Yes (constructor arg) | Yes (light) |

The *outer* loop already runs at **200 Hz**, with gains calibrated at
`LEVEL_REF_HZ = 50` and **auto-scaled** (`rate_limit`, `alpha`,
`integ_decay` via `rate_scale = dt/ref_dt`). So "raise the control loop to
100 Hz" is already satisfied — and the rate-change machinery is proven.
The meaningful levers are the **feeder** (what actually commands the
motors) and the **encoder RTR** (what the loop senses).

### 20.3 Raising the feeder 50 → 100 Hz — safe, one plumbing change

- **Plumb a period/rate into `ODriveFeeder`** — line 1367 constructs it
  without a `period`, so it uses the hardcoded `0.02`. Make it a
  constructor/CLI arg.
- **CAN TX doubles**: feeder ≈ 6 legs × 50 Hz × ~116 bits ≈ 35 kbps →
  ~70 kbps at 100 Hz. The bus is **1 Mbps**, and the design deliberately
  uses RTR (10 Hz/2 Hz) instead of cyclic ODrive broadcast to keep it
  light — utilization is order ~5–10 %. **Ample headroom; 100 Hz is
  safe.** Monitor `can_bus_utilization_pct` (already computed in
  `_compute_can_rates`) and bag it (§13).
- **Watchdog**: the 500 ms ODrive watchdog is fed every 20 ms (50 Hz) →
  25× margin; at 100 Hz, 50×. Non-issue.
- **`_feeder_safety_check` runs every feeder tick** ("called every 20 ms
  (50 Hz)", line 1912) → 2× its CPU at 100 Hz; it's cheap bounds-checks,
  but confirm.
- **Docs/comments** saying "50 Hz cyclic" need updating
  (`level_loop_architecture.md`, `ARCHITECTURE.md`, feeder docstrings).
- **Benefit**: the 200 Hz outer loop is currently down-sampled to 50 Hz
  at the feeder; at 100 Hz, twice as much of the loop's resolution reaches
  the motors and the zero-order-hold lag halves (~10 ms → ~5 ms) — a
  direct, free cut to the §19 latency budget.

### 20.4 Raising the BALL_TRACK loop rate — one hidden gain-scaling gap

The outer loop is already 200 Hz, but BALL_TRACK's **tick-based** params
are NOT all rate-scaled the way the level loop's are:
- `stiction_moving_hyst_ticks` (3): comment says "At 50 Hz, 3 ticks =
  60 ms" — but at the 200 Hz default it's already only **15 ms**. This is
  a latent mismatch *today*, and any rate change shifts it further.
  **Convert to a time (ms) or scale by rate.**
- Confirm `tilt_slew_up_deg_per_s` is applied as `slew × period` (it reads
  as °/s, so verify the multiply isn't per-tick).
- The integrator (`integ + ex·period`) and Kd LPF (`alpha =
  period/(tau+period)`) are already period-correct.

This is the **main correctness change** for any BALL_TRACK rate change —
otherwise the stiction-break timing drifts silently.

### 20.5 Bonus finding: leg encoder/current telemetry is only 10 Hz-fresh

`/leg_encoders` and `/leg_currents` are *published* at 20 Hz
(`_tick_state`), but the underlying data is RTR-refreshed at only **10 Hz**
(`EncoderListener`, 10 Hz/2 Hz) — so the bag holds 20 Hz of 10 Hz-fresh
data (aliased). For the ball-physics fit, which wants leg current
(applied-torque proxy) and FK pose, **10 Hz is coarse**. Raising
`rtr_period` (e.g. 10 → 50 Hz) costs a little CAN (headroom exists) and
materially sharpens the motor-side telemetry. Worth doing alongside §13.

### Summary — changes & blast radius

| Change | Files touched | Downstream risk |
|---|---|---|
| On-device detection robustness (close publish gap) | `oak_driver_node`, blob/training, exposure | Low — re-tune KF `R`/`Q` |
| Feeder 50 → 100 Hz | `stewart_control_node` (plumb period) + docs | Low — CAN headroom ample, watchdog fine; update "50 Hz" comments |
| BALL_TRACK tick-param rate-scaling | `stewart_control_node._ball_track_run` | **Correctness** — convert `stiction_moving_hyst_ticks` or timing shifts silently |
| Encoder RTR 10 → 50 Hz | `stewart_control_node.EncoderListener` | Low — CAN headroom; better telemetry |

**None of these touch the bag-consumer scripts**; all are additive or
rely on the existing rate-scaling machinery. The two genuinely free wins:
feeder → 100 Hz (halves actuation ZOH lag) and encoder RTR → 50 Hz
(sharper modeling telemetry); the one that needs care is the BALL_TRACK
`stiction_moving_hyst_ticks` rescale (already subtly wrong at 200 Hz).

---

## 11. Sources

Ball-and-plate dynamics & validation:
- [Mathematical modelling of ball and plate system with experimental and correlation-function-based validation (Springer, 2024)](https://link.springer.com/article/10.1007/s11768-024-00208-8)
- [Nonlinear model development and validation for ball and plate (Cal Poly thesis)](https://digitalcommons.calpoly.edu/theses/)
- [Modelling and control of ball-plate system (project report)](https://danielkhashabi.com/files/2011_LinearControl/16.pdf)
- [Lagrangian in accelerating/rotating frames (Physics LibreTexts)](https://phys.libretexts.org/Bookshelves/Classical_Mechanics/Graduate_Classical_Mechanics_(Fowler)/29:_Non-Inertial_Frame_and_Coriolis_Effect/29.01:_The_Lagrangian_in_Accelerating_and_Rotating_Frames)

Friction modeling & identification:
- [Revisiting the LuGre friction model (HAL)](https://hal.science/hal-00394988/document)
- [Dynamic friction parameter identification with LuGre (Wang 2016, Wiley)](https://onlinelibrary.wiley.com/doi/10.1155/2016/6929457)
- [Inclusion of the dwell-time effect in the LuGre model (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0957415820300258)
- [Rate-and-state friction (Dieterich–Ruina; Oxford GJI)](https://academic.oup.com/gji/article/161/1/179/621855)
- [Coefficient of rolling friction — lab experiment (AJP)](https://pubs.aip.org/aapt/ajp/article/86/1/77/1045848/Coefficient-of-rolling-friction-Lab-experiment)
- [Why low-bounce balls exhibit high rolling resistance (velocity-dependent, foam)](https://www.researchgate.net/publication/283275557_Why_low_bounce_balls_exhibit_high_rolling_resistance)

Data-driven / hybrid system identification:
- [Data-driven interpretable hybrid robot dynamics (physics + learned residual)](https://arxiv.org/pdf/2512.11900)
- [SINDy-PI: robust sparse identification of nonlinear dynamics (Royal Society)](https://royalsocietypublishing.org/doi/10.1098/rspa.2020.0279)
- [GP meets Neural ODE: Bayesian dynamics from scarce, noisy data](https://arxiv.org/pdf/2103.03385)

Excitation / experiment design:
- [MIT 6.435 — input design & persistence of excitation](https://ocw.mit.edu/courses/6-435-system-identification-spring-2005/ffdd1299a755458f8b990bf3f8de8d20_lec4_6_435.pdf)
- [Excitation signals for identification of dynamic systems (PRBS/multisine/chirp)](https://sciengineer.com/excitation-signals-for-identification-of-dynamic-systems/)

Data sufficiency / optimal experiment design (Part II §17):
- [Optimal experiment design & Fisher information (Springer ref)](https://link.springer.com/10.1007/978-1-4419-9863-7_1222)
- [D-optimal design for parameter estimation in nonlinear dynamic systems](https://www.hindawi.com/journals/mpe/2012/296701/)
- [Optimal experimental design under observation noise (arXiv)](https://arxiv.org/pdf/2504.19233)
- [Learning curves for model selection / data sufficiency (scikit-learn)](https://scikit-learn.org/stable/auto_examples/model_selection/plot_learning_curve.html)

Ball-balancing control & the value of rich observations (Part II §17):
- [Rich state observations empower RL to surpass PID — drone ball balancing](https://arxiv.org/abs/2509.21122)
- [Learning a ball-balancing robot through deep RL (compound controller, contacts hard to model)](https://arxiv.org/abs/2208.10142)
- [Model predictive control of a ball-and-plate model](https://www.researchgate.net/publication/301407898_Model_Predictive_Control_of_a_Ball_and_Plate_laboratory_model)

Latency / dead-time compensation (Part II §19):
- [Smith predictor modified with pseudo-feedforward for CCD optical tracking (MDPI Sensors)](https://www.mdpi.com/1424-8220/24/17/5546)
- [Discrete-time Smith dead-time compensator (MathWorks ref)](https://www.mathworks.com/help/sps/ref/smithpredictorcontroller.html)
