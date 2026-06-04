# Ball Physics Modeling & Data-Collection Plan

**Added 2026-06-03.** Research-grounded plan for replacing the
constant-velocity "look-forward" (and ultimately the PID + stiction-break
loop) with a **data-driven physics model** of the ball + platform,
fit from ROS bags of the real hardware. This is the deep-dive companion
to [NEXT_STEPS.md → Phase 2](NEXT_STEPS.md#phase-2--data-driven-ball--platform-forward-model-look-forward-upgrade).

The goal: a simulation accurate enough that (a) the controller's
forward-prediction is right on transients, and (b) we can develop and
tune a better-than-PID controller in sim before touching hardware.

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
