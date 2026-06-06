# Latency + headroom journey — slaying the Pi-saturation dragon (2026-06)

**What this is.** A retrospective of the multi-day effort that took the
closed-loop see→move latency from **~450 ms to ~264 ms**, the Pi from
**100 % CPU / load 22** to **~96 % / load 8.7**, and — the real prize —
made the **latency decoupled from host load** (cpu↔latency correlation
+0.47 → −0.20). Written so we don't re-derive the findings, and so the
*next* person understands why the system is shaped the way it is.

## TL;DR — the numbers

| metric | start | end | what moved it |
|---|---|---|---|
| see→move loop latency | ~450 ms | **~264 ms** | almost entirely the 2 t/s actuation cut |
| actuation (cmd→motion) | ~285 ms | **105 ms** | leg speed cap 1.0 → 2.0 turns/s |
| OAK capture→Pi | 95 ms | **54 ms** | YOLO offload (no host cv2 frame-drain) |
| see-pipeline p50 (photon→state) | ~158 ms | ~159 ms | **unchanged** — inherent detect→state + staleness |
| load (1-min avg) | 22–26 | **8.7** | the whole waste-cutting campaign |
| cpu↔latency correlation | +0.47 (coupled) | **−0.20 (decoupled)** | freed headroom |
| tracking error rms | ~88 mm | 91 mm | unchanged (no capability lost) |

**Two structural wins did the heavy lifting; a pile of small "compute
nobody consumes" fixes freed the headroom that made them measurable.**

## The problem

Demo/bench digests showed the Pi 5 **pegged at 100 % CPU, load ~26 on 4
cores**, with the firmware `throttled` flag set at only 58 °C. Every
latency number was contaminated (the detect→state split swung run to run;
the bench gain read garbage from a starved excitation thread). We couldn't
tell if the platform was slow because of *power*, *compute*, or *the loop
itself*.

## Methodology — measure, don't guess

The single most important discipline: **profile before optimising.** Every
time we guessed the hot spot, we were wrong.

1. **Power vs compute first.** `vcgencmd get_throttled` / `measure_clock
   arm` separated under-voltage (the flag was the `0x1` under-volt bit, not
   thermal) from genuine compute. A proper 5 V/5 A PSU cleared the
   under-voltage; load 21 *remained* → compute-bound, confirmed.
2. **`scripts/profile_system.py`** (built for this): per-node %CPU/RAM/
   threads over a window + system aggregate + an automated **py-spy**
   own-time drill-down on the hottest Python nodes. This is what found
   every real hog. The bet was always wrong before py-spy: we predicted
   `getCvFrame`; the actual #1 was a discarded depth map.
3. **The cpu↔latency correlation digest** (`host_latency_correlation` in
   `_latency.py`): both timelines are co-recorded in every demo bag
   (`system_stats.jsonl` cpu/load @1 Hz + `/ball_state.orientation.z`
   per-frame photon→state age), so the Pearson r is the "did headroom help
   latency?" metric. Watching it flip +0.47 → −0.20 is how we *knew* the
   CPU was no longer the latency bottleneck.

## The recurring anti-pattern: compute nobody consumes

Most of the headroom came from finding work the Pi did every frame whose
result was thrown away:

| fix | what it was | reclaimed |
|---|---|---|
| `oak_driver` depth-map gate | `_expected_plane_depth_map` (540×960 meshgrid + per-pixel plane solve) rebuilt on every ArUco pose, but consumed **only** by the depth-blob detector — which is off by default (`OAK_ENABLE_DEPTH=0`). #1 hog, ~31 % of the busiest node. | ~0.3 core |
| `calibration_node` gate | full ArUco `detectMarkers` on **every** RGB frame forever — duplicating `platform_pose`'s identical detection — just in case the operator calibrates. Gated to an active session (GUI panel open). | ~0.35 core |
| `auto_tune_node` disabled | ~13 subs churning the executor for a node the operator no longer uses. `STABLE_BOT_AUTOTUNE=1` to restore. | ~0.23 core |
| legacy `bag_recorder_node` disabled | spawned its **own** `ros2 bag record` to `~/stable_bot_bags/` on every demo — fully redundant with the GUI auto-bag (→`tuning_data/`, the digested one). The "two `ros2 bag record`" mystery. `STABLE_BOT_LEGACY_BAGREC=1` to restore. | ~0.7 core |
| `_save_persisted_pose` throttle | wrote the commanded pose to the SD card **every control tick** (via `_do_set_pose`) for restart recovery. 1 Hz is plenty. | ~0.34 core |
| GUI live-video off by default + auto-drop on record | rosbridge serialising JPEG to the browser (`__repr__(CompressedImage)` was 22 % of the node) competes with the see→move path. | ~0.3 core when off |
| Vision Debug panel-collapse gating | the 4 per-frame detection-overlay streams unsubscribe when the panel is collapsed (they feed only `renderBallOverlays`). | rosbridge fan-out |

**Lesson:** on a *pegged* CPU, removing waste drops the **load** (run-queue
depth) but CPU stays ~100 % — the starved nodes immediately absorb the
freed cycles. So `load` (not `CPU %`) was the honest progress metric until
the very end. Watch the load trend: 22 → 17 → 14 → 11 → 8.7.

## The two structural wins

### 1. Vision offload to the OAK VPU (YOLO backend)
Switching the detector from host **cv2 HSV** to **on-device YOLO**
(`OAK_V0_BACKEND=v1_yolo`) deleted the host `detect_ball_v0` *and* the
60 Hz `rgb_raw` `getCvFrame` copy. Result: `oak_driver` −13 %, **capture→Pi
95 → 54 ms**, and tracking *held* (rms 76 vs cv2's 88 — YOLO sees the ball
fine). Its biggest contribution was **headroom + fresh-frame latency**, not
the see-*median* (which is staleness-dominated — see below).

### 2. Leg speed cap 1.0 → 2.0 turns/s — the actuation lever
This is what actually cut the *loop* latency. The control node commands the
ODrive `vel_limit = soft_cap × 1.5` at runtime, so doubling the soft cap let
the legs reach commanded tilt faster: **actuation 250–285 ms → 105 ms.**
That single change is ~all of the 450 → 264 ms loop reduction. Cost:
~2× peak leg current → re-confirm power before running at 2 t/s.

## Where the remaining latency lives

The **see-pipeline p50 (~159 ms) did not move** across the whole campaign —
it's dominated by **detect→state** (localizer + KF tick + the 27→12.6 Hz
inter-detection staleness, `ball_physics_modeling_plan.md` §20), not by
transport. So:
- The **actuation** half is now small (105 ms) and bounded by motor speed.
- The **see** half (159 ms) is the inherent pipeline — the next frontier if
  more latency is wanted, and it's *not* CPU-bound (the −0.20 correlation
  proves it).

## Is 264 ms "good"? — read this with §19 of the physics plan

Naively, 264 ms is high for 0.5 mm-class control of a fast ball. **But the
controller predicts the ball `Td` ahead (`x̂ = x + v·Td`)**, so latency is a
*prediction horizon*, not a raw penalty. The physics-modeling plan's whole
premise is a model-based predictor (Smith-predictor / known-input EKF that
forward-integrates the ball through the **known** ~283 Hz tilt over the dead
time). The plan's §19 verdict already said 108 ms is "only tolerable with a
model-based predictor" — at 264 ms that's more true, **and that predictor is
the planned work.** The 2 t/s win matters doubly here: it shortened the dead
time the predictor must cover by ~145 ms, making the model's job easier.

**The right next question is not "cut more latency" but "does the model
predict ~264 ms ahead within tolerance?"** — i.e. the plan's horizon-error
curve (§17), extended to 264 ms. Measure that before cutting further. Also
prefer feeding the **per-sample** `/oak/latency_ms` to the predictor over a
fixed `control_latency_s` (plan §19 mitigation #1) — the jitter, not the
mean, was the limit-cycle culprit.

## Tooling left behind

- **`scripts/profile_system.py`** — per-node CPU + py-spy drill-down, `--compare` deltas, `--push`. The workhorse.
- **`_latency.host_latency_correlation` + `read_system_stats_series`** — the cpu↔latency metric, surfaced in `digest_demo_bag.py` (`summary.host_latency_correlation` + a scatter panel). Re-runs on every demo digest.
- **Env gates** for re-enabling the disabled-by-default nodes: `STABLE_BOT_AUTOTUNE`, `STABLE_BOT_LEGACY_BAGREC`.

## One scare worth recording

Arming the platform once browned out the Pi (SSH + GUI + vision all died at
once, clock dipped to 1000 MHz at 58 °C). It was **power, not code** — the
PSU/cable is marginal under the motor-arming current spike. A power-cycle
recovered it; an ODrive re-home cleared the resulting calibration garbage
(which had shown up as a one-off rms 210). If you raise leg speed/current,
keep the power margin honest.
