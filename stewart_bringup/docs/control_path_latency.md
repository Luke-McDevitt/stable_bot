# Control-path latency map: detection → platform moved

**Added 2026-06-04.** Traces every latency source from a photon hitting
the sensor to the platform actually tilting toward the target, **grounded
in the code** (file:line), with **where each value comes from** so nobody
has to guess. Supersedes the vision-only `oak_latency_map.md` for the
end-to-end picture. Built after demo bags exposed that I had been quoting
the wrong latency number — see §1.

## 1. The correction up front: two different "latency" numbers

The OAK publishes **three** latency figures (`oak_driver._log_pipeline_health`,
oak_driver_node.py L2008–L2075). They are NOT interchangeable:

| Field | Frame it measures | Measured (demo bags 2026-06-04) | Is it the control path? |
|---|---|---|---|
| **`v0_lat_ms`** (`/oak/health[idx]`) | the **active detector's** frame (cv2 on 320×180, or the on-device NN) — set from the detection's capture stamp, L2391 (NN) / L2471 (cv2) | **~56–62 ms** | **YES — this is what feeds the controller** |
| `jpeg_lat_ms` | the MJPEG GUI live-feed frame (L2325) | ~100–150 ms | no — GUI feed |
| `/oak/latency_ms` (topic) | the rgb-raw drain frame (L2323) | ~124 ms | no — GUI/debug frame |

**I had been citing `/oak/latency_ms` (~120 ms) as the control latency. It
isn't.** The controller's vision input rides `v0_lat_ms` ≈ **56 ms**, even
on cv2. The GUI "circle" trails because it rides `jpeg_lat_ms` (~100 ms)
**plus** rosbridge + browser render — which is why it looks ~2–3× worse
than the loop actually is. (Verified: cv2 and YOLO bags both ~56–62 ms
`v0_lat_ms`; switching backends did **not** change control latency.)

## 2. The full chain (capture → platform tilting)

| # | Stage | Latency | Rate | Value & where it's set | Measured? |
|---|---|---|---|---|---|
| 1 | **Exposure integration** | 1.5 ms | — | `OAK_EXP_US=1500` — `stable_bot.service` env | config |
| 2 | Sensor readout + ISP + on-device detect + USB transport | **bundled in `v0_lat_ms` ≈ 56–62 ms** | — | pipeline in `oak_driver._build_pipeline` | **measured** (`/oak/health`) |
| 2b | **New-detection cadence** | — | **~12 Hz** (`v0_pub_hz`) → a fresh sighting every ~80 ms | cv2 host loop / NN rate | **measured** |
| 3 | `ball_localizer` reproject pixel→plane mm | ≤ 16 ms (tick) | **60 Hz** | `create_timer(1/60, _tick)` — ball_localizer_node.py L162 | code |
| 4 | `ball_kf` predict/update → `/ball_state` | ≤ 10 ms | 100 Hz predict | `DT=1/100` — ball_kf_node.py L39 | code |
| 5 | BALL_TRACK reads state, computes tilt (+ lookahead) | ≤ 1 tick | **200 Hz** (5 ms) | `ctrl_period_s = 1/level_loop_hz`, default `LEVEL_DEFAULT_HZ=200` — stewart_control_node.py L1051, L183 | code |
| 6 | `_do_set_pose → _compute_motor_targets → feeder.set_pos_targets` | ~0 (sets array) | — | L2789, L939 | code |
| 7 | **ODriveFeeder streams `Set_Input_Pos`(target, vel_ff)** | **≤ 20 ms** | **50 Hz** | `ODriveFeeder period=0.02` — L897 (hardcoded, not plumbed) | code |
| 8 | CAN transmit | ~0.1 ms/frame | — | 1 Mbps — `install_on_pi.sh` / `global_limits` | code |
| 9 | **ODrive pos loop → leg slews to target** | **leg-vel-cap bound** (see §3) | — | `vel_limit = soft_max_vel*1.5` — L1476; `INPUT_MODE_PASSTHROUGH` L1518; `vel_ff` L1021 | code |
| 10 | Motor torque + platform inertia + settling | mechanical | — | hardware | — |

**Vision half** (capture → fresh `/ball_state`): ≈ `v0_lat_ms` (56 ms) +
localizer tick (≤16) + KF (≤10) ≈ **~70–80 ms**, refreshed at the **~12 Hz**
detection cadence (the 60 Hz `/ball_xy_mono` is reprojection, not new info).

**Control half** (`/ball_state` → platform actually tilting): control tick
(≤5) + feeder (≤20) + CAN (~0) + **leg slew** (§3) + mechanical.

## 3. The actuation stage — your "max turn speed" (the missing piece)

This was never in the latency budget and is a real, tunable lag:

- The legs run in **position mode, passthrough** (no trapezoidal profile),
  with an ODrive **velocity ceiling** `vel_limit = soft_max_vel × 1.5`
  (stewart_control_node.py L1476).
- `soft_max_vel` default **1.0 turn/s** (`global_limits.yaml`
  `default_soft_max_leg_vel_turns_per_sec`), hard ceiling **4.0 turn/s**
  (`hard_max_leg_vel_turns_per_sec`). At 71 mm/rev that's **~106 mm/s** leg
  speed at the default cap.
- **Set live by the GUI** vel-cap slider (range 0.1–2.0) →
  `cmd: set_speed_cap` → `self.soft_max_vel` (L2473). So the operator can
  throttle how fast the platform can move.
- The feeder adds **velocity feed-forward** (`vel_ff = Δtarget/Δt`, L1005,
  L1021) so the legs *lead* a moving target instead of chasing it —
  this is what keeps tracking error low despite the 50 Hz feeder.
- A separate **tilt-rate** envelope exists in `global_limits.yaml`
  (`hard_max_tilt_rate_deg_per_sec 15`, soft default `8`). In BALL_TRACK
  the per-tick growth cap is `tilt_slew_up_deg_per_s = 100` (effectively
  off, ball_track_gains.yaml), so the **binding actuation limit is the leg
  velocity cap**, not the tilt-rate cap.

**Why it matters:** a tilt *reversal* to brake the ball (e.g. +1.2° → −1.2°)
is a real leg displacement at ≤106 mm/s — tens of ms — on top of the 20 ms
feeder quantization and mechanical settling. So the **actuation half is a
similar order to the vision half**, and lowering the vel cap (or the
1.2° tilt cap) directly slows the platform's ability to respond.

## 4. Re-reading `control_latency_s = 0.2`

The BALL_TRACK lookahead (`control_latency_s`, ball_track_gains.yaml,
operator-set via the GUI gains panel) extrapolates the ball forward by
0.2 s. I earlier called that "too long vs the 56 ms vision latency" — but
the lookahead must cover the **whole loop**: vision (~56 ms) + localizer/KF/
tick (~30 ms) + feeder (~20 ms) + **leg slew + mechanical settling**. Summed,
~0.15–0.2 s is **not** unreasonable as a total see→platform-moved delay.
The right way to set it is to **measure** the end-to-end delay (§6), not
to assume it equals `v0_lat_ms`.

## 5. Configuration provenance — where the knobs live

| Knob | File | Set via | Notes |
|---|---|---|---|
| `max_tilt_deg` (1.2), `kp/kd/ki`, `control_latency_s` (0.2), stiction params | `stewart_bringup/config/ball_track_gains.yaml` | **GUI → Gains & Control Parameters panel** (`ball_track_save_gains`) | 1.2° is operator-tuned and **below** the STEP_ID floor `max(θ_s+2.5, 8)°` → saturation/orbiting |
| `soft/hard_max_leg_vel`, `hard_max_tilt_rate`, translation rate | `stewart_bringup/config/global_limits.yaml` | hard caps = file edit; soft = **GUI vel-cap slider** | the actuation speed cap |
| leg `min/max/rest_pos_turns` (the Z envelope) | `stewart_bringup/config/leg_limits.yaml` | homing | clamps every `set_pose` |
| `level_loop_hz` (200) → `ctrl_period_s` | CLI `--level-loop-hz` | `stable_bot.service` ExecStart | outer-loop rate |
| Exposure 1500/ISO 3200, focus 130, backend cv2 | `stable_bot.service` env (`OAK_*`) | deploy; runtime via `/oak/cmd_exposure`,`/oak/cmd_focus` | adopted 2026-06-04 |
| feeder rate (50 Hz) | **hardcoded** `ODriveFeeder period=0.02` (L897) | — | not exposed; see §20.3 of `ball_physics_modeling_plan.md` |

## 6. Measured vs estimated, and the missing test

- **Measured (vision):** `v0_lat_ms` (~56 ms), `jpeg_lat_ms`, detection
  rate (`v0_pub_hz` ~12 Hz), loop rates (topic counts in the demo digests).
- **Measured (actuation) — NEW 2026-06-04:** the demo digest now reports
  an `actuation_ms` = the *effective* command→motion delay (stages 7–10),
  computed by cross-correlating commanded tilt (`/ball_track/diagnostic`)
  against the platform IMU's fused tilt (`/platform/imu/data`, ~240 Hz).
  No new instrumentation — both signals were already bagged; the digest
  just never read the IMU. Lands in `digest.summary.json` →
  `latency_breakdown` (with `see_to_move_est_ms = vision + actuation`)
  and is plotted as the IMU-tilt overlay on the commanded-tilt row. The
  math (`stewart_bringup/stewart_bringup/_latency.py`) is unit-tested:
  a pure delay is recovered exactly, a mechanical rise adds only a
  constant offset (`test_latency.py`). **Re-digest the existing demo
  bags to get this retroactively** — the data is already there.
- **Still estimated:** the split *within* actuation (feeder ZOH vs leg
  slew vs mechanical rise) — `actuation_ms` is their sum. Separating them
  needs the Tier-2 capture-stamp propagation (see below).
- **The remaining missing piece:** per-stage **vision** sub-latency
  (capture→detect→localize→KF). The localizer/KF re-stamp with `now()`
  (ball_localizer_node L477), destroying the capture time before the
  controller sees it. Fix = propagate the capture stamp + log freshest-
  sighting age in `/ball_track/diagnostic` (backward-compatible append).

## 7. Doc-accuracy fixes made / flagged
- `ARCHITECTURE.md` L181 comment says "Set_Input_Pos × 6 @ 200 Hz"; the
  feeder is **50 Hz** (`period=0.02`). Flagged — the 200 Hz is the *level
  loop*, not the feeder.
- `oak_latency_map.md` implied `/oak/latency_ms` (~120 ms) was the control
  latency; corrected here and there to `v0_lat_ms` ≈ 56 ms.

## 8. Bottom line for the latency thread
- **Vision detection latency ≈ 56 ms** and was never the ~120 ms we feared;
  on-device YOLO doesn't lower it (cv2 already runs on the 320×180 frame).
- **The under-counted lag is actuation** (feeder 50 Hz + leg-vel-cap slew +
  mechanical) — comparable to vision. **Now measured** per-run as
  `actuation_ms` in the demo digest (§6); re-digest the bags to see it.
- **The demo's actual limiter is the 1.2° tilt cap** (saturation/orbiting),
  which is a gains issue, not latency.
