# Latency map — every source, photon → motor (and the GUI's separate display lag)

**Added 2026-06-04.** A complete map of where latency comes from in the
ball-tracking loop, what's measurable today, and how to attack each. Built
to get the "cv2 circle latency all over the place" under control. Companion
to `oak_throughput_diagnosis.md`, `oak_highspeed_detection_analysis.md`,
and `ball_physics_modeling_plan.md` §19.

## The one distinction that matters first

There are **two different latencies**, and they're easy to conflate:

- **Control-path latency** — photon → `/ball_state` → controller → motor.
  This is what the predictor/controller fights. The vision part is measured
  by **`/oak/latency_ms`** (device-capture timestamp → host receipt).
- **GUI display latency** — the "cv2 circle" you see drawn over the live
  feed lags by control-path-vision latency **plus** the JPEG/rosbridge/
  browser render path. The circle bouncing "all over the place" is partly
  *display* jitter (rosbridge + browser), **not** necessarily the control
  loop. Fix the right one: tune the controller against `/oak/latency_ms`,
  not against how the circle looks.

`exposure_sweep.py` and the demo digests log `/oak/latency_ms`, so we can
track the *control-path* number across changes (e.g. did dropping
autofocus help? → compare the manual-focus runs to the autofocus baseline
of ~122 ms).

## The chain (per stage)

| # | Stage | Typical | Measured by | Jitter source | How to reduce |
|---|---|---|---|---|---|
| 1 | **Exposure integration** | **8 ms** (= `OAK_EXP_US`) | `exposure_sweep.py` (latency vs exp) | — (fixed per setting) | **Shorten exposure** (the sweep) — direct latency + blur win |
| 2 | Sensor readout (IMX378, rolling) | ~10–20 ms | part of `/oak/latency_ms` | minor | lower resolution / ISP scale |
| 3 | On-OAK ISP (debayer + 1080p→540p scaler) | ~ms–10s ms | part of `/oak/latency_ms` | scaler contention | smaller ISP scale; suspected ~20 Hz scaler cap |
| 4 | **Transport: capture → host (USB)** | **bulk of ~50–120 ms** | **`/oak/latency_ms`** | **USB bandwidth + scheduling** (raw RGB ~1.5 MB/frame on the cv2 path) | **detect on-device, ship (cx,cy)** (throughput doc); smaller frames |
| 5 | Host cv2 HSV detect (decode+thresh+contour) | ~3–12 ms | `measure_detector_latency.py` | **host CPU contention** (rosbridge JPEG, other nodes) | platform-mask ROI; reduce contention; on-device |
| 6 | `ball_localizer` (pixel→plane mm) | ~1 ms | — | — | — |
| 7 | `ball_kf` (KF predict/update) | <1 ms | — | — | — |
| 8 | → `/ball_state` published | — | — | — | — |
| 9 | Control tick (BALL_TRACK reads state) | ≤ loop period (5 ms @200 Hz) | controller diag | — | already 200 Hz (§20) |
| 10 | Feeder → ODrive (CAN Set_Input_Pos) | ≤ 20 ms (50 Hz) + ~0.1 ms frame | CAN util | feeder period | **feeder 50→100 Hz** (§20.3) → halves ZOH lag |
| 11 | ODrive current loop + motor + mechanical | ms-scale + settling | — | load-dependent | — |
| — | **GUI circle** (display only) | adds JPEG+rosbridge+browser | (visual) | **rosbridge + browser** | lower JPEG quality, throttle, Blob URLs (already done, `vision_control_tuning_lessons.md`) |

## The measured reality (so far)

- `/oak/latency_ms` (cv2 path, autofocus): **mean ~108–122 ms, std ~29 ms,
  p95 ~160 ms** — the jitter is as much the problem as the mean.
- On-device NN path measured ~45–60 ms capture→host (`oak_throughput_diagnosis.md`)
  — i.e. **stage 4 is the dominant, reducible chunk**, and detecting
  on-device is the big lever.
- Detection throughput: `v0_arr ≈ 27 Hz`, `v0_pub` bounces 5–13 Hz with the
  ball moving (HSV misses under motion blur — a *detection* problem that
  looks like latency because the controller goes stale between hits).

## Attack order (biggest, cheapest first)

1. **Shorten exposure** (`exposure_sweep.py`) — removes stage 1 directly
   (8 ms → 1–2 ms) *and* kills motion blur (which fixes the v0_pub
   collapse, stage 4/5 effective staleness). One change, two wins.
2. **Use per-sample `/oak/latency_ms` in the predictor** instead of a fixed
   `control_latency_s` — the ±29 ms jitter is otherwise baked-in error
   (§19). Free.
3. **Reduce host contention** for stage 5 — the JPEG encoder throttle is
   done; confirm cv2 isn't fighting rosbridge for CPU.
4. **On-device detection** (stage 4) — the structural fix that takes
   capture→host from ~108 ms to ~50 ms, but needs the detector robust
   under motion (couples to #1). The `oak_phase2b_on_device_v0.md` path.
5. **Feeder 50→100 Hz** (stage 10) — halves actuation ZOH lag (§20.3).
6. **Did autofocus removal help?** Now that focus is fixed manual 130,
   compare `/oak/latency_ms` from a fresh run to the ~122 ms autofocus
   baseline. AF's CONTINUOUS_VIDEO refocus pauses the sensor — removing it
   should cut both mean and jitter; the exposure sweep's latency column is
   the first place we'll see it.

## How we keep tracking it

- **Every `exposure_sweep.py` row** logs latency mean/std/p95 → exposure↔
  latency curve + the post-autofocus number.
- **Demo bags** record `/oak/latency_ms` (+ `/oak/health`); the digests
  already plot it.
- **TODO (the missing end-to-end piece):** a "ball-tap" analyzer that times
  detection→motor response, to measure stages 9–11 (only the vision part
  is instrumented today). Listed in `vision_control_tuning_lessons.md`.
