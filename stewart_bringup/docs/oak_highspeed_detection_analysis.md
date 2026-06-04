# OAK-D Pro high-speed ball detection — analysis & improvement plan

**Added 2026-06-04.** A from-scratch look at whether we're using the
OAK-D Pro AF anywhere near its potential for fast-ball detection, what
we've already tried, and what others do that we haven't. Companion to
`oak_throughput_diagnosis.md` (the 2026-04-30 throughput work) and
`ball_physics_modeling_plan.md` §19–20 (latency).

## TL;DR

We are detecting the ball on the **rolling-shutter RGB sensor (IMX378)**
with an **8 ms exposure**, while the camera's **two global-shutter mono
sensors (OV9282) — the ones Luxonis explicitly markets for ping-pong-ball
tracking — sit disabled.** Our motion-blur, USB-bandwidth, and
detection-miss problems are largely downstream of using the wrong sensor
for fast motion. The high-speed-vision playbook (global shutter +
sub-millisecond exposure + IR illumination, optionally an IR-bright
target) is exactly what this "Pro" hardware was built for, and we own all
of it. Plausible upside: from ~12.6 Hz / ~108 ms today to ~100+ Hz /
~15–30 ms, while *removing* the cv2-vs-YOLO robustness tradeoff.

## 1. Exactly what we run today

From `oak_driver_node.py` + the `/oak/config` snapshot in the Demo-2 bag:

| Setting | Value | Note |
|---|---|---|
| Detection sensor | **RGB IMX378** (`CAM_A`) | **rolling shutter**, autofocus |
| Resolution | 1080p → `setIspScale(1,2)` → **540p** | suspected ISP-scaler ~20 Hz cap noted in code |
| RGB fps | 60 | but raw only reaches host at ~27 Hz (USB BW) |
| **Exposure** | **8000 µs (8 ms)** | `OAK_EXP_US`; at 800 mm/s → **6.4 mm motion blur** |
| ISO | 800 (1600 in the demo) | analog gain |
| Focus | **default `auto` (CONTINUOUS_VIDEO)** | the FPS-killer the code itself documents |
| Mono cameras (OV9282) | **DISABLED** (`OAK_ENABLE_MONO=0`) | "ArUco runs on RGB now" |
| IR projector / IR flood LED | not used for detection | the "Pro" features |
| Detector | cv2 HSV (orange) on RGB, or on-device YOLOv8n 320² | `/ball_xy_mono` = *monocular projection*, not the mono cams |
| Depth | off | ball XY via ArUco-plane projection |
| USB | SuperSpeed (USB3) | verified |

Measured result (Demo-2 `20260502T031335Z`): `v0_arr` 27 Hz, `v0_pub`
**12.6 Hz**, `/oak/latency_ms` **108 ms mean ± 29**. Detection collapses
under motion ("v0_pub crash 13 → 5 Hz once the ball started moving").

## 2. What we've already tried (credit where due)

The throughput history is genuinely thorough — this list is so the next
person doesn't re-walk it:

- Auto-exposure → stuck at ~67 ms → 15 Hz, so switched to **manual 8 ms**.
- Auto-focus CONTINUOUS_VIDEO → sensor pauses to refocus → FPS killer →
  manual-focus option added.
- ISP-scale modes (540p / 720p / 1080p) — suspected scaler rate-limit.
- Frame-pool bumps to stop back-pressure stalls.
- **Disabled mono** to free the pipeline ("ArUco on RGB").
- MJPEG encoder throttled to 10 fps — freed the Myriad X, 22 → 60 Hz YOLO.
- Moved detection **on-device** (YOLOv6n/v8n, 37–60 Hz) — but it **loses
  the ball under fast motion**, so cv2-on-host stayed the demo default.
- Widened the HSV gate so motion-blurred (desaturated) pixels still pass.
- Verified USB3.

Every one of these treats the symptom on the **rolling-shutter RGB
path**. The throughput work got the *pipeline* fast; it didn't change the
*sensor* or the *exposure*, which is where the motion problem actually
lives.

## 3. The core problem

Two compounding issues, both intrinsic to the current sensor + exposure
choice:

1. **Rolling shutter (IMX378).** Rows are exposed sequentially, so a
   fast-moving ball is geometrically skewed/smeared even before blur.
   Global-shutter sensors expose all pixels at once and "freeze" fast
   motion ([Luxonis image-quality docs](https://docs.luxonis.com/hardware/platform/sensors/image-quality/),
   [global-shutter for fast objects](https://www.aiusbcam.com/news/782215089792057431.html)).
2. **8 ms exposure.** At 800 mm/s the ball travels 6.4 mm during a single
   frame → a smeared, desaturated streak that HSV misses and YOLO can't
   localize. The fix is well-known: **shorter exposure → less blur, but
   darker → compensate with more gain or, better, IR illumination**
   ([DepthAI image-quality / motion-blur](https://docs.luxonis.com/projects/api/en/latest/tutorials/image_quality/)).

These are *why* cv2 misses half the frames and why on-device YOLO "loses
the ball almost immediately" under motion — not a tuning detail, a sensor
choice.

## 4. What the hardware can actually do

The OAK-D Pro is built for exactly this:

- **OV9282 mono: 1 MP (1280×800) global shutter, up to 120 FPS, IR-sensitive.**
  Luxonis literally pitches the global-shutter pair for tracking "a ping
  pong ball" and "fast-moving objects … at over 200 FPS"
  ([OAK-D Pro / global shutter](https://docs.luxonis.com/hardware/products/OAK-D%20Pro),
  [OV9282 module](https://www.arducam.com/ov9282-mono-global-shutter-wide-angle-oak.html)).
  Our ball is a 40 mm ping-pong/foam sphere — the canonical demo.
- **IR illumination LED (flood) + IR dot projector** — the "Pro" parts.
  IR flood lets you light the scene invisibly and run **sub-millisecond
  exposures** on the IR-sensitive mono cams, freezing motion in the dark
  with no visible flicker to operators
  ([IR illumination](https://docs.luxonis.com/hardware/platform/sensors/image-quality/)).
- **On-device YOLO** via the `YoloDetectionNetwork` node — decode on the
  VPU, ship only `(cx, cy)` over USB; OAK handles vision up to 120 FPS.
- **Onboard IMU (BNO086)** — frame-synced; a free latency/sync reference.

## 5. What others do for high-speed detection (that we don't)

The professional motion-tracking recipe, all supported here:

1. **Global shutter, not rolling.** Detect on the OV9282 pair. No skew,
   120 FPS, IR-capable.
2. **Sub-millisecond exposure.** 30 µs freezes a propeller; <1 ms freezes
   our ball (0.8 mm at 800 mm/s). Trade light for sharpness.
3. **IR illumination + an IR-bright / retroreflective target.** Flood IR,
   make the ball the brightest thing in frame, and detection becomes a
   trivial blob-centroid at 120 FPS with zero blur — the same principle
   as commercial mocap with retroreflective markers
   ([retroreflective IR tracking](https://arxiv.org/pdf/2303.13681),
   [BrightMarker fluorescent tracking](https://hcie.csail.mit.edu/research/brightmarker/brightmarker.html)).
4. **Detect on-device, ship coordinates.** Removes the USB-raw bottleneck
   that caps us at 27 Hz.

## 6. Gap analysis — issues & improvements, prioritized

| # | Issue (today) | Improvement | Effort | Expected effect |
|---|---|---|---|---|
| 1 | Detect on rolling-shutter RGB; global-shutter mono disabled | Move ball detection to an **OV9282 mono cam** | Med (new detect + mono→plane calib) | No skew, 120 FPS-capable, IR-ready |
| 2 | 8 ms exposure → 6.4 mm blur | Drop to **0.5–2 ms** (with §3 light) | Trivial (`OAK_EXP_US`) | ~4–16× less blur → detection survives motion |
| 3 | IR flood/projector unused | **IR flood + IR-bright/retroreflective ball** on the mono cam | Low (env + a ball) | Sub-ms exposure feasible; ball pops as a blob |
| 4 | Focus default `auto` (CONTINUOUS_VIDEO) | **Lock `OAK_FOCUS_MODE=manual`** at the platform distance | Trivial | Removes refocus FPS stalls (we *documented* this, still default-auto) |
| 5 | cv2-on-host capped at 27 Hz by USB raw | **On-device blob/NN**, ship `(cx,cy)` | Low–Med (partly built) | Kills the bandwidth bottleneck → 100+ Hz |
| 6 | YOLO loses ball under motion | Root cause is blur (#2/#1); fix that and on-device becomes reliable | — | Removes the cv2-vs-YOLO tradeoff entirely |
| 7 | Suspected ISP-scaler ~20 Hz cap at 1080p→540p | Mono path needs **no ISP scaling** (native 1280×800) | — (falls out of #1) | Sidesteps the scaler entirely |

## 7. Recommended path (the integrated recipe)

The cheapest high-leverage sequence, in order — each step is independently
testable:

1. **Lock manual focus** (`OAK_FOCUS_MODE=manual`, `OAK_FOCUS_POS` for
   ~600 mm). Free, removes refocus stalls. *(issue 4)*
2. **Drop exposure to ~1–2 ms** and raise light (ISO and/or room light)
   on the *current* RGB path; A/B `v0_pub` under motion. *(issue 2)* —
   this alone should recover much of the publish gap.
3. **Bring up an OV9282 mono cam as a parallel ball detector**: global
   shutter, native 1280×800, on-device blob/NN, publish `(cx,cy)`; project
   onto the ArUco plane with mono intrinsics. *(issues 1, 5, 7)* The
   stereo/per-eye scaffolding (`/ball_xy_stereo`, the V1 left/right TODO)
   means this isn't from zero.
4. **Add IR flood + an IR-bright/retroreflective ball**, push exposure
   sub-ms on the mono cam. *(issue 3)* Ball becomes a trivial bright blob
   at 120 FPS, zero blur — the endgame.

Keep the RGB camera for the **ArUco pose** (slow-moving, color/feature-
rich, lower rate is fine) and use the mono cam for the **fast ball**. Dual
stream, each sensor doing what it's good at.

Plausible end state: ball at **~100–120 Hz**, capture→host **~15–30 ms**,
no motion blur, no rolling-shutter skew, one robust detector. That
collapses both the publish gap (§20.1) and most of the latency (§19) at
once — and it's what the $400 of hardware was designed to do.

## 8. Tradeoffs, caveats, risks (be honest)

- **Mono is grayscale → no HSV color discrimination.** This is the real
  cost of moving off RGB. Mitigations: an IR-bright/retroreflective ball
  (blob detection, trivial), or a tiny NN trained on mono frames. Cheap
  experiment first: flood IR, short exposure, see if the orange foam is
  separable on mono before committing.
- **The ArUco ring is detected on RGB.** Keep it there (pose changes
  slowly); only the ball moves to mono. Verify the ring isn't IR-
  retroreflective if you flood IR, or it'll bloom.
- **IR dot projector vs flood.** Use the *uniform flood* LED for ball
  illumination; the *dot projector* adds texture (good for stereo depth,
  bad as ball backdrop) — likely disable it on the detection cam.
- **New calibration**: mono intrinsics + mono→platform-plane mapping
  (factory stereo calib + `calibrate_oak.py` already most of the way).
- **Rolling shutter also skews ArUco pose during fast platform motion** —
  another reason a global-shutter pose path could help later, but pose is
  not the current bottleneck.
- This does **not** reduce the *control* dead-time (§19) by itself — it
  cuts the *vision* contribution; the model-based predictor is still the
  tool for the residual lag.

## 9. Cheap experiments to validate before committing

1. **Exposure sweep on RGB**: set `OAK_EXP_US` to 4000/2000/1000/500,
   `OAK_ISO` up to compensate; record `v0_pub` vs ball speed. Confirms the
   blur hypothesis at near-zero cost.
2. **Manual focus lock**: `OAK_FOCUS_MODE=manual`; confirm steady fps.
3. **Mono blob feasibility**: enable a mono cam, IR flood on, 1 ms
   exposure; eyeball whether the ball is a clean blob (with/without an
   IR-reflective coating) before building the detector.
4. **On-device latency check**: re-run `measure_detector_latency.py` on
   the mono/on-device path vs the 108 ms cv2 baseline.

## 10. Sources

- [OAK-D Pro product page / specs (Luxonis)](https://docs.luxonis.com/hardware/products/OAK-D%20Pro)
- [Luxonis image quality — global shutter, IR, motion blur](https://docs.luxonis.com/hardware/platform/sensors/image-quality/)
- [DepthAI image-quality tutorial (exposure / motion blur)](https://docs.luxonis.com/projects/api/en/latest/tutorials/image_quality/)
- [OV9282 global-shutter mono module](https://www.arducam.com/ov9282-mono-global-shutter-wide-angle-oak.html)
- [Global shutter for fast-moving objects](https://www.aiusbcam.com/news/782215089792057431.html)
- [Retroreflector localization / mobile mocap (IR markers)](https://arxiv.org/pdf/2303.13681)
- [BrightMarker — fluorescent markers for object tracking (MIT)](https://hcie.csail.mit.edu/research/brightmarker/brightmarker.html)
- [Limiting max exposure for low blur (DepthAI issue #732)](https://github.com/luxonis/depthai-core/issues/732)
