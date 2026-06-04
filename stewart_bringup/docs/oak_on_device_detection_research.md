# On-device ball detection — research & scoping (latency project)

**Added 2026-06-04. Research-only; no code changes yet.** The goal is to
move ball detection from the Pi onto the OAK's Myriad X VPU so it ships
only `(cx, cy, conf)` (~16 B) instead of a frame, removing the host-side
cv2 work and the frame transfer — the lever for the ~120 ms latency that
focus/exposure could not touch (`oak_latency_map.md`). This doc captures
the prior attempts (and why they failed), the state of the art, three
concrete options with effort/payoff, and a recommendation.

## 1. Why on-device, and what it actually buys

The measured `/oak/latency_ms` is **~120 ms while cv2 is detecting vs
~80 ms idle** — the gap is **host queue-wait**: while the Pi runs HSV +
contour, it can't drain the raw-frame queue fast enough, so frames age in
the buffer (`oak_latency_map.md`). On-device detection removes *both* the
host cv2 work and the frame from the host loop, so:

- **Latency**: the existing on-device YOLO path measured **~56 ms**
  capture→host (`oak_throughput_diagnosis.md`) — roughly **half** of the
  cv2 path, and with **less jitter** (no host-load-dependent queue-wait).
- **Rate**: on-device runs **37–60 Hz** vs cv2's **~12 Hz** publish — the
  controller would get a fresh ball every ~17–27 ms instead of ~80 ms.
  That ~3–5× rate bump is as valuable as the latency cut for the predictor.

Note Phase 2A already downscales the cv2 frame to 320×180 on-device, so
this is **not** a USB-bandwidth fix — it's a **host-CPU / queue-wait** fix.

## 2. Prior art in this repo (and why each stalled)

| Attempt | What | Status | Why |
|---|---|---|---|
| **Phase 2A** (shipped 2026-04-30) | on-device `ImageManip` → 320×180 BGR, cv2 HSV on Pi | **live** (`OAK_V0_BACKEND=cv2`) | cut V0 USB ~9×; Pi keeps 60 Hz. But cv2 still runs on host → the queue-wait latency. |
| **V0.5c on-device color NN** | linear 1×1-conv "orange-ness" + 5×5 density pool + centroid blob (`build_v0_blob.py`) | **disabled** (`OAK_DISABLE_V0_NN=1`) — "NaN output, wrong detection ceiling" | see §3 — both causes are diagnosed + fixable |
| **V1 YOLOv8n on-device** | trained 320² detector, `v1_yolov8n_320.blob` | **available, not default** | 37–60 Hz, ~56 ms, but **lost the ball under motion blur** (`step_id_tuning_lessons.md` §3) — *the blur we just fixed* |

## 3. Why V0.5c produced NaN (root cause) — and the fixes

`build_v0_blob.py` builds an ONNX graph (no kornia, so it dodged the
[kornia `rgb_to_hsv` ONNX export bug](https://github.com/kornia/kornia/issues/1327)).
The centroid does `cx = mx_avg / (m_avg + eps)` with `eps = 1e-6`. **In
FP16 (the Myriad X datatype), `1e-6` underflows to 0** (smallest FP16
normal ≈ 6e-5). So on any frame with no orange match (the dark platform,
or the ball out of view), `m_avg → 0`, `denom → 0`, and `0/0 → NaN`
([the classic divide-by-zero NaN](https://github.com/coloria-dev/coloria/issues/75)).
The "wrong detection ceiling" is the second known issue: a **whole-frame
soft-argmax centroid** is pulled off by any stray orange pixel outside the
platform (the Phase 2B doc's risk #1), so it can't match cv2's
contour+`minEnclosingCircle` accuracy.

**Both are fixable:**
- **NaN:** use an **FP16-safe epsilon** (e.g. `1e-2`) or `clamp(denom, min=…)`,
  and **conf-gate** the output (publish only when `m_avg > floor`, so a
  no-detection frame returns "no ball" instead of a NaN centroid).
- **Accuracy:** AND the result against the **projected platform mask** —
  cheapest on the host *after* receiving `(cx,cy)` but *before* the
  ray-plane projection (we already have the polygon); or send the mask as
  a second NN input (Phase 2B doc) — more work.

## 4. State of the art / hard constraints

- **Script node is out for CV.** DepthAI's `Script` node is MicroPython on
  one LEON core, no numpy/cv2 — "shouldn't be used for heavy computing
  (image manipulation/CV)"; scalar pixel loops ~5 Hz. Use it only for flow
  control / decoding ([Luxonis NeuralNetwork+Script](https://docs.luxonis.com/software/depthai-components/nodes/neural_network),
  [on-device programming](https://docs.luxonis.com/en/latest/pages/tutorials/on-device-programming/)).
- **The only scalable on-device path is the `NeuralNetwork` node** — any
  OpenVINO-compilable model as a `.blob` (ONNX → `blobconverter`). Custom
  output, decoded on host ([NeuralNetwork node](https://docs.luxonis.com/software/depthai-components/nodes/neural_network)).
- **Latency knobs** ([Luxonis low-latency](https://docs.luxonis.com/projects/api/en/latest/tutorials/low-latency/)):
  more **SHAVES/NCE** for the NN; **XLink chunk size 0** (send as soon as
  ready); **non-blocking queues, size 1**; keep the Leon CPU off 100 %.
  These apply to whichever NN path we pick.

## 5. Options

### Option A — Re-test the existing on-device YOLO with the deblurred image *(try first)*
- **Effort:** ~minutes. Flip the V0 backend to the YOLO path
  (`OAK_V0_BACKEND=v1_yolo` per `oak_phase2b_on_device_v0.md` / the backend
  selector), roll the ball, measure `/oak/latency_ms` + detect rate.
- **Payoff:** ~56 ms latency + 37–60 Hz, **zero new code**, if it holds.
- **Why it might work now:** its *only* documented failure was losing the
  ball under motion blur — and 1500 µs/ISO 3200 just froze the ball
  (verified rolling). The deblur removes the exact failure mode.
- **Risk:** the 320² letterbox makes the ball ~20 px (small-object weakness)
  and the training set was 262 sharp frames (`oak_highspeed_detection_analysis.md`
  detector section). Deblur helps; resolution/training may still cap it —
  the test tells us in minutes.

### Option B — Fix + ship the on-device color-centroid blob (V0.5c → V0.6)
- **Effort:** ~half day. Patch `build_v0_blob.py` (FP16-safe eps + conf
  gate), rebuild via `blobconverter`, add the host platform-mask AND, test.
- **Payoff:** ~56 ms + high rate, **color-based** (sidesteps YOLO's
  small-object/training fragility), interpretable, tunable via the existing
  weight sliders.
- **Risk:** blobconverter/OpenVINO op-compile gotchas; must verify the
  mask-AND restores cv2-level accuracy.

### Option C — Tiny learned color-segmentation CNN
- **Effort:** high (collect/label, train, compile). **Payoff:** most robust
  to lighting/clutter. **Defer** unless A and B both fall short.

## 6. Recommendation & scope

1. **Do Option A first** — it's a config flip and a 5-minute test, and the
   one thing that killed it (blur) is fixed. If on-device YOLO now tracks
   the rolling ball, we get ~56 ms + 37–60 Hz for free and we're done.
2. **If YOLO still misses** (small-object/resolution), **do Option B** — the
   V0.5c failures are diagnosed (FP16 eps, mask bias), so it's a known,
   bounded fix, not open research. Budget ~half a day for the
   blob-compile/test loop.
3. **Apply the DepthAI latency knobs** (SHAVES/NCE, XLink chunk 0,
   non-blocking q=1) to whichever path wins.
4. **Validate** against the same metric we used for exposure: `/oak/latency_ms`
   mean/std + detect rate under a **moving** ball, before/after — and
   confirm the host-queue-wait gap (detecting vs idle) collapses.

**Decision criteria (from `oak_phase2b_on_device_v0.md`, still valid):**
pursue on-device only because we've now *measured* the host cv2/queue-wait
as the dominant remaining latency — which we have. The model-based
predictor (Phase 2) remains the parallel track that makes whatever latency
remains tolerable.

## 7. Open questions
- Does on-device YOLO's accuracy hold on the rolling ball at 1500 µs, or is
  320² resolution the real cap? (Option A test answers this.)
- Is the platform-mask AND best done on host (simple, needs `(cx,cy)` only)
  or baked as a 2nd NN input (keeps it fully on-device)? Start host-side.
- Can `blobconverter` round-trip the V0.6 graph cleanly (the 2A ops did),
  and does a larger FP16 eps fully kill the NaN across the dark-platform
  case? (Build + unit-check the blob output on a black frame.)

## 8. Sources
- [DepthAI NeuralNetwork node](https://docs.luxonis.com/software/depthai-components/nodes/neural_network) · [On-device programming (Script node limits)](https://docs.luxonis.com/en/latest/pages/tutorials/on-device-programming/)
- [DepthAI low-latency tutorial (SHAVES/NCE, XLink chunk, queues)](https://docs.luxonis.com/projects/api/en/latest/tutorials/low-latency/)
- [OAK NN inference walkthrough (PyImageSearch)](https://pyimagesearch.com/2022/12/19/oak-d-understanding-and-running-neural-network-inference-with-depthai-api/)
- [kornia `rgb_to_hsv` ONNX export bug #1327](https://github.com/kornia/kornia/issues/1327) · [divide-by-zero NaN in RGB→HSV](https://github.com/coloria-dev/coloria/issues/75)
- Repo: `oak_phase2b_on_device_v0.md`, `scripts/build_v0_blob.py`, `blobs/README.md`, `oak_throughput_diagnosis.md`, `oak_latency_map.md`.
