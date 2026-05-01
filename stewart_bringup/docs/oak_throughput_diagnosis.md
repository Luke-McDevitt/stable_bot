# OAK pipeline throughput diagnosis (2026-04-30 → 2026-05-01)

**Goal:** ≥ 30 Hz of low-latency ball position data for the closed-loop
controller. Earlier 15 Hz cv2-on-host was visibly laggy in operation.

**Final state:** 37–60 Hz YOLOv6n on-device + 10 Hz GUI live preview +
10 Hz platform_pose + 21 Hz cv2 fallback, all simultaneously.

This doc captures the diagnostic chain so future-me / future-anyone
doesn't re-walk it.

---

## TL;DR

The Myriad X had three compute hogs sharing it: the V1 NN, the V0.5c
color-filter NN (already left over from earlier work), and the **MJPEG
encoder running at 60 fps**. The encoder was the silent killer —
nothing in the logs said "I'm using SHAVES." Disabling it took V1
from 22 Hz → 60 Hz. The fix is a DepthAI Script node that throttles
the encoder input to 10 fps, freeing 5⁄6 of its Myriad X cost while
preserving the GUI live feed.

---

## Symptoms

- V1 YOLOv8n live: capped at 14–22 Hz
- Standalone benchmark of the same blob: **60 Hz** ✓
- Disabling the V0.5c NN: no change
- MultiThreadedExecutor refactor: no change
- Disabling mono: no change
- Disabling MJPEG encoder: **22 Hz → 60 Hz** ← bingo

The standalone benchmark (`stewart_vision/scripts/benchmark_v1_yolo.py`)
was the breakthrough. It built a pipeline with **only** `cam_rgb →
ImageManip → NN → XLinkOut` — no JPEG, no mono, no other consumers.
Hit 60 Hz cleanly. That proved the device was capable; everything
extra in the live pipeline was the loss.

## Failed hypotheses (recorded so they aren't re-tested)

1. **USB bandwidth.** Initially suspected USB 2.0 limiting raw RGB
   transfer. Switched to USB SUPER (3.0). No measurable improvement
   on YOLO rate — NN output is small (~16 KB/frame).
2. **rgb_raw XLink backpressure.** Made the XLinkOut non-blocking on
   device side. Helped a tiny bit (15 → 17 Hz) but nowhere near 60.
3. **ImageManip input blocking.** Made it non-blocking. No change.
4. **Mono camera throttling cam_rgb.** Disabled mono. No change.
5. **V0.5c NN sharing Myriad X with V1.** Disabled V0 NN. No change.
6. **Single-threaded ROS executor.** Switched to MultiThreadedExecutor
   with separate callback groups + 120 Hz YOLO drain timer in its own
   ReentrantCallbackGroup. **No change.** Confirmed the bottleneck
   was device-side, not host-side: `/oak/health` showed `v0_arr` (NN
   attempts received from device) capped at the same rate as YOLO
   publishes — host wasn't dropping frames at the queue.
7. **YOLO model size.** Trained YOLOv6n thinking it'd be 2× faster.
   Confirmed in benchmark (60 Hz vs v8n's 30 Hz). Helped — but the
   v8n's 30 Hz benchmark was already higher than the live driver's
   22 Hz, so the v8 → v6 swap wasn't the actual fix either.

## The actual diagnosis

- The OAK has a Myriad X VPU with N SHAVE cores. NN inference, JPEG
  encoding, and a few other operations all consume SHAVES.
- The MJPEG encoder running at the source frame rate (60 fps) was
  burning ~70% of Myriad X compute that the NN needed.
- DepthAI doesn't surface "SHAVE budget" directly — there's no log
  saying "encoder taking 60% of compute." Has to be diagnosed via
  the strip-and-measure approach in the standalone benchmark.

## The fix (Script-node throttle)

```python
script = pipeline.create(dai.node.Script)
script.setScript("""
import time
last = 0.0
period = 1.0 / 10.0   # 10 Hz target
while True:
    msg = node.io['rgb_in'].get()
    now = time.monotonic()
    if now - last >= period:
        node.io['rgb_out'].send(msg)
        last = now
""")
script.inputs['rgb_in'].setBlocking(False)
script.inputs['rgb_in'].setQueueSize(1)
cam_rgb.video.link(script.inputs['rgb_in'])
script.outputs['rgb_out'].link(enc_rgb.input)
```

Script nodes run Python on the OAK device itself. The throttle is
on-device — frames are dropped before they reach the encoder, so
the encoder only does compute work at the throttled rate.

`OAK_JPEG_FPS` env var controls the throttle (default 10). 0 disables
throttling (encoder at full 60 fps, V1 drops to ~22 Hz). Lower values
free more Myriad X budget; 5 fps live feed is still usable for tuning.

## Knobs added during diagnosis (kept for future use)

| Env var | Default | Purpose |
|---|---|---|
| `OAK_DISABLE_V0_NN` | `1` | Skip V0.5c color NN (legacy, no longer needed) |
| `OAK_ENABLE_MONO` | `0` | Skip mono camera (ArUco runs on RGB now) |
| `OAK_DISABLE_JPEG` | `0` | Hard-disable encoder (loses GUI feed entirely) |
| `OAK_JPEG_FPS` | `10` | Script-node throttle for the encoder |
| `OAK_FORCE_V1_ARCH` | unset | Force `v6` or `v8` blob if both present |

## Post-fix performance (steady state)

| Topic | Rate | Notes |
|---|---|---|
| `/oak/ball/v0/yolo_pixel` | 37–60 Hz | Live driver depending on competing host work |
| `/oak/rgb/image_compressed` | 10 Hz | Throttled, GUI live preview |
| `/platform_pose` | ~10 Hz | ArUco solve, plenty for the controller |
| `/oak/ball/v0/cv2_pixel` | 21 Hz | Fallback when YOLO unavailable |

End-to-end NN latency (capture → host receive): ~45–60 ms.

## Lessons

1. **Standalone benchmarks are essential.** Without
   `benchmark_v1_yolo.py`, we'd still be guessing whether the model
   or the pipeline was the bottleneck. Build the smallest possible
   reproducer and measure.
2. **DepthAI Script nodes are powerful.** Running Python on the
   device for in-pipeline rate-limiting / routing avoids host
   round-trips. Vastly underused.
3. **Myriad X compute is finite and shared, but not surfaced.**
   Anything you add to the pipeline can quietly steal from the NN.
   When in doubt, strip everything and measure the ceiling.
4. **Per-topic Hz on `/oak/health` is the most useful diagnostic.**
   `v0_arr` (host received) vs `v0_pub` (host published) tells you
   immediately whether the bottleneck is device-side or host-side.

## Adjacent fixes folded in

- `cv2.aruco.estimatePoseBoard` removed in OpenCV 4.7+ —
  switched to `Board.matchImagePoints + solvePnP`. Was the cause of
  `pose=0` and the off-platform cv2 hallucinations earlier in the
  session.
- Multi-threaded executor + separate callback groups remained
  worth keeping; the YOLO drain on its own thread protects against
  any future host-side regressions.
