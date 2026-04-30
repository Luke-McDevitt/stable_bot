# OAK Phase 2B — on-device V0 ball detection

Phase 2A (shipped 2026-04-30) downscales the RGB stream on-device to
320×180 BGR888p via an `ImageManip` node and runs the cv2 HSV +
contour detector on the Pi against the small frame. Net effect: V0
USB load dropped ~9× and the Pi can keep up with 60 Hz of detection
without breaking a sweat.

Phase 2B is the next step if we still need more headroom or want to
shrink the see-to-command latency further. Instead of shipping the
small RGB frame across USB, we run the HSV → centroid math on the
OAK's Myriad X VPU and send only the result (cx, cy, r, conf, ts) —
≈16 bytes per detection — to the host. This drops the per-detection
USB cost from ~170 KB to ~16 B and removes the cv2 Python work from
the Pi entirely.

## Why we didn't do it in Phase 2A

- DepthAI's `Script` node runs MicroPython on a single LEON core with
  no `numpy` / `cv2` available. Scalar pixel loops max out around
  5 Hz on a 160×90 frame — slower than the cv2 path on the Pi.
  Confirmed via Luxonis docs: *"the Script node shouldn't be used
  for heavy computing (e.g. image manipulation/CV)"*.
- The only on-device path that scales is the `NeuralNetwork` node,
  which requires a compiled `.blob` (Myriad X format, produced from
  ONNX via `blobconverter`). That's a bigger lift than the Phase 2A
  ImageManip-only change, so we shipped 2A first to unlock 60 Hz on
  the existing detector.

## Architecture

```
ColorCamera(1080p, 60 Hz)
     │
     ├─ ISP scale 1/2  →  cam_rgb.video (540p)
     │                       │
     │                       └─ VideoEncoder (MJPEG q70) ──→ XLinkOut 'rgb_jpeg'  (GUI live feed)
     │
     ├─ ImageManip (320×180 BGR888p)  ──┬──→ XLinkOut 'rgb_v0'  (Phase 2A: cv2 on Pi)
     │                                  │
     │                                  └──→ NeuralNetwork ──→ XLinkOut 'v0_nn'   (Phase 2B: NNData
     │                                          │                                  cx, cy, r, conf)
     │                                          │
     │                                          └─ blob: Kornia HSV → soft argmax
     │
     └─ MonoCamera left ──→ XLinkOut 'left'  (ArUco)
```

## Custom blob — minimum-viable design

Everything runs in PyTorch / Kornia, exported via ONNX, compiled by
`blobconverter`. The full forward pass should be order-of 10k FLOPs
(not megaflops) so it inferences in <2 ms on the Myriad X.

```python
import torch
import torch.nn as nn
import kornia.color

class V0HSVDetector(nn.Module):
    """RGB888p [3,H,W] → (cx, cy, conf) ∈ [0,1]³.

    Cheaper than a CNN: HSV threshold + soft argmax over the mask.
    Trains nothing — the HSV bounds are baked-in constants.
    """
    HSV_LO = torch.tensor([5/180,   140/255, 90/255])   # spec match
    HSV_HI = torch.tensor([22/180,  255/255, 255/255])

    def forward(self, rgb_chw_uint8: torch.Tensor) -> torch.Tensor:
        x = rgb_chw_uint8.float() / 255.0                 # [3,H,W]
        hsv = kornia.color.rgb_to_hsv(x.unsqueeze(0))     # [1,3,H,W]
        lo = self.HSV_LO.to(hsv.device).view(1, 3, 1, 1)
        hi = self.HSV_HI.to(hsv.device).view(1, 3, 1, 1)
        mask = ((hsv >= lo) & (hsv <= hi)).all(dim=1, keepdim=True)
        mask = mask.float()                               # [1,1,H,W]
        # Soft argmax via centroid of mask weights.
        H, W = mask.shape[-2:]
        ys = torch.linspace(0, 1, H, device=mask.device).view(1,1,H,1)
        xs = torch.linspace(0, 1, W, device=mask.device).view(1,1,1,W)
        m_sum = mask.sum() + 1e-6
        cx = (mask * xs).sum() / m_sum
        cy = (mask * ys).sum() / m_sum
        conf = mask.sum() / (H * W)                       # area fraction
        return torch.stack([cx, cy, conf])                # [3]
```

Compile:

```bash
pip install blobconverter
python -c "
import torch, blobconverter
from v0_blob import V0HSVDetector
m = V0HSVDetector().eval()
dummy = torch.zeros(3, 180, 320, dtype=torch.uint8)
torch.onnx.export(m, dummy, 'v0.onnx', input_names=['rgb'],
                  output_names=['cxcyconf'], opset_version=12)
blob = blobconverter.from_onnx('v0.onnx', shaves=6, data_type='FP16')
print('blob at', blob)"
```

Resulting `.blob` goes in `stewart_vision/blobs/v0_hsv_centroid.blob`.

## Pipeline wiring (oak_driver_node.py)

```python
nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath(BLOB_DIR / 'v0_hsv_centroid.blob')
nn.setNumInferenceThreads(1)
nn.input.setBlocking(False)
nn.input.setQueueSize(1)
v0_manip.out.link(nn.input)
add_out('v0_nn', nn.out)
```

Host side:

```python
self.q_v0_nn = device.getOutputQueue('v0_nn', 1, False)
# in _tick:
nn_data = self.q_v0_nn.tryGet()
if nn_data is not None:
    cx_n, cy_n, conf = nn_data.getFirstLayerFp16()  # 3 floats
    cx = cx_n * RGB_W
    cy = cy_n * RGB_H
    # NN node auto-propagates the input ImgFrame's getTimestamp()
    # and getSequenceNum(), so capture-time stamping works the same
    # way it does in Phase 2A.
```

The `getFirstLayerFp16()` API (DepthAI 2.x) returns the inference
output as a flat list of FP16 → Python float values.

## Risks & open questions

- **Soft-argmax over the whole mask** can be biased by spurious pixels
  outside the platform. The Pi-side mask currently fixes that by
  ANDing with the projected platform polygon; we'd need to either
  bake the mask into the blob (fixed shape — won't track platform
  pose) or AND on the Pi after receiving (cx, cy) but before the
  ray-plane projection. Cleanest answer: send the mask as a second
  NN input via `ImageManip`'s data port.
- **Kornia's `rgb_to_hsv` may not compile** to a Myriad-friendly graph
  — it uses ops the OpenVINO front-end occasionally chokes on.
  Fallback: write the HSV transform from scratch with `min`/`max`
  primitives.
- **Confidence metric** — `mask.sum()/area` is a crude heuristic.
  Replace with circularity (perimeter² / area) once we know what the
  Myriad X can compile cheaply.
- **Drift vs cv2** — the soft argmax weights *every* matching pixel
  equally, where cv2's `minEnclosingCircle` weights only the contour.
  Worst case: a stray HSV-matching pixel pulls the centroid 2-3 px
  off. Mitigation: morphological open inside the blob (1-pixel kernel
  via average pooling + threshold).

## Pointers in the existing code

`stewart_vision/stewart_vision/oak_driver_node.py`:
- `_build_pipeline` — where the `NeuralNetwork` node would be added
  alongside the existing `v0_manip`.
- The `_tick` block tagged `TODO(phase2b)` — replace the cv2 detector
  block with the `q_v0_nn.tryGet()` path.
- `V0_W`, `V0_H`, `V0_SCALE_X/Y` — coordinate-space constants stay
  unchanged; the NN outputs normalized coords that map back the same
  way.

## Decision criteria

Don't pursue 2B until:
- Phase 2A is solid (V0 reliably 60 Hz, controller closed-loop
  stable).
- Either we hit a USB / Pi-CPU ceiling that 2A can't push past, OR
  we measure see-to-command latency (`/oak/latency_ms` + bag stamp
  diffs) and find the cv2 step is still the dominant cost.

If both hold, budget half a day for the blob compile + test cycle —
most of the time goes into tuning `blobconverter` flags and verifying
the Kornia ops actually round-trip through OpenVINO.
