#!/usr/bin/env python3
"""Build the on-device V0 ball-detection .blob (Phase 2B).

The blob runs on the OAK-D Pro AF's Myriad X VPU and replaces the
host-side cv2 HSV+contour detector with an NN that produces just
(cx_norm, cy_norm, conf) per frame — three FP16 floats — over USB.

The NN is intentionally minimal: a single 1×1 convolution with
hand-tuned weights for "saturated orange ball on dark platform"
plus a soft-argmax centroid head. No training. The convolution
implements a linear color score (high R, lower G, lowest B), the
argmax extracts the ball's centroid, and confidence is the area
fraction of the score-positive pixels. This compiles cleanly
through OpenVINO IR → Myriad X .blob, where Kornia's full HSV
pipeline sometimes does not.

Build process (must be run on a machine with PyTorch + an internet
connection — blobconverter is a Luxonis cloud service that does
the OpenVINO + Myriad X compile remotely):

    pip install torch onnx blobconverter
    python stewart_vision/scripts/build_v0_blob.py

Output:
    stewart_vision/blobs/v0_320x180.blob

Commit the .blob to git so the Pi can use it without needing a
local PyTorch install. Switch the OAK driver to use it via env:

    Environment=OAK_V0_BACKEND=nn

In the systemd override (see stewart_bringup/scripts/stable_bot.service).

If you tune the color thresholds, re-run this script and recommit.
"""
from __future__ import annotations

import argparse
import os
import sys

import torch
import torch.nn as nn


# ---- Model constants -------------------------------------------------------

# Input resolution to the NN. Chosen to match the ImageManip downscale in
# stewart_vision/oak_driver_node.py (V0_W, V0_H = 320, 180). Multiple of
# 16 — required by ImageManip's warp/resize engine on RVC2.
NN_W, NN_H = 320, 180

# Linear color-score weights tuned for saturated orange ball. Input
# order is BGR (the format ImageManip emits with BGR888p):
#   score = w_b * B + w_g * G + w_r * R + bias
# Picked so:
#   - bright orange (R≈1, G≈0.6, B≈0.1) → score ≈ 0.55
#   - white (R=G=B≈1)                    → score ≈ 0.10
#   - dark platform (R=G=B≈0.05)         → score ≈ 0.005
#   - blue / green objects                → score < 0
W_B = -0.50
W_G = -0.40
W_R = +1.00
BIAS = -0.10

# Confidence floor: count a pixel as "ball" only if score > 0.10.
# Same value used in the area-fraction confidence numerator.
SCORE_FLOOR = 0.10


class V0Detector(nn.Module):
    """RGB-frame → (cx_norm, cy_norm, conf).

    Tiny model on purpose. The Myriad X compiles 1x1 convs +
    elementwise ops without complaint; reductions (sum, argmax)
    are also supported. All ops are static-shape friendly so
    OpenVINO IR shape inference doesn't choke.

    Input: BGR888p planar uint8, shape (1, 3, H, W). DepthAI's
    NeuralNetwork node delivers this directly when the upstream
    ImageManip outputs BGR888p.

    Output: tensor of shape (3,) in [cx_norm, cy_norm, conf]:
      cx_norm, cy_norm ∈ [0, 1]      pixel position normalised to (W, H)
      conf             ∈ [0, 1]      area fraction of "ball" pixels
    """

    def __init__(self):
        super().__init__()
        # 1×1 conv with explicit BGR-ordered weights; a single linear
        # combination of channels, no spatial mixing.
        self.score = nn.Conv2d(3, 1, kernel_size=1, bias=True)
        with torch.no_grad():
            self.score.weight.data = torch.tensor(
                [[[[W_B]], [[W_G]], [[W_R]]]], dtype=torch.float32)
            self.score.bias.data = torch.tensor([BIAS], dtype=torch.float32)

    def forward(self, bgr_uint8: torch.Tensor) -> torch.Tensor:
        # bgr_uint8: (1, 3, H, W) in [0, 255], BGR planar.
        x = bgr_uint8.float() / 255.0
        s = self.score(x)                 # (1, 1, H, W)
        m = torch.relu(s - SCORE_FLOOR)   # zero out non-ball pixels

        # Soft argmax — weighted centroid of m. Static shape so the
        # OpenVINO front-end picks up the index tensors as constants.
        H = bgr_uint8.shape[-2]
        W = bgr_uint8.shape[-1]
        ys = torch.arange(H, dtype=torch.float32).view(1, 1, H, 1)
        xs = torch.arange(W, dtype=torch.float32).view(1, 1, 1, W)
        # Move to model's device (no-op on CPU export, harmless on GPU).
        ys = ys.to(m.device)
        xs = xs.to(m.device)

        # Add a tiny constant so the divisor is never zero — empty-mask
        # case still produces a finite (cx, cy) at the frame center.
        denom = m.sum() + 1e-3
        cx = (m * xs).sum() / denom / float(W)
        cy = (m * ys).sum() / denom / float(H)

        # Confidence: how much of the frame the score-positive pixels
        # cover. Above ~0.001 means the ball was seen.
        n_above = (m > 0.0).to(torch.float32).sum()
        conf = n_above / float(H * W)

        return torch.stack([cx, cy, conf])


# ---- ONNX export + blobconverter --------------------------------------------

def export_onnx(model: V0Detector, out_path: str) -> None:
    model.eval()
    # Static-shape dummy input: blobconverter wants no dynamic axes.
    dummy = torch.zeros((1, 3, NN_H, NN_W), dtype=torch.uint8)
    torch.onnx.export(
        model,
        dummy,
        out_path,
        input_names=['bgr_uint8'],
        output_names=['cxcyconf'],
        opset_version=12,
        do_constant_folding=True,
        dynamic_axes=None,
    )
    print(f"[build_v0_blob] ONNX written to {out_path}")


def compile_blob(onnx_path: str, blob_path: str, shaves: int) -> None:
    try:
        import blobconverter  # type: ignore
    except ImportError:
        print("[build_v0_blob] ERROR: blobconverter not installed.\n"
              "   pip install blobconverter", file=sys.stderr)
        sys.exit(1)
    print(f"[build_v0_blob] compiling for Myriad X (shaves={shaves}) — "
          f"this calls blobconverter's cloud service, ~30 s")
    blob = blobconverter.from_onnx(
        model=onnx_path,
        data_type='FP16',
        shaves=shaves,
        version='2022.1',
    )
    # blobconverter returns a path under its cache; copy to our blobs dir.
    import shutil
    os.makedirs(os.path.dirname(blob_path), exist_ok=True)
    shutil.copy(blob, blob_path)
    print(f"[build_v0_blob] blob written to {blob_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--out',
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'blobs', f'v0_{NN_W}x{NN_H}.blob'),
        help='Where to write the .blob. Default: stewart_vision/blobs/.')
    parser.add_argument(
        '--shaves', type=int, default=6,
        help='Number of SHAVE cores to compile for (max 16). 6 is a '
             'safe default leaving headroom for other NN nodes; bump '
             'higher if you start running multiple NN workloads in '
             'parallel and want more inference throughput.')
    parser.add_argument(
        '--onnx-only', action='store_true',
        help='Just emit the ONNX, skip blobconverter (for offline debug).')
    args = parser.parse_args()

    print(f"[build_v0_blob] input shape = (1, 3, {NN_H}, {NN_W})  BGR uint8")
    print(f"[build_v0_blob] output      = (cx_norm, cy_norm, conf)  FP16[3]")
    print(f"[build_v0_blob] color score weights B/G/R = "
          f"{W_B:+.2f}/{W_G:+.2f}/{W_R:+.2f}  bias={BIAS:+.2f}")

    onnx_path = args.out.replace('.blob', '.onnx')
    model = V0Detector()
    export_onnx(model, onnx_path)

    if args.onnx_only:
        print("[build_v0_blob] --onnx-only set; not compiling blob.")
        return

    compile_blob(onnx_path, args.out, args.shaves)
    print("[build_v0_blob] done — commit the .blob and switch the Pi to "
          "OAK_V0_BACKEND=nn.")


if __name__ == '__main__':
    main()
