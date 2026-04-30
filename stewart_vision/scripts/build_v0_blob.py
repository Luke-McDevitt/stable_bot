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
        # Precompute coordinate grids as buffers — they serialize as
        # ONNX Constants which OpenVINO IR handles cleanly. Doing
        # `torch.arange` inside forward() produces a Range op that
        # OpenVINO 2022.1 (the blobconverter default) sometimes fails
        # on with "Const layer ... has incorrect dimensions".
        ys = torch.arange(NN_H, dtype=torch.float32).view(1, 1, NN_H, 1)
        xs = torch.arange(NN_W, dtype=torch.float32).view(1, 1, 1, NN_W)
        self.register_buffer('ys_grid', ys / float(NN_H))   # already / H
        self.register_buffer('xs_grid', xs / float(NN_W))   # already / W
        # 1-D shape (not 0-D) — OpenVINO 2022.1's IR converter has
        # trouble with 0-D constants under some op compositions.
        self.register_buffer(
            'inv_area',
            torch.tensor([1.0 / float(NN_H * NN_W)],
                         dtype=torch.float32))

    def forward(self, bgr_uint8: torch.Tensor) -> torch.Tensor:
        # bgr_uint8: (1, 3, H, W) in [0, 255], BGR planar.
        x = bgr_uint8.float() / 255.0
        s = self.score(x)                 # (1, 1, H, W)
        m = torch.relu(s - SCORE_FLOOR)   # zero out non-ball pixels

        # Use adaptive_avg_pool2d for global reductions instead of
        # `.sum()` + `.stack()`. Pool ops keep the 4-D shape, sidestep
        # the 0-D-scalar / concat-axis corner cases that bit OpenVINO
        # 2022.1 with the previous version.
        m_avg = nn.functional.adaptive_avg_pool2d(m, 1)        # (1,1,1,1)
        mx_avg = nn.functional.adaptive_avg_pool2d(
            m * self.xs_grid, 1)                               # (1,1,1,1)
        my_avg = nn.functional.adaptive_avg_pool2d(
            m * self.ys_grid, 1)                               # (1,1,1,1)

        # Centroid = (sum of m*xs) / (sum of m); the H*W divisor in
        # both adaptive-avg-pool ops cancels. xs_grid / ys_grid are
        # already pre-normalized to [0, 1].
        denom = m_avg + 1e-6
        cx = mx_avg / denom                                    # (1,1,1,1)
        cy = my_avg / denom                                    # (1,1,1,1)
        # Output shape (1, 3, 1, 1). Host unpacks three floats from
        # NNData.getFirstLayerFp16(). Concat along channel axis is
        # well-supported by OpenVINO IR.
        return torch.cat([cx, cy, m_avg], dim=1)


# ---- ONNX export + blobconverter --------------------------------------------

def export_onnx(model: V0Detector, out_path: str) -> None:
    model.eval()
    # Static-shape dummy input: blobconverter wants no dynamic axes.
    dummy = torch.zeros((1, 3, NN_H, NN_W), dtype=torch.uint8)
    # Modern torch's onnx exporter sometimes externalizes weight
    # tensors into a sidecar `.onnx.data` file even for tiny models;
    # blobconverter's cloud service rejects that with "invalid
    # external data". Force the legacy exporter (dynamo=False) and
    # then re-save the ONNX flattened (save_as_external_data=False)
    # to guarantee a single self-contained file.
    try:
        torch.onnx.export(
            model, dummy, out_path,
            input_names=['bgr_uint8'],
            output_names=['cxcyconf'],
            opset_version=12,
            do_constant_folding=True,
            dynamic_axes=None,
            dynamo=False,
        )
    except TypeError:
        # Older torch (< 2.5) doesn't accept `dynamo=`.
        torch.onnx.export(
            model, dummy, out_path,
            input_names=['bgr_uint8'],
            output_names=['cxcyconf'],
            opset_version=12,
            do_constant_folding=True,
            dynamic_axes=None,
        )
    # Re-save inline so blobconverter sees a single file with no
    # ExternalDataInfo references.
    try:
        import onnx as _onnx
        m = _onnx.load(out_path)
        _onnx.save_model(m, out_path, save_as_external_data=False)
        # Clean up any sidecar that the first export wrote.
        sidecar = out_path + '.data'
        if os.path.isfile(sidecar):
            os.remove(sidecar)
    except Exception as e:
        print(f"[build_v0_blob] WARN: failed to flatten ONNX: {e}",
              file=sys.stderr)
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
    # blobconverter's defaults add `--mean_values=[127.5,...] --scale_values=
    # [255,...]` which is fine for FP32 inputs but errors out on uint8
    # ("Mean preprocessing can be applied to float inputs"). Our model
    # already normalises internally (x = bgr_uint8.float() / 255.0), so
    # override optimizer_params to drop the auto preprocessing entirely.
    blob = blobconverter.from_onnx(
        model=onnx_path,
        data_type='FP16',
        shaves=shaves,
        version='2022.1',
        optimizer_params=[],
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
