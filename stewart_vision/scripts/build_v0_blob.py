#!/usr/bin/env python3
"""Build the on-device V0 ball-detection .blob (Phase 2B).

Builds the ONNX graph **directly with `onnx.helper`** — no PyTorch.
The model is small enough that hand-building the graph is shorter
than wrapping it in a torch.nn.Module and exporting. Removes torch
as a dependency, which means the build can run on the Pi itself
when installed via `pip install --break-system-packages onnx
blobconverter` or via a venv — no need for a separate dev machine.

V0.5 (2026-04-30): added a spatial-coherence stage so the soft-
argmax doesn't get pulled by scattered noise pixels. The original
1×1-conv-only model worked as a per-pixel color filter, which was
mathematically equivalent to HSV thresholding — and just as fragile
to background noise. The new chain enforces "orange pixels must be
clustered to vote" by smoothing m_raw with a 5×5 average pool and
re-thresholding against `density_floor`. A 1-px noise speckle in a
25-px window has m_density ≈ 0.018 (rejected); a tight 4×4 ball
cluster has m_density ≈ 0.29 (passed).

Pipeline (single-frame, static-shape):

    bgr_uint8 (1, 3, H, W) uint8
      ↓ Cast to float
      ↓ Mul by 1/255            → x_norm
      ↓ Conv 1×1 [w_b, w_g, w_r] + bias  → score
      ↓ Sub by score_floor      → score_minus_floor
      ↓ Relu                    → m_raw  (per-pixel orange-ness)
      ↓ AveragePool 5×5 (pad=2) → m_density  (NEW: local cluster mass)
      ↓ Sub by density_floor    → m_density_minus_floor
      ↓ Relu                    → m  (only spatially-clustered pixels)
      ↓ GlobalAveragePool       → m_avg          (1, 1, 1, 1)
      ↓ Mul by xs_grid          → m_xs
      ↓ GlobalAveragePool       → mx_avg
      ↓ Mul by ys_grid          → m_ys
      ↓ GlobalAveragePool       → my_avg
      ↓ Add eps to m_avg        → denom
      ↓ Div mx_avg / denom      → cx
      ↓ Div my_avg / denom      → cy
      ↓ Concat[cx, cy, m_avg, axis=1]  → cxcyconf  (1, 3, 1, 1)

xs_grid / ys_grid are [0, 1]-normalised position grids; the H*W
divisor cancels between the avg-pool over (m * xs) and the avg-pool
over m, so cx is a true centroid in normalised coordinates.

Output is FP16[3] when read by `NNData.getFirstLayerFp16()` on the
host: [cx_norm, cy_norm, m_avg]. Host scales (cx_norm, cy_norm) by
the canonical 540p frame size before publishing /oak/ball/v0/nn_pixel.

Build process:

    pip install onnx blobconverter      # blobconverter is cloud-side
    python stewart_vision/scripts/build_v0_blob.py

Output: stewart_vision/blobs/v0_320x180.blob

Weights come from stewart_vision/blobs/v0_weights.json (the GUI's
slider-saved file). Falls back to the module defaults below when
the JSON is missing.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np


# ---- Model constants --------------------------------------------------------

# Input resolution to the NN. Chosen to match the ImageManip downscale in
# stewart_vision/oak_driver_node.py (V0_W, V0_H = 320, 180). Multiple of
# 16 — required by ImageManip's warp/resize engine on RVC2.
NN_W, NN_H = 320, 180

# Default linear color-score weights (orange-favoring, 2026-04-30
# retune). Sanity at saturated colours (R, G, B normalised to 1.0):
#   orange (1.00, 0.65, 0.00)  →  1.00 + 0.26 + 0.00 - 0.50 = 0.76 ✓
#   red    (1.00, 0.00, 0.00)  →  1.00 + 0.00 + 0.00 - 0.50 = 0.50 ✓ (lower)
#   white  (1.00, 1.00, 1.00)  →  1.00 + 0.40 - 1.50 - 0.50 = -0.60 ✓
# Override via stewart_vision/blobs/v0_weights.json (GUI sliders).
W_B = -1.50
W_G = +0.40
W_R = +1.00
BIAS = -0.50
SCORE_FLOOR = 0.30
# Spatial-coherence threshold (V0.5). After Conv1×1+Relu produces the
# raw per-pixel orange mask m_raw, we AveragePool it with a 5×5
# kernel to compute local density m_density, then keep only pixels
# where m_density > DENSITY_FLOOR. Tuned so that:
#   - 1 isolated noise px (m=0.46)            → m_density ≈ 0.018  (rejected)
#   - 2-3 scattered px in 5×5 window          → m_density ≈ 0.05   (rejected)
#   - 4×4 tight ball cluster (m≈0.46 each)    → m_density ≈ 0.29   (passed)
# Override via stewart_vision/blobs/v0_weights.json (GUI slider).
DENSITY_FLOOR = 0.05


def _load_weights_json():
    """Return (W_B, W_G, W_R, BIAS, SCORE_FLOOR, DENSITY_FLOOR) —
    JSON-overridden if stewart_vision/blobs/v0_weights.json exists,
    else the module defaults defined above."""
    import json as _json
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(os.path.dirname(here), 'blobs', 'v0_weights.json')
    if not os.path.isfile(path):
        return W_B, W_G, W_R, BIAS, SCORE_FLOOR, DENSITY_FLOOR
    try:
        with open(path) as f:
            d = _json.load(f) or {}
        return (
            float(d.get('w_b', W_B)),
            float(d.get('w_g', W_G)),
            float(d.get('w_r', W_R)),
            float(d.get('bias', BIAS)),
            float(d.get('score_floor', SCORE_FLOOR)),
            float(d.get('density_floor', DENSITY_FLOOR)),
        )
    except Exception as e:
        print(f"[build_v0_blob] WARN: {path} unreadable ({e}); "
              f"using defaults", file=sys.stderr)
        return W_B, W_G, W_R, BIAS, SCORE_FLOOR, DENSITY_FLOOR


# ---- Graph builder (onnx.helper, no torch) ---------------------------------

def build_onnx(out_path: str, w_b, w_g, w_r, bias,
               score_floor, density_floor) -> None:
    """Hand-construct the V0 detector ONNX graph using onnx.helper.

    Why hand-built: the original torch-based exporter pulled in ~500
    MB of dependencies for a model that fits in 60 lines of helper
    calls. The onnx package alone is enough.
    """
    try:
        import onnx
        from onnx import helper, TensorProto, numpy_helper
    except ImportError:
        print("[build_v0_blob] ERROR: onnx not installed.\n"
              "   pip install onnx", file=sys.stderr)
        sys.exit(1)

    H, W = NN_H, NN_W

    inp = helper.make_tensor_value_info(
        'bgr_uint8', TensorProto.UINT8, [1, 3, H, W])
    out = helper.make_tensor_value_info(
        'cxcyconf', TensorProto.FLOAT, [1, 3, 1, 1])

    initializers = []

    def _const(name, arr):
        initializers.append(numpy_helper.from_array(
            np.asarray(arr, dtype=np.float32), name=name))

    # 1/255 normaliser, broadcast across (1, 1, 1, 1).
    _const('scale', np.array([[[[1.0 / 255.0]]]], dtype=np.float32))

    # 1×1 conv weights — shape (out_channels=1, in_channels=3, 1, 1).
    # Input is BGR-planar so weight order matches the (W_B, W_G, W_R)
    # spec.
    _const('conv_w', np.array([[[[w_b]], [[w_g]], [[w_r]]]],
                              dtype=np.float32))
    _const('conv_b', np.array([bias], dtype=np.float32))

    _const('floor', np.array([[[[score_floor]]]], dtype=np.float32))
    _const('density_floor_const',
           np.array([[[[density_floor]]]], dtype=np.float32))

    # V0.5c (2026-04-30): NO in-model division.
    #
    # V0.5b tried fixing FP16 underflow by adding eps=1e-4 and scaling
    # m by 1000 before the Div. Myriad X *still* produced cx=cy=NaN
    # despite both fixes — confirmed via [nn-debug] log with the
    # rebuilt blob (eps=1e-4 verified in-model). The Div op itself is
    # the failure mode on Myriad X for this graph topology, not the
    # operands. Compiler may be fusing ops in a way that bypasses the
    # eps add, or the in-FP16 Div has its own quirks we can't probe.
    #
    # New approach: output the 3 sums (mx_avg_K, my_avg_K, m_avg_K),
    # all scaled by K=1000 so they fit comfortably in FP16 normal
    # range for any plausible ball size. Host computes the centroid
    # in FP32 — no in-model Div, no NaN possible. Math is identical:
    #     cx = (K * mx_avg) / (K * m_avg) = mx_avg / m_avg
    _const('m_scale', np.array([[[[1000.0]]]], dtype=np.float32))

    # 5×5 box-blur kernel for the spatial-coherence stage. Stored as
    # a Conv weight (1 in / 1 out / 5×5, all entries 1/25) instead of
    # using AveragePool: AveragePool with explicit padding silently
    # produces no output on Myriad X / OpenVINO 2022.1 (the host's
    # q_v0_nn.tryGet() returns None forever even though the ONNX
    # validates with onnxruntime locally — confirmed 2026-04-30).
    # Conv with the same kernel + pads is mathematically identical
    # and Conv is the most battle-tested op on the Myriad X NN
    # accelerator (everything else in this graph already uses it).
    _const('blur_kernel',
           np.full((1, 1, 5, 5), 1.0 / 25.0, dtype=np.float32))

    # Position grids pre-normalised to [0, 1]. Same trick as the
    # torch version — store as Constants so the Range op (which
    # OpenVINO 2022.1 is finicky about) never appears in the graph.
    ys = np.arange(H, dtype=np.float32).reshape(1, 1, H, 1) / float(H)
    xs = np.arange(W, dtype=np.float32).reshape(1, 1, 1, W) / float(W)
    initializers.append(numpy_helper.from_array(ys, name='ys_grid'))
    initializers.append(numpy_helper.from_array(xs, name='xs_grid'))

    nodes = [
        helper.make_node('Cast', ['bgr_uint8'], ['x_float'],
                         to=TensorProto.FLOAT),
        helper.make_node('Mul',  ['x_float', 'scale'], ['x_norm']),
        helper.make_node('Conv', ['x_norm', 'conv_w', 'conv_b'],
                         ['score'],
                         kernel_shape=[1, 1]),
        helper.make_node('Sub',  ['score', 'floor'],
                         ['score_minus_floor']),
        helper.make_node('Relu', ['score_minus_floor'], ['m_raw']),
        # Spatial-coherence stage: 5×5 box blur on the raw mask
        # converts per-pixel orange-ness into local cluster density.
        # Symmetric padding keeps the output at the same spatial size,
        # so xs_grid / ys_grid don't have to be resized. Implemented
        # as a Conv (not AveragePool) — see blur_kernel comment above.
        helper.make_node('Conv',
                         ['m_raw', 'blur_kernel'], ['m_density'],
                         kernel_shape=[5, 5],
                         strides=[1, 1],
                         pads=[2, 2, 2, 2]),
        helper.make_node('Sub', ['m_density', 'density_floor_const'],
                         ['m_density_minus_floor']),
        helper.make_node('Relu', ['m_density_minus_floor'], ['m']),
        # Scale m by K=1000 so all global-pool outputs stay in FP16
        # normal range (smallest m_avg becomes ~1.6e-2 even for a
        # 2-px ball). Host divides in FP32, so the K cancels:
        #     cx_n = (K * mx_avg) / (K * m_avg) = mx_avg / m_avg
        helper.make_node('Mul', ['m', 'm_scale'], ['m_K']),
        helper.make_node('GlobalAveragePool', ['m_K'], ['m_avg_K']),
        helper.make_node('Mul', ['m_K', 'xs_grid'], ['m_xs_K']),
        helper.make_node('GlobalAveragePool', ['m_xs_K'], ['mx_avg_K']),
        helper.make_node('Mul', ['m_K', 'ys_grid'], ['m_ys_K']),
        helper.make_node('GlobalAveragePool', ['m_ys_K'], ['my_avg_K']),
        # Output: [mx_avg_K, my_avg_K, m_avg_K] — three FP16 floats.
        # Host parses out[0]/out[2] for cx_n, out[1]/out[2] for cy_n,
        # and out[2] / 1000 for the conf used in the radius proxy.
        helper.make_node('Concat',
                         ['mx_avg_K', 'my_avg_K', 'm_avg_K'],
                         ['cxcyconf'], axis=1),
    ]

    graph = helper.make_graph(
        nodes,
        'V0Detector',
        inputs=[inp],
        outputs=[out],
        initializer=initializers,
    )
    model = helper.make_model(
        graph,
        opset_imports=[helper.make_opsetid('', 12)],
        producer_name='build_v0_blob.py',
    )
    # ir_version 7 is the last broadly-compatible version for OpenVINO
    # 2022.1; default in modern onnx is 8 which sometimes errors out.
    model.ir_version = 7
    onnx.checker.check_model(model)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    onnx.save(model, out_path)
    print(f"[build_v0_blob] ONNX written to {out_path}")


# ---- blobconverter compile --------------------------------------------------

def compile_blob(onnx_path: str, blob_path: str, shaves: int) -> None:
    try:
        import blobconverter  # type: ignore
    except ImportError:
        print("[build_v0_blob] ERROR: blobconverter not installed.\n"
              "   pip install blobconverter", file=sys.stderr)
        sys.exit(1)
    print(f"[build_v0_blob] compiling for Myriad X (shaves={shaves}) — "
          f"this calls blobconverter's cloud service, ~30 s")
    # blobconverter's defaults add `--mean_values=[127.5,...]
    # --scale_values=[255,...]` which is fine for FP32 inputs but
    # errors on uint8 ("Mean preprocessing can be applied to float
    # inputs"). Our model already normalises internally, so override
    # optimizer_params to drop the auto preprocessing entirely.
    blob = blobconverter.from_onnx(
        model=onnx_path,
        data_type='FP16',
        shaves=shaves,
        version='2022.1',
        optimizer_params=[],
    )
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
             'safe default leaving headroom for other NN nodes.')
    parser.add_argument(
        '--onnx-only', action='store_true',
        help='Just emit the ONNX, skip blobconverter (for offline debug).')
    args = parser.parse_args()

    w_b, w_g, w_r, bias, score_floor, density_floor = _load_weights_json()

    print(f"[build_v0_blob] input shape = (1, 3, {NN_H}, {NN_W})  BGR uint8")
    print(f"[build_v0_blob] output      = (cx_norm, cy_norm, conf)  FP16[3]")
    print(f"[build_v0_blob] weights B/G/R = "
          f"{w_b:+.2f}/{w_g:+.2f}/{w_r:+.2f}  "
          f"bias={bias:+.2f}  floor={score_floor:.2f}  "
          f"density={density_floor:.3f}")

    onnx_path = args.out.replace('.blob', '.onnx')
    build_onnx(onnx_path, w_b, w_g, w_r, bias, score_floor, density_floor)

    if args.onnx_only:
        print("[build_v0_blob] --onnx-only set; not compiling blob.")
        return

    compile_blob(onnx_path, args.out, args.shaves)
    print("[build_v0_blob] done — commit the .blob and pi_deploy.sh.")


if __name__ == '__main__':
    main()
