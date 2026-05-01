#!/usr/bin/env python3
"""Standalone YOLO-only benchmark — measure the actual Myriad X
inference ceiling for the V1 YOLOv8n blob, with NO competing
pipeline elements (no mono, no JPEG encoder, no rgb_raw XLink, no
V0.5c NN). This isolates the question of "is the model itself slow"
vs "is the pipeline slow".

Run:
    sudo systemctl stop stable_bot.service   # release the OAK
    cd ~/stable_bot_repo
    python3 stewart_vision/scripts/benchmark_v1_yolo.py
    # ... expect a table of measured rates
    sudo systemctl start stable_bot.service

Pipeline:
    cam_rgb (1080p ISP /2 = 540p, 60 fps)
      → ImageManip (setResizeThumbnail 320×320 letterboxed)
      → NeuralNetwork (v1_yolov8n_320.blob)
      → XLinkOut

Reports:
    - Frames received per second over a 5-second window
    - Per-message age (latency from device clock to host receipt)
    - Min/max/mean inference latency

If this benchmark reports ~30+ Hz, the bottleneck in the live
oak_driver is contention from other nodes or stream backpressure.
If it reports ~14 Hz, the bottleneck is the model + Myriad X
compute, and the path forward is a smaller input (256×256) or a
lighter architecture.
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import depthai as dai


def find_blob(name: str) -> str:
    """Find the named blob in the usual locations."""
    repo = os.path.expanduser('~/stable_bot_repo')
    candidates = [
        os.path.join(repo, 'stewart_vision', 'blobs', name),
        os.path.expanduser(
            f'~/ros2_ws/install/stewart_vision/share/stewart_vision/'
            f'blobs/{name}'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return None


def build_minimal_pipeline(blob_path: str, rgb_fps: int = 60,
                           shaves: int = 6,
                           num_inference_threads: int = 2,
                           include_rgb_raw: bool = False,
                           include_jpeg: bool = False) -> dai.Pipeline:
    p = dai.Pipeline()
    cam = p.create(dai.node.ColorCamera)
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam.setResolution(
        dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setIspScale(1, 2)  # 540p output
    cam.setFps(rgb_fps)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    manip = p.create(dai.node.ImageManip)
    manip.initialConfig.setResizeThumbnail(320, 320, 114, 114, 114)
    manip.initialConfig.setFrameType(dai.RawImgFrame.Type.BGR888p)
    manip.setMaxOutputFrameSize(320 * 320 * 3)
    manip.inputImage.setBlocking(False)
    manip.inputImage.setQueueSize(1)
    cam.video.link(manip.inputImage)

    nn = p.create(dai.node.NeuralNetwork)
    nn.setBlobPath(blob_path)
    nn.setNumInferenceThreads(num_inference_threads)
    nn.input.setBlocking(False)
    nn.input.setQueueSize(1)
    manip.out.link(nn.input)

    out = p.create(dai.node.XLinkOut)
    out.setStreamName('nn_out')
    out.input.setBlocking(False)
    out.input.setQueueSize(1)
    nn.out.link(out.input)

    # Optional consumers — for A/B testing whether they steal cam_rgb cycles.
    if include_rgb_raw:
        out_raw = p.create(dai.node.XLinkOut)
        out_raw.setStreamName('rgb_raw')
        out_raw.input.setBlocking(False)
        out_raw.input.setQueueSize(1)
        cam.video.link(out_raw.input)
    if include_jpeg:
        enc = p.create(dai.node.VideoEncoder)
        enc.setDefaultProfilePreset(rgb_fps,
            dai.VideoEncoderProperties.Profile.MJPEG)
        enc.input.setQueueSize(1)
        enc.input.setBlocking(False)
        enc.setNumFramesPool(2)
        cam.video.link(enc.input)
        out_jpeg = p.create(dai.node.XLinkOut)
        out_jpeg.setStreamName('rgb_jpeg')
        out_jpeg.input.setBlocking(False)
        out_jpeg.input.setQueueSize(1)
        enc.bitstream.link(out_jpeg.input)
    return p


def measure(label: str, pipeline: dai.Pipeline,
            usb_speed: dai.UsbSpeed,
            duration_s: float = 5.0) -> None:
    print(f"\n=== {label} ===")
    with dai.Device(pipeline, maxUsbSpeed=usb_speed) as device:
        print(f"  device: {device.getDeviceName()} | "
              f"USB: {device.getUsbSpeed()}")
        # 1 s warm-up
        q = device.getOutputQueue('nn_out', 1, False)
        t_warm = time.time() + 1.0
        while time.time() < t_warm:
            q.tryGet()
        # Measure
        ages_ms = []
        n = 0
        t0 = time.time()
        while time.time() - t0 < duration_s:
            msg = q.tryGet()
            if msg is None:
                continue
            n += 1
            try:
                age = (dai.Clock.now().total_seconds()
                       - msg.getTimestamp().total_seconds())
                ages_ms.append(max(0.0, age * 1000.0))
            except Exception:
                pass
        elapsed = time.time() - t0
        rate = n / elapsed if elapsed > 0 else 0.0
        if ages_ms:
            mn = min(ages_ms); mx = max(ages_ms)
            mean = sum(ages_ms) / len(ages_ms)
        else:
            mn = mx = mean = float('nan')
        print(f"  frames: {n} in {elapsed:.2f} s → "
              f"{rate:.1f} Hz")
        print(f"  age (ms): min {mn:.0f}  mean {mean:.0f}  max {mx:.0f}")


def main():
    super_speed = os.environ.get('USB', 'super').lower() == 'super'
    usb_speed = dai.UsbSpeed.SUPER if super_speed else dai.UsbSpeed.HIGH
    print(f"USB cap: {usb_speed.name}")

    # Blob lookup — try v6 first (preferred), fall back to v8.
    blobs = []
    for label, name in (('YOLOv6n', 'v1_yolov6n_320.blob'),
                        ('YOLOv8n', 'v1_yolov8n_320.blob')):
        p = find_blob(name)
        if p:
            blobs.append((label, p))
        else:
            print(f"  ({label} blob {name} not found, skipping)")

    if not blobs:
        raise SystemExit("No v1_*.blob found. Train + compile first.")

    for label, blob in blobs:
        print(f"\n##### {label} — {blob} ({os.path.getsize(blob):,} bytes) #####")

        # 1. Minimal: only YOLO. No RGB raw, no JPEG, no mono. The
        #    purest measure of what the model + Myriad X can do.
        measure(f"{label} MINIMAL (YOLO only, no RGB/JPEG)",
                build_minimal_pipeline(blob),
                usb_speed)

        # 2. With raw RGB output (representative of the live pipeline).
        measure(f"{label} + raw RGB",
                build_minimal_pipeline(blob, include_rgb_raw=True),
                usb_speed)


if __name__ == '__main__':
    main()
