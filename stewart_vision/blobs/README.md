# OAK on-device V0 blobs (Phase 2B)

This directory holds compiled `.blob` files for the OAK-D Pro AF's
Myriad X VPU. They replace the host-side cv2 HSV+contour V0 detector
with on-device inference, dropping per-detection USB load from ~1.5 MB
(raw frame) to ~16 B (NNData with three FP16 floats).

## Building

The blob isn't built automatically — `blobconverter` is a Luxonis
cloud service that needs internet, and the Pi may not have torch
installed. Build from a dev machine:

```bash
pip install torch onnx blobconverter
cd ~/stable_bot_repo
python stewart_vision/scripts/build_v0_blob.py
```

That writes `stewart_vision/blobs/v0_320x180.blob`. Commit it:

```bash
git add stewart_vision/blobs/v0_320x180.blob
git commit -m "blob: rebuild V0 NN (color weights ...)"
git push
```

Then on the Pi:

```bash
~/stable_bot_repo/stewart_bringup/scripts/pi_deploy.sh
```

## Activating

The OAK driver picks `cv2` (the host detector) by default. To switch
to the NN backend, set `OAK_V0_BACKEND=nn` in the systemd override:

```bash
sudo systemctl edit stable_bot.service
```
Add:
```
[Service]
Environment=OAK_V0_BACKEND=nn
```
Save, then `sudo systemctl restart stable_bot.service`.

The `[boot]` log will show which backend was selected. If you ask
for `nn` but the blob isn't found at startup, the driver falls back
to `cv2` and logs a warning — your demos won't break.

## Tuning

Edit the color-score weights in `scripts/build_v0_blob.py` and rebuild.
The current defaults are tuned for saturated orange foam ball under
the headlamp / IKEA-panel lighting. If you change ball color or
lighting drastically:

- High-R, low-B objects: keep current weights, raise `BIAS`.
- Different hue (e.g., red vs orange vs yellow): adjust `W_G` upward
  for yellow-ish, downward for redder.
- More motion blur tolerance: lower `SCORE_FLOOR` (lets dimmer
  pixels contribute to the centroid; risk: false positives).

After rebuilding, the OAK driver picks up the new blob on next
restart — no Python source change required.
