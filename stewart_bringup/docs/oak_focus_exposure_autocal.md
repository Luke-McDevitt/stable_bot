# Empirical focus & exposure auto-calibration (closed-loop, safe-by-design)

**Added 2026-06-04.** A data-driven (not theoretical) way to pick focus
and exposure for *this* rig, with image-clarity / detection feedback, and
a rollout that **cannot make things worse than today**. Refines
`IMPLEMENTATION_PLAN.md` Step 1; companion to
`oak_highspeed_detection_analysis.md`.

## Guiding principle — separate measurement from control

The safety comes from staging it so every step's *default* is
no-worse-than-the-last-known-good:

1. **Characterize** (offline sweeps) — pure measurement, zero operational
   risk, and it generates all the GitHub data.
2. **Analyze** the pushed data (with Claude) → choose the schema.
3. **Deploy as calibrated open-loop maps** `focus(Z)`, `exposure(Z)` —
   anchored to *measured* optima; already strictly better than today's
   hunting autofocus + 8 ms exposure.
4. **Add observe-only feedback monitors** (sharpness / detection below the
   calibrated expectation → flag, don't act).
5. **Only then** enable **bounded** closed-loop correction, behind a flag,
   with last-good fallback and settled-only gating.

At every stage the fallback is the previous known-good, so a regression
is impossible by construction. Steps 1–2 are the first deliverable and
are risk-free (they don't change live behavior at all).

## The geometry question (answered)

Yes — the camera is fixed overhead, so when the platform's **+Z raises
the deck, it moves *closer* to the camera**, and focus must shift
*nearer*. We do **not** model this with a lens equation — the empirical
calibration measures the best focus at each Z directly, so whatever the
true relationship is (monotonic), the data encodes it, *and* the focus
metric is computed on the real image, so distance is accounted for by
construction. The same Z-dependence affects brightness (inverse-square-ish
falloff), so exposure is calibrated per-Z too.

---

## Focus

### Metric (how to tell if the image is actually sharp)
Use a **no-reference sharpness operator** on the **ball ROI** (and the
ArUco region as a second check). The literature's top performers are
**Tenengrad** (variance of Sobel gradient — best *noise robustness*, which
matters at our high ISO) and **Variance-of-Laplacian** (simplest, widely
used); Sum-of-Modified-Laplacian is a good third
([Pertuz 2013 survey](http://isp-utb.github.io/seminario/papers/Pattern_Recognition_Pertuz_2013.pdf),
[OpenCV comparative study](https://opencv.org/blog/autofocus-using-opencv-a-comparative-study-of-focus-measures-for-sharpness-assessment/),
[MDPI evaluation](https://www.mdpi.com/1424-8220/25/10/3144)). Compute
*both* during calibration and keep whichever best predicts detection
success on our data — pick the metric empirically too.

### Calibration run (offline, safe — the data generator)
Routine: park the platform at a ladder of Z across the working range; at
each Z, **sweep `focus_pos` 0→255** and record the sharpness metric on the
ball ROI at each step. Outputs:
- the **focus curve** (sharpness vs `focus_pos`) per Z → peak = best focus;
- `config/z_focus_map.yaml` built from the *measured* peaks;
- the curves tell you the **depth of field** (peak width) and **whether
  one fixed focus + DoF already covers the whole Z range** (broad,
  overlapping curves) or how far focus must actually move with Z.

This run is pure measurement — it cannot degrade live operation.

### Online control (gated, with fallback)
- **Baseline (ship first):** `focus_pos = interp(Z)` from the calibrated
  map, set deterministically when Z changes *and the platform is settled*.
  Anchored to measured peaks → strictly better than continuous-AF hunting
  (which pauses the sensor, the documented FPS killer).
- **Monitor (observe-only):** compute the ball-ROI sharpness each frame;
  compare to the calibration's expected value for the current Z. Below
  expectation ⇒ flag in the GUI + log (drift / temperature / needs
  recal). No action yet.
- **Bounded correction (flag, last):** only when settled + ball present,
  allow a **±few-step** local refocus to re-peak the metric — never a
  full-range hunt mid-demo. Fall back to the map if the metric is noisy.

**Why it can't make focus worse:** default is the measured map (better
than today); feedback only nudges within a tiny bound when safe; last-good
is always retained; the whole feedback layer is behind a flag
(default-off = map only).

---

## Exposure

### Metric (what "good exposure" means here)
Not scene brightness — **safe ball detection**. SOTA exposure-for-vision
maximizes a gradient/feature metric rather than average brightness
([Shim gradient AE](https://joonyoung-cv.github.io/assets/paper/14_iros_auto_adjusting.pdf),
[Zhang active exposure, ICRA'17](https://rpg.ifi.uzh.ch/docs/ICRA17_Zhang.pdf),
[noise-aware IQA](https://ar5iv.labs.arxiv.org/html/1907.12646)). For us
the composite metric per setting is:
- **detection success rate** + **detection confidence** (the ultimate test);
- **ball-ROI HSV saturation/value** — must stay above the cv2 HSV gate;
- **histogram clipping** fraction (reject over/under-exposed);
- **ball↔background contrast** (gradient/Tenengrad on the ROI);
- **motion-blur** = ball_speed × exposure (the hard upper bound).

### The hard trade-off (state it plainly)
**Motion blur sets the MAX exposure** (must be short for a fast ball);
**detection SNR sets the MIN** (must be bright enough). The SmallRig light
is what makes a short-*and*-bright window exist at all. Calibration finds
the **shortest exposure that still detects robustly** — that's the optimum.

### Calibration run (offline, safe)
Sweep `exposure_us` × `iso` (and repeat per Z, and **static vs.
hand-rolled-moving ball**); record every metric above. Outputs:
- detection-rate / confidence / saturation / clipping / contrast / blur
  **curves vs exposure** (and Z);
- the chosen `exposure(Z)` (or fixed) = shortest exposure meeting the
  detection + saturation thresholds under motion;
- raw keyframes at a few settings so the image itself is inspectable.

### Online control (detector-driven, bounded — optional, gated)
A **slow, bounded** auto-exposure that nudges `exposure/iso` to hold ball
confidence/saturation in a target band, with a **hard exposure ceiling**
(the blur limit) and last-good fallback. This is detector-driven AE, *not*
the camera's scene-average AE (which gave 67 ms → 15 Hz). Default = the
calibrated fixed exposure (no live loop) so worst case is already better
than today's 8 ms. (Gradient-based AE is known to be slow per iteration —
keep it bounded and low-rate so it never fights the controller.)

---

## Data pushed to GitHub (as much as possible, for analysis)

Both calibrations write a session under `tuning_data/<UTC>_focuscal/` and
`_expcal/` (auto-committed + pushed via the existing pattern):
- **Focus:** per-`(Z, focus_pos)` sharpness (both metrics) → `focus_sweep.csv`;
  `z_focus_map.yaml`; focus-curve PNGs per Z; the ROI + metric used.
- **Exposure:** per-`(exposure, iso, Z, static/moving)` detection-rate /
  confidence / saturation / clipping / contrast / blur → `exposure_sweep.csv`;
  chosen-curve PNGs; **9 keyframe JPEGs** (like the vision-debug digest) so
  the image quality is directly visible, not just numbers.
- **Metadata sidecar** (ball, surface, lighting, ambient temp, git SHA).

This is exactly the dataset to hand back for joint analysis — the curves
tell us whether the schema is good, where it's marginal, and whether
fixed-vs-scheduled is even necessary.

---

## How this plugs into the plan & deploy

- **Implements `IMPLEMENTATION_PLAN.md` Step 1a/1b** the empirical way.
  Reuses the runtime setters: `setManualFocus` (already wired, oak_driver
  lines 1696/1871) and the new `/oak/cmd_exposure` (Step 1a).
- **Calibration routines** = new `scripts/` tools + a "Focus/Exposure
  Cal" GUI sub-panel (clone the STEP_ID record→digest→push pattern). The
  sharpness/exposure metrics are cheap host-side ops (Sobel/Laplacian +
  histogram on the ROI).
- **Deploy impact:** all source + new `scripts/*.py` (path-invoked) + new
  `config/*.yaml` (glob-installed, auto-committed) + `index.html` (live).
  **No new ROS node ⇒ no `setup.py` change; no `pi_deploy.sh` change**, as
  long as artifacts land under `tuning_data/` (CSV/PNG/keyframes) or as
  `.yaml` — which they do.
- **No-break:** the calibration is observe-only. The online layers are
  flag-gated, default-off, with last-good fallback — so the controller and
  detector behavior are unchanged until you deliberately enable each layer.

## Suggested build order
1. Sharpness + exposure **metrics module** (host-side, pure functions; unit-testable).
2. **Focus-sweep** routine + digest + push → first GitHub dataset.
3. **Exposure-sweep** routine + digest + push → second dataset.
4. Analyze together → commit `z_focus_map.yaml` + `exposure(Z)`.
5. Ship **open-loop maps**; add **monitors** (observe-only); review logs.
6. Only then flag-enable **bounded** closed-loop focus / exposure.

## Sources
- [Pertuz et al., Analysis of focus measure operators in shape-from-focus (2013)](http://isp-utb.github.io/seminario/papers/Pattern_Recognition_Pertuz_2013.pdf)
- [Autofocus using OpenCV — comparative study of focus measures](https://opencv.org/blog/autofocus-using-opencv-a-comparative-study-of-focus-measures-for-sharpness-assessment/)
- [Quantitative evaluation of focus measure operators (MDPI Sensors, 2025)](https://www.mdpi.com/1424-8220/25/10/3144)
- [Shim et al., Gradient-based auto-adjusting camera exposure (IROS'14)](https://joonyoung-cv.github.io/assets/paper/14_iros_auto_adjusting.pdf)
- [Zhang et al., Active Exposure Control for Robust Visual Odometry in HDR (ICRA'17)](https://rpg.ifi.uzh.ch/docs/ICRA17_Zhang.pdf)
- [Camera Exposure Control with Noise-Aware Image Quality Assessment](https://ar5iv.labs.arxiv.org/html/1907.12646)
- [Efficient Camera Exposure Control via Deep RL (2024)](https://arxiv.org/html/2408.17005)
