# Implementation Plan — exposure sweep → data collection → model-based predictor

**Added 2026-06-04.** Self-contained execution plan so a fresh Claude
Code instance (WSL, SSH-ing to the Pi) can pick up with no prior context.
Builds on `ball_physics_modeling_plan.md` (the *what/why*) and
`oak_highspeed_detection_analysis.md` (the camera analysis). This doc is
the *how*, with exact files, substeps, downstream-break analysis, and
deploy impact for each.

Read order for the WSL instance: this doc → `ball_physics_modeling_plan.md`
§12–20 → `oak_highspeed_detection_analysis.md`.

---

## 0. How the machinery works (so the plan can reuse it)

### Deploy (`stewart_bringup/scripts/pi_deploy.sh`) — git-pull based
1. Auto-commits only operator-tuned artifacts matching
   `SAFE_RE = \.(yaml|yml)$|^tuning_data/|^stewart_vision/blobs/`; **stops
   if any other source file is dirty** on the Pi.
2. `git pull --rebase` from GitHub → 3. `git push` → 4. copy `.service`
   files + `daemon-reload` → 5. source ROS overlay → 6. `colcon build
   --symlink-install --packages-select stewart_vision stewart_bringup
   jugglebot_interfaces` → 7. restart both services → 8. verify the live
   git SHA in `journalctl`.

**Implication:** edits reach the Pi *through GitHub*. Commit + push from
dev, then run `pi_deploy.sh` on the Pi. Uncommitted source edits on the
Pi block the deploy — so never hand-edit `.py` on the Pi.

### Build / packaging (`setup.py`, both packages)
- `data_files` use **globs**: `config/*.yaml`, `launch/*.py`,
  `web/*.html`, `blobs/*.blob` — so **new config/launch/web files install
  automatically, no `setup.py` edit.**
- `console_scripts` register **nodes**. A *new ROS node* needs an
  entry here; a new *module* (plain import) or a new *script* does not.
- `scripts/*.py` (digests, `gui_server.py`, fitters) are **invoked by
  repo path**, not colcon-installed — new scripts need no `setup.py` edit.

### GUI (`scripts/gui_server.py` + `web/index.html`)
Two command channels, both already used by STEP_ID and the demo recorder:
- **ROS (roslib/rosbridge):** services like `/step_id/run_stiction`, and
  `publishCmd({...})` → `/control_cmd` String bus. Used for *actions*.
- **HTTP (gui_server `do_GET`/`do_POST`):** `/X/bags` (list),
  `/X/bags/digest` (run `digest_X_bag.py` via subprocess), `/X/bags/png`
  (serve plot), `/X/bags/push` (git add artifacts + commit + push). Used
  for *bag management*.

The per-pipeline pattern to **clone** (functions in `gui_server.py`):
`_list_*_sessions` · `_digest_*_bag` (→ subprocess to `scripts/digest_*.py`)
· `_push_*_to_git` (artifacts only — bags stay gitignored). Frontend: a
panel of `<button>`s wired with `$('id').addEventListener('click', …)`
that either `publishCmd(...)` or `fetch('/X/bags/...')`.

### Bag consumers are safe to extend
Every digest reads selected topics and **skips unknown ones**
(`cls = _TYPE_CLASS.get(type); if cls is None: continue`). Adding bag
topics cannot break them (verified, `ball_physics_modeling_plan.md` §19).

### Substep 0 — line-ending guard (do once, first)
Add to `.gitattributes` so `.sh`/`.service` can never get CRLF from a
Windows edit:
```
*.sh      text eol=lf
*.service text eol=lf
*.py      text eol=lf
```
Low-risk, protective; apply deliberately (it may renormalize on next
touch). Do this from whichever environment, commit, push.

---

## Step 1 — Exposure / focus, to cut blur + latency

**Goal:** spend the SmallRig light on a **short exposure** (kill motion
blur → recover detection rate → lower effective latency), and keep focus
sharp across the platform Z range. See `oak_highspeed_detection_analysis.md`.

> **Do this empirically, not theoretically.** Focus and exposure here are
> chosen from *measured* sharpness / detection curves on this rig, with
> image-clarity feedback and a staged rollout that can't regress past the
> current behavior. The full protocol (focus metric = Tenengrad /
> variance-of-Laplacian; detector-driven exposure; offline-sweep →
> open-loop map → observe-only monitor → bounded closed-loop) is in
> [`oak_focus_exposure_autocal.md`](oak_focus_exposure_autocal.md). The
> sub-steps below are the wiring; that doc is the method + safety design.

### 1a. Runtime exposure control + GUI sweep
- **Code:** `oak_driver_node.py` — add an `/oak/cmd_exposure`
  subscriber (`Float32MultiArray [exp_us, iso]`) that pushes a
  `CameraControl().setManualExposure(exp_us, iso)` onto the camera input
  queue. **Mirror the existing `/oak/cmd_focus` / `/oak/cmd_hsv` runtime
  setters** (lines ~1696/1871) — same pattern, new control. Update
  `_publish_config_snapshot` to report the live values.
- **GUI:** add an "Exposure" sub-panel in `web/index.html` (two sliders +
  a "Sweep" button) next to the existing HSV sliders; sliders
  `publishCmd`/publish to `/oak/cmd_exposure`; "Sweep" steps
  `exp_us ∈ {4000,2000,1000,500}` (ISO compensating) and reads back
  `/oak/health` `v0_pub_hz` + `/oak/latency_ms` live.
- **Downstream/upstream:** purely additive — detection consumes whatever
  frames arrive; no consumer cares how exposure was set. The boot-time
  `OAK_EXP_US` path is untouched (runtime override only).
- **Deploy:** edit `oak_driver_node.py` (source → pull + colcon rebuild)
  + `index.html` (served live from the checkout). **No `setup.py`, no
  `.service`, no deploy-script change.**
- **Validate:** the sweep itself is the validation — watch `v0_pub_hz`
  hold up under hand-rolled motion as exposure drops. This is the cheap
  experiment that de-risks everything else.

### 1b. Z-scheduled focus (replaces the unworkable single lock)
- **Code:** publish the controller's *commanded Z* (already in `/status`
  JSON; add a typed `/platform/commanded_z` Float32 if cleaner) and have
  `oak_driver_node` subscribe + map `Z → focus_pos` via a small
  interpolation table, calling the existing `setManualFocus`.
- **Calibration:** new `config/z_focus_map.yaml` (4–5 `(z_mm, focus_pos)`
  points). A quick operator routine: set Z, one-shot autofocus, read
  `focus_pos`, record. (Optional GUI helper button.)
- **Downstream/upstream:** additive + backward-compatible — if the Z
  topic/map is absent, falls back to current manual/auto behavior. The
  controller already computes Z; only a publish is added.
- **Deploy:** `oak_driver_node.py` + maybe `stewart_control_node.py`
  (publish Z) → source/rebuild; new `z_focus_map.yaml` → installed by the
  `config/*.yaml` glob **and** auto-committed by `pi_deploy.sh` (matches
  `SAFE_RE`). **No `setup.py` / deploy-script change.**

### 1c. (Defer) on-device color detection — the USB-raw cap
Lifting `v0_arr` past ~27 Hz means not shipping raw frames (on-device
HSV/blob, the `oak_phase2b_on_device_v0.md` plan). Bigger sub-project;
schedule after 1a/1b prove the blur fix. No deploy-script change (blob via
`blobs/*.blob` glob, already wired).

**Step-1 deploy summary:** all source + config edits; git-pull + colcon
handles it; **deploy script unchanged.**

---

## Step 2 — Robust logging + data collection

**Goal:** record everything the fit needs (and that flags bad runs), then
run the §5 campaigns from `ball_physics_modeling_plan.md`.

### 2a. Expand the auto-bag allowlist (+ health during BALL_TRACK)
- **Code:** `bag_recorder_node.TOPICS_DEFAULT` (lines 57–81) — add
  `/oak/latency_ms`, `/oak/health`, `/ball_track/diagnostic`,
  `/level_diag`, `/leg_currents`, the per-motor `RobotState` topic, and
  the CAN-traffic topic. `stewart_control_node` — keep `/level_diag` (or
  a slim `/motor_health`) publishing **during BALL_TRACK** (it's
  level-loop-only today), so closed-loop runs capture per-leg
  `axis_state`/`feeder_mode`/`active_errors`/temps.
- **Downstream/upstream:** verified safe — digests skip unknown topics,
  and `digest_demo_bag.py` *already* parses latency/health/diag/currents
  (lines 198–201). `gui_server.DEMO_TOPICS` already records most of these,
  so this just aligns the auto-recorder. Larger bags (still no images).
- **Deploy:** two source edits → pull + rebuild. **No `setup.py` /
  deploy-script change.**

### 2b. Per-bag metadata sidecar
- **Code:** `bag_recorder_node` (or `gui_server` on record-start) writes
  `tuning_data/<session>/metadata.yaml` (ball type/mass, surface, Z,
  lighting, exposure/ISO, vision backend, ambient temp, git SHA).
- **Deploy:** writes a `.yaml` under `tuning_data/` → auto-committed by
  `pi_deploy.sh`. ✅ no change.

### 2c. Campaign modes + Data Collection panel (clone STEP_ID)
- **Code:**
  - `stewart_control_node` — add campaign drivers mirroring
    `_do_generate_rolling_ball`: breakaway ramp (rate + dwell sweep),
    coast-down (kick then level), **chirp/PRBS tilt** excitation, ArUco
    coverage-grid goto. Trigger via `/control_cmd` `data_collect:<campaign>:<json>`
    or new ROS services like the `/step_id/run_*` set.
  - `bag_recorder_node` — start recording on the new `DATACOLLECT_*`
    modes (extend the trigger set; same expanded allowlist).
  - `gui_server.py` — clone `_digest_demo_bag`/`_push_demo_bag_to_git` →
    `_digest_datacollect_bag`/`_push_datacollect_to_git`, and add
    `/datacollect/bags{,/digest,/png,/push,/delete}` routes.
  - `scripts/digest_datacollect_bag.py` (new) + `scripts/coverage_map.py`
    (new) — invoked by path; produce the §17/§18 plots (coverage map,
    per-bag QA timeline, residual/horizon plots later).
  - `web/index.html` — new "Data Collection" panel + live coverage map +
    run-QA strip (clone the STEP_ID panel markup + listeners).
- **Downstream/upstream:** all additive. New modes are new strings on an
  existing bus; new endpoints/scripts/panels don't touch existing ones.
  No existing digest/recorder behavior changes.
- **Deploy:** source edits (control node, recorder, gui_server) +
  new scripts (path-invoked) + new config yamls (glob-installed,
  auto-committed) + `index.html` (live). **No new ROS node ⇒ no
  `setup.py` change.** **Watch-item:** keep every generated artifact
  under `tuning_data/` or as `.yaml` — anything else (`.csv/.npz/.json`)
  written *elsewhere* won't match `SAFE_RE` and will **block the deploy**.
  Mitigation: write under `tuning_data/` (preferred) **or** extend
  `SAFE_RE` in `pi_deploy.sh` — the one place the deploy script might need
  a one-line change.

### 2d. Auto-QA in the digest
- **Code:** in `digest_datacollect_bag.py`, flag a run unusable on
  `axis_state≠8`, unexpected `feeder_mode`, `active_errors≠0`, thermal/
  voltage limits, `/oak/health` starvation, or CAN saturation; tag and
  exclude (reuse the existing `_fault` suffix convention).
- **Deploy:** new script only. ✅ no change.

---

## Step 3 — Fit the model, swap the look-forward

**Goal:** replace the constant-velocity look-forward in `_ball_track_run`
with the data-fit forward model, behind a flag. The look-forward is
**shared by all BALL_TRACK modes** (GOTO / TRAJECTORY / PATH) — so this
improves Demo 2 *and* Demo 3; validate on whichever you run.

### 3a. Offline fit script
- **Code:** `scripts/fit_ball_forward_model.py` (new) — ingest
  data-collection bags (reuse the rosbag2 reader + `smooth_demo_bag.py`
  non-causal smoother), fit the gray-box params (§3 of the modeling plan),
  write `config/ball_forward_model.yaml` (params + covariance + valid
  ranges + fit conditions).
- **Deploy:** path-invoked script + a `.yaml` output → auto-installed and
  auto-committed. ✅ no change.

### 3b. Predictor module
- **Code:** `stewart_bringup/stewart_bringup/ball_forward_model.py` (new
  *module*, not a node) — loads `ball_forward_model.yaml`, exposes
  `predict(state, tilt_history, Td)` (forward-integrate the fitted ODE
  over the commanded-tilt window).
- **Deploy:** lives inside the package → importable after colcon rebuild.
  **No `setup.py` entry** (module, not console_script). ✅ no change.

### 3c. Swap behind a flag (+ per-sample latency)
- **Code:** `stewart_control_node._ball_track_run` — replace the two lead
  lines (PID branch `ex_lead = ex + edot_x*control_latency_s` ≈ L4576;
  bang-bang `px_lead = px + vx*latency_s` ≈ L4496) with a call to
  `ball_forward_model.predict(...)`, gated by
  `g.get('use_model_predictor', False)` from `ball_track_gains.yaml`
  (**mirror the existing `use_empirical_ik` toggle**). Keep a short ring
  buffer of issued tilts (new local state) for the integration. Subscribe
  to `/oak/latency_ms` and feed the **measured per-sample latency** as
  `Td` instead of the fixed `control_latency_s`.
- **Downstream/upstream:** flag **defaults OFF → byte-for-byte current
  behavior**; constant-velocity stays as the fallback. The loop's output
  interface (`tilt_pitch`/`tilt_roll` → `_do_set_pose`) is unchanged, so
  nothing downstream of the controller sees a difference. New inputs are
  the model file + the latency topic (both optional — absent ⇒ fall back).
- **Deploy:** `stewart_control_node.py` + new module + a flag in
  `ball_track_gains.yaml` (auto-committed) → pull + rebuild. ✅ no
  deploy-script change. A/B live from the existing gains panel.

### 3d. (Optional, later) EKF motion model in `ball_kf_node`
Fold the model into the KF; tune Q/R. Source edit, no deploy change.

---

## Deploy-script compatibility — bottom line

| Change type | Reaches Pi via | `setup.py`? | `pi_deploy.sh`? |
|---|---|---|---|
| Edit existing node (`oak_driver`, `stewart_control_node`, `bag_recorder`) | git pull + colcon | no | no |
| New **config** `*.yaml` | glob-install + auto-commit | no | no |
| New **script** (digest/fit) | repo path | no | no |
| New **module** in a package | colcon | no | no |
| Edit `web/index.html` | served from checkout | no | no |
| New **ROS node** | colcon | **yes** (console_scripts) | no |
| Artifact written **outside** `tuning_data/` and not `.yaml` | — | no | **yes** (extend `SAFE_RE`) |
| New `.service` / boot node | copied in deploy step 4 | maybe | maybe (`.service` edit) |

**The plan deliberately avoids new nodes and writes all artifacts under
`tuning_data/` or as `.yaml`, so in the common case the deploy script
needs no changes.** The single thing to watch is artifact paths/extensions
(2c) — keep them in `tuning_data/`/`.yaml` and `SAFE_RE` stays untouched.

---

## Sequencing & validation gates

1. **Substep 0** (`.gitattributes`) → 2. **Step 1a exposure sweep**
   (gate: `v0_pub_hz` stays high under motion at short exposure) →
   3. **Step 1b Z-focus** (gate: sharp across Z) → 4. **Step 2a/2b
   logging** (gate: a demo bag now contains latency/health/diag/currents/
   level_diag + metadata) → 5. **Step 2c/2d campaigns** (gate: coverage
   map fills, QA rejects bad runs) → 6. **Step 3a/3b fit** (gate:
   held-out horizon-error beats constant-velocity, §17) → 7. **Step 3c
   swap behind flag** (gate: A/B on Demo 2/3 shows tighter settle; flag
   OFF reproduces today exactly).

Each gate is a digest you can eyeball before moving on. Nothing is
irreversible: every behavioral change is flag-gated or additive.

---

## Handoff checklist for the WSL instance
- [ ] `git pull` (this plan + all `*_lessons`/analysis docs are in `stewart_bringup/docs/`).
- [ ] Apply substep 0 (`.gitattributes`), commit, push.
- [ ] Work top-down through the gates; commit + push each change, then
      `pi_deploy.sh` on the Pi and watch `journalctl -u stable_bot.service`.
- [ ] Keep new artifacts under `tuning_data/` or `.yaml`; only touch
      `pi_deploy.sh`'s `SAFE_RE` if you can't.
- [ ] No new ROS nodes unless you also add the `setup.py` console_script.
