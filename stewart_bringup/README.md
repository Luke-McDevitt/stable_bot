# Stable-Bot Stewart Platform

Browser-controlled 6-DOF Stewart platform with dual-IMU closed-loop
leveling, keyframe routine playback, and data logging. Built on ROS 2
Kilted, six ODrive Pro motor controllers over CAN, and two Xsens MTi-630
IMUs (base + platform).

**Live pilot UI**: `http://localhost:8080/` once the stack is up.

---

## What's in this package

- `stewart_bringup/stewart_control_node.py` — the one long-lived ROS 2
  node. Owns the CAN bus via a per-leg-mode `ODriveFeeder` (50 Hz
  `Set_Input_Pos` with velocity feed-forward), passive + RTR
  `EncoderListener` for encoders / errors / Iq, PI level loop, homing
  subprocess orchestrator, routine playback + data logging. All GUI
  actions reach it via a `/control_cmd` topic dispatcher (see
  [`ARCHITECTURE.md`](docs/ARCHITECTURE.md)).
- `web/index.html` — single-file web GUI, roslibjs + Tailwind via CDN.
- `launch/stewart_gui_launch.py` — bundled launch: dual Xsens IMUs +
  control node + rosbridge on `ws://localhost:9090`.
- `scripts/gui_server.py` — tiny HTTP server that serves the HTML **and**
  exposes `/reset`, `/launch`, `/stop_launch`, `/launch_status`
  endpoints so the browser can recover the backend when rosbridge is
  unreachable.
- `scripts/reset_stewart_stack.py` — one-shot recovery: kills stale
  processes, frees port 9090, restarts the ros2 daemon, optionally
  relaunches + polls `/status` to verify the stack is actually
  publishing.
- `scripts/analyze_routine_log.py` — offline analyzer for recorded
  routines; plots commanded vs measured tilt, leg currents, and runs a
  rolling-sphere ball-on-disk simulation against the measured IMU data.
- `scripts/read_odrive_errors.py` — RTR query of every ODrive's
  `active_errors` + `disarm_reason` with human-readable decode.

---

## Hardware inventory

| Item | Interface | Role |
|---|---|---|
| 6 × ODrive Pro (firmware 0.6.x) | CAN (gs_usb adapter) | leg motor drives |
| 2 × Xsens MTi-630 | USB-serial at 921600 baud | base + platform IMUs |
| 6 × 3D-printed linear actuators (~71 mm / motor rev) | — | legs |
| Stewart top plate (carbon fiber, ~400 mm Ø) | — | platform surface |

---

## Quick start (WSL2 laptop, one double-click)

If USB-CAN and both IMUs are already `usbipd attach`-ed, the fastest
path from desktop to running GUI is the Windows launcher:

**Double-click `scripts/start_stable_bot.bat`** — brings up can0, starts
`gui_server.py` on :8080, runs `ros2 launch`, opens the browser. Two
WSL windows stay open so you can watch logs; close them to shut down.

Companion launchers in the same folder:
- `scripts/stop_stable_bot.bat` — clean teardown (kills every Stable-Bot
  process)
- `scripts/reset_stable_bot.bat` — one-click hard reset + relaunch +
  verify (wraps `reset_stewart_stack.py --launch --verify`)

### Manual start (three terminals)

If you prefer terminals or need finer control, three WSL panes sourced
to ROS 2 Kilted + workspace overlay:

```bash
source /opt/ros/kilted/setup.bash
source ~/ros2_ws/install/local_setup.bash
```

**Terminal 1** — CAN bus up (needs sudo once per session):
```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
```

**Terminal 2** — HTTP server serving the GUI + recovery endpoints:
```bash
python3 ~/ros2_ws/src/stewart_bringup/scripts/gui_server.py
```

**Terminal 3** — The ROS 2 stack:
```bash
ros2 launch stewart_bringup stewart_gui_launch.py
```

Open `http://localhost:8080/` in a browser.

---

## Quick start (Pi — headless, auto-boot)

See [`docs/PI_MIGRATION.md`](docs/PI_MIGRATION.md) — includes headless
WiFi setup (phone hotspot), ROS 2 install, systemd services, and how to
control the robot from your laptop browser over WiFi.

---

## Typical session flow

1. Open GUI. Wait for the Status panel to populate (≤ 2 s).
2. Click **Read now** in the ODrive errors panel. All 6 should read `OK`.
   If any don't, click **Clear all**; if they re-latch, power-cycle.
3. Run the homing section's **Stall-home this leg (bottom)** once per
   leg (or all 6 selected together). Respond `y` in the stdin box when
   prompted.
4. If you need a leveled reference, use the **Stabilization** card:
   Arm → Level ON → drag Z slider to desired height → watch the
   roll/pitch errors settle to green (<0.1°).
5. For rolling-ball or routine demos: record a routine with the **rec**
   checkbox, then analyze it offline with
   `analyze_routine_log.py --all`.

---

## One-command recovery

When anything goes wrong (GUI stuck on "waiting", ODrives red-pulsing,
CAN bus-off, etc.):

```bash
python3 ~/ros2_ws/src/stewart_bringup/scripts/reset_stewart_stack.py --launch --verify
```

This kills everything, frees port 9090, restarts the ROS daemon,
relaunches the bundle, and polls `/status` to confirm it actually came
up. Full diagnostics in the terminal; full log saved to
`logs/last_launch.log`.

For deeper trouble, see [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

---

## Data output

Recorded routine runs land in `~/ros2_ws/src/stewart_bringup/logs/<timestamp>_<routine>/`:

- `routine.json` — exact copy of the routine played
- `metadata.json` — motor limits, level cal, PI gains, feature flags
- `telemetry.csv` — 50 Hz: commanded pose, IMU RPY/accel/gyro, encoder
  positions, Iq, level corrections, per-leg modes
- `can.log` — raw `candump -L` capture
- `end.json` — final duration + timestamp

Bulk-compare all logs:
```bash
python3 ~/ros2_ws/src/stewart_bringup/scripts/analyze_routine_log.py --all
```
Writes `logs/summary.csv` with one row per run + ball-trajectory metrics.

---

## Further reading

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — process graph, topic flow, control algorithms
- [`docs/PI_MIGRATION.md`](docs/PI_MIGRATION.md) — Phase 8: move backend to a Raspberry Pi
- [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) — consolidated fixes for all the issues seen during bring-up
