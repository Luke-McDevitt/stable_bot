# Troubleshooting

Consolidated fixes for issues encountered during bring-up. Organized
from most-common to least-common. The one-command fix at the top is the
right answer about 70% of the time.

---

## The one command

```bash
python3 ~/ros2_ws/src/stewart_bringup/scripts/reset_stewart_stack.py --launch --verify
```

Kills everything, frees port 9090, restarts the ROS daemon, relaunches
the bundle, polls `/status` for 20 s, and prints a verdict. If it says
`/status IS publishing`, you're good; close and reopen the browser tab.

---

## Symptom → fix

### GUI shows "(connected but no /status message received yet…)"

1. Run the one command above.
2. If verify still fails, check the log tail it prints. Usually one of:
   - **"Address already in use" on 9090** → a zombie rosbridge. The
     script's step 0 (`fuser -k 9090/tcp`) fixes this. Re-run.
   - **No `stewart_control_node ready` line** → the control node is
     crashing during init. Run it standalone to see the traceback:
     `ros2 run stewart_bringup stewart_control_node`.
   - **Node is ready but CLI can't see topics** → DDS env mismatch.
     Make sure the launch file does NOT set `ROS_LOCALHOST_ONLY` or
     `ROS_AUTOMATIC_DISCOVERY_RANGE`. (See memory
     `feedback_ros2_discovery_env.md`.)

### ODrives pulsing red

Latched error state. Open the GUI errors panel, click **Read now** to
see which flag:

| Flag | Meaning | Fix |
|---|---|---|
| `WATCHDOG_TIMER_EXPIRED` | host stopped sending commands | Click **Clear all**. Usually caused by killing the node while armed; the feeder-before-arm pattern prevents it. |
| `CURRENT_LIMIT_VIOLATION` | motor wanted more current than cap | Raise current slider in Motor Limits panel. |
| `SPINOUT_DETECTED` | motor stalled / fought mechanical bind | Clear errors, reduce load or current, investigate mechanics. |
| `DC_BUS_OVER_CURRENT` | PSU can't sustain demand | Lower current limit, check PSU sizing. |

If **Clear all** doesn't work (drives don't respond), the CAN bus is
off. Click **Soft-reset CAN bus** — if that times out,
physically unplug + replug the USB-CAN cable.

### CAN "Transmit buffer full"

The kernel TX queue is saturated because sends are failing (ODrives in
error state aren't ACKing). Cycle:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

If that returns `Timer expired`, the gs_usb adapter itself is wedged —
physically unplug USB, wait 3 s, replug, and re-`usbipd attach` if on
WSL2.

### `ros2 node list` hangs

ROS 2 daemon is stale. Either:
- GUI Diagnostics → **Restart ros2 daemon**, or
- Terminal: `ros2 daemon stop && ros2 daemon start`.

### Xsens IMU "No MTi device found"

USB not attached. On WSL2: `usbipd attach --wsl --busid <X-Y>` from
Windows admin PowerShell. On Pi: verify the udev symlinks exist:
`ls /dev/imu_mti630*`. If they don't, check `dmesg | tail` for USB
enumeration errors.

### Rolling ball rolls right off the disk

Open-loop + static friction is fundamentally hard. In order of
effectiveness:

1. **Physically flick the ball** before pressing Play so it's moving
   when the routine starts.
2. Swap to a **smoother surface** (acrylic, glass, polished aluminum,
   or a sheet of paper taped to the carbon fiber).
3. Use a **heavier / more-dense ball** (glass marble, ball bearing).
4. Tune the routine: smaller `radius_m`, `max_tilt_deg`, try the
   `tilt_multiplier` slider.
5. **Give up and use the stabilization demo** — hold level, tilt the
   base by hand, top stays flat. Same platform, more impressive
   showcase, works every time.

### Browser shows "Can 'Upgrade' only to 'WebSocket'"

You've navigated to `http://localhost:9090` (the rosbridge port).
Navigate to `http://localhost:8080/` instead (the GUI server port).

### Everything seems fine but no motor movement

Drives armed but at the wrong mode. Check:
- GUI **per-leg arming** row: each checkbox should be ticked (green)
  for velocity mode, or use the main **Arm** button for position mode.
- Motor limits slider isn't at minimum.
- For per-leg jog (+ / − buttons), the leg auto-switches to pos mode.
  If it doesn't respond, check the errors panel — might be latched.

### GUI buttons do nothing but /status is flowing

`/control_cmd` messages aren't reaching the control node. Usually a
rosbridge reconnect fixes it:
- GUI Diagnostics → **Reconnect rosbridge**.
- Or: close the browser tab completely, reopen to
  `http://localhost:8080/`.

### Pi: SSH refused / `stablebot.local` doesn't resolve

- Phone hotspot must be ON when the Pi boots, with the SAME SSID and
  password you baked into the SD card. Case-sensitive.
- Give it ~60 s after power-on.
- mDNS (`.local`) may be blocked on phone hotspots — check the
  hotspot's "connected devices" list and SSH to the raw IP instead.

### Pi: launch comes up but IMUs fail with "No MTi device"

udev rules don't know your serial numbers yet. Check:
```bash
udevadm info -a /dev/ttyUSB0 | grep serial
```
Copy the serial(s) into `/etc/udev/rules.d/60-stablebot.rules`,
`sudo udevadm control --reload-rules && sudo udevadm trigger`.

---

## Recovery hierarchy (when to escalate)

1. Close browser tab completely, reopen → fixes ~30% of GUI hangs
2. **Reconnect rosbridge** button → fixes websocket wedges
3. **Restart ros2 daemon** button → fixes CLI hangs
4. **Hard reset stack + (Re)launch stack** buttons → fixes zombie processes
5. `reset_stewart_stack.py --launch --verify` in terminal → the big hammer
6. `reset_stewart_stack.py --nuclear --launch --verify` → cleans stale python3 procs
7. Physical USB cycle + `usbipd attach` (WSL2) or replug (Pi) → last resort
8. Power-cycle the ODrive supply → ODrives won't clear via CAN

If any step before a later one is usable, skip to it. Earlier steps are
less disruptive.
