# Pi 5 power throttling — investigation + fixes

**Added 2026-06-05.** During the latency-instrumentation work the demo/bench
digests showed the Pi **pegged at ~100% CPU, load ~26 (4 cores!), and
`throttled_now=True`** at only ~58 °C. That contaminates every latency and
bench number (the detect→state split swings, the bench gain reads garbage
from a starved excitation thread). This doc records what's going on, how to
diagnose it, and what to do — so we don't re-derive it.

The host stats are now captured automatically: gui_server's `/system/stats`
(GUI banner, red = throttling) and `<bag>/system_stats.jsonl` →
`digest.summary.json.host`.

## TL;DR

1. **It's under-voltage, not heat** (58 °C is cool; thermal limit is ~80–85 °C).
   The SoC firmware itself raised the flag because the 5 V rail sagged under
   load.
2. **The Pi 5 needs a 5 V/5 A (25 W) USB-PD supply.** High-wattage GaN
   chargers do NOT count — they deliver their big wattage at 20 V/28 V, and
   only **5 V/3 A (15 W)** at 5 V. A Pi 5 under full load wants up to ~5 A at
   5 V, so 15 W brown-outs → throttle.
3. **Order: the official Raspberry Pi 5 27 W PSU** (5.1 V/5 A). Likely the
   single fix. An **Active Cooler** is cheap insurance once it holds full
   clock. An **NVMe SSD + HAT** only if the bag recorder turns out I/O-bound.
4. **2.4 GHz is the Pi 5 stock max.** Idle it scales down to ~1.5 GHz (normal
   power saving, NOT throttling) — so always read `measure_clock arm` *under
   load*.

## Why our chargers fail (the counterintuitive part)

The Pi takes a fixed **5 V** input. What matters is amps **at 5 V**, not the
sticker wattage.

| Supply / port | Amps at 5 V | Verdict for Pi 5 |
|---|---|---|
| Official Pi 5 PSU | **5 A (25 W)** | ✅ what it's designed for |
| Anker 33 W | 3 A (15 W) | ❌ marginal → brown-out under load |
| Anker 140 W **A2697**, port C1/C2 | 3 A (15 W) — 5 A only at 20/28 V | ❌ |
| Anker 140 W **A2697**, port **C3** (used) | 3 A (15 W), 40 W port | ❌ |

So "140 W" is irrelevant here — every port on the A2697 caps at **5 V/3 A**.
The 5 A profiles are at 20 V/28 V, which the Pi can't use. (Per-port specs:
Anker A2697 user guide.) Long/thin cables make it worse via IR drop; the
official PSU has a short captive cable for exactly this reason.

The OAK being on a separate rail (Castle BEC Pro → hub) is correct and helps
— it keeps the OAK's ~1.2 A off the Pi's 5 V — but it does nothing about what
the **Pi's own** brick can deliver.

## The cascade that produces "load 26"

Under-voltage doesn't just slow one thing — it snowballs:

```
inadequate 5 V rail  →  firmware caps the ARM clock (throttle)
   →  the normal ROS2 + cv2 workload no longer fits the reduced CPU
   →  CPU pins at 100%, runnable threads pile up  →  load average climbs to ~26
```

So load 26 is a *symptom*. Fixing the rail (full 2.4 GHz restored) should
drop it a lot on its own. If it DOESN'T, the remaining load is genuine
compute (see below).

## Diagnosis toolkit (run on the Pi)

Power / clock (no sudo needed for vcgencmd):
```bash
vcgencmd get_throttled        # 0x0 = healthy; bit 0x1 = under-volt NOW; 0x10000 = since boot; 0x4 = throttled NOW
vcgencmd measure_clock arm    # UNDER LOAD: should hit ~2.4e9; if it sags, throttling
vcgencmd measure_volts core   # the actual core voltage
sudo dmesg | grep -i volt     # firmware logs "Undervoltage detected!" (needs sudo — dmesg_restrict)
```

What's eating CPU / IO (CPU-bound vs IO-bound):
```bash
htop                 # press P = sort by CPU; the node processes (python3 stewart_control_node, oak driver, rosbridge…)
top -H               # per-THREAD (find a single pegged thread inside a node)
iotop -ao            # accumulated disk I/O per process — confirms if `ros2 bag record` is the writer
iostat -x 1          # %util near 100% on the SD card = disk saturated
uptime               # load average — compare idle vs during a recording run
```
Rule of thumb: **high load + high %CPU = compute-bound**; **high load + high
%wa (iowait) + modest %CPU = I/O-bound** (the SD card / bag writer). Our runs
showed ~100% CPU → primarily compute (made worse by the throttle).

## Fixes, in order

1. **Official Pi 5 27 W PSU (5.1 V/5 A)** — ~$12. The actual fix for the
   under-voltage. Keep the OAK on the BEC.
2. **Pi 5 Active Cooler** — ~$5. Once the clock holds 2.4 GHz under sustained
   load the SoC runs hotter; active cooling keeps it off the thermal limit
   and lets `arm_boost` stay on. Cheap insurance.
3. **Cut compute** (if CPU is still pegged WITH a good PSU):
   - **Minimal-topic recording** for latency runs — drop the 400 Hz pixel/
     diagnostic streams; bag only `/ball_state`, `/ball_xy_mono/lat`,
     `/platform/imu/data`, `/latency_bench/diag`, `/status`. (We can add this
     toggle to the recorder.)
   - **mcap `fastwrite` preset** for `ros2 bag record` — disables CRC +
     indexing for lowest resource use on constrained machines.
   - Lower `OAK_JPEG_FPS` / RGB fps if the host cv2 path is hot.
4. **NVMe SSD + Pi 5 M.2 HAT** — ~$30–50. Only if `iotop`/`iostat` show the
   recorder is I/O-bound on the SD card. Massive win for high-rate logging if
   so, but verify first.

## Notes specific to this rig
- `usb_max_current_enable=1` is set in `config.txt` (for the OAK's USB draw on
  the old single-supply setup). With the OAK now external this is harmless,
  but it means the Pi won't *self-limit* on a weak PSU — it'll draw until the
  rail sags, which is exactly the under-voltage we see.
- Don't bother **overclocking** past 2.4 GHz while fighting under-voltage —
  it raises voltage/heat/draw, the opposite of what we need. The goal is to
  reliably *reach* 2.4 GHz under load, not exceed it.

## Profiling the compute (the Pi held 2.4 GHz → compute-bound, not throttled)

Find the hog in increasing resolution:
1. `htop` (press **P** = sort by CPU). Expect: OAK driver (cv2), rosbridge,
   `ros2 bag record`, control node.
2. **py-spy** on the Python nodes — attaches live, no restart:
   ```bash
   sudo py-spy top  --pid $(pgrep -f oak_driver_node)        # live which-function
   sudo py-spy dump --pid $(pgrep -f stewart_control_node)   # call stack / thread
   sudo py-spy record -o oak.svg --pid <pid> --duration 20   # flame graph
   ```
   Pinpoints e.g. `getCvFrame` vs HSV vs projection vs publish.
3. `sudo perf top` — system-wide incl. C (Fast-DDS, cv2 internals, rosbridge).
4. The digest `host` block now tags **compute-bound** vs **I/O-BOUND (disk)**
   via the iowait split — read that first.

## Increase throughput WITHOUT adding latency

Latency = the see→move *critical path* (capture→detect→localize→KF→control→
actuate). Much of "load 26" is NOT on that path — it steals CPU from it:
rosbridge serialising to the browser, `ros2 bag record` serialising 400 Hz
IMU, the GUI live feed. **Cutting that bookkeeping frees the critical path →
lower latency** (live-video off, fewer/throttled browser subs, minimal-topic
recording, mcap `fastwrite`).

The bigger lever: **offload vision to the OAK's own VPU.** DepthAI "does the
heavy lifting… leaving the host CPU free for business logic"; the host
`getCvFrame()` copies the full ~1.5 MB frame each time. Detecting on-device
and shipping only (cx,cy) deletes that copy + the host HSV — the Pi then only
runs the light control. (Scoped in `oak_phase2b_on_device_v0.md`; caveat: the
on-device detector must match the host cv2's motion robustness — the earlier
YOLO attempt didn't, so this needs detector work, not just a flag.)

## Should we switch off the Pi 5? — profile + offload first

Load 26 at *full clock* for a control + camera task is a software-utilisation
smell, not a hardware ceiling; a clean version runs at load 2–4. The OAK
already has a VPU, so **we don't need a host GPU for the vision.**

If it still can't keep up after offloading:

| Option | vs Pi 5 | Why / why not | Effort |
|---|---|---|---|
| **Intel N100 mini-PC** (~$150) | ~3× CPU | x86, ROS 2 **binary** install (no 3 h source build), big headroom. No GPIO → CAN via USB adapter (SocketCAN). Best $/CPU. | port workspace + USB-CAN + re-test (~days) |
| **Jetson Orin Nano** (~$249) | ≈CPU, +CUDA/TensorRT (100+ FPS YOLO vs 5–15 on Pi) | GPU is **redundant here** (the OAK does vision); only worth it for heavy *on-host* NNs later. 25 W. | JetPack/Isaac + port (~days+) |
| **Stay on Pi 5** | — | Likely fine after offload + cutting bookkeeping; CAN HAT already wired. | software only |

**Verdict: don't switch yet.** Profile → offload vision to the OAK → cut the
browser/recorder load → re-measure. A switch is a multi-day port (ROS 2 +
depthai + CAN + re-test) and probably unnecessary.

## Sources
- Raspberry Pi 5 PSU / under-voltage: pimylifeup, peppe8o, RPi forums
  (t=381080 "5V/5A power issues", t=371450, t=382143).
- Clock / throttle monitoring: Jeff Geerling (overclocking the Pi 5),
  jamesachambers (measure undervoltage + true clock), Tom's Hardware
  (vcgencmd benchmark).
- I/O-wait diagnosis: Netdata, Site24x7, Alibaba ECS (iostat+iotop).
- rosbag2 / mcap on constrained machines: ros2/rosbag2#1787, mcap.dev
  storage-plugin benchmarks (fastwrite preset).
- Profiling: benfred/py-spy (sampling profiler, attach by PID);
  awesomebytes/python_profiling_tutorial_with_ros.
- Compute offload: Luxonis DepthAI docs ("device does the heavy lifting"),
  LearnOpenCV intro to OAK-D/DepthAI.
- Hardware: Pi 5 vs Jetson Orin Nano (kunalganglani, ThinkRobotics,
  hackster); Pi 5 vs Intel N100 (CNX Software, Jeff Geerling, Tom's Hardware).
