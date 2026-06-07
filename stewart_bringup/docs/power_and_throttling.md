# Pi 5 power delivery & CPU throttling

## ⚠️ The BEC-back-feed trap (2026-06-07)

**Symptom:** the Pi 5 was under-voltage throttling at *moderate* CPU load
(`load ≤ 10`, `temp ≈ 55 °C` — so NOT thermal, NOT load). The ARM clock dropped
to ~2.3 GHz right before each throttle, and `vcgencmd get_throttled` showed
`0x50000` (under-voltage + throttling *have occurred*). `pmic_read_adc EXT5V_V`
read **4.88 V** — already below the nominal 5.0–5.1 V even at idle.

**Cause:** a **BEC** (battery-eliminator-circuit 5 V regulator) was wired to the
Pi's **5 V rail in parallel with the USB‑C PSU**. Two unmanaged 5 V sources on
one rail fight: whichever sits slightly lower becomes a *sink*. The BEC, at a
hair below the PSU, **pulled current / back-fed instead of contributing**,
dragging the whole rail down to 4.88 V. Under the vision-pipeline + control-loop
transient current spikes, that sagged past the Pi 5's ~4.6–4.8 V under-voltage
threshold and the firmware capped the ARM clock to survive — the "clock drops
right before throttling" tell.

**Fix:** unplug the BEC. Immediately: `EXT5V_V = 5.03 V`, `get_throttled = 0x0`,
clock pinned at 2.4 GHz, **zero throttling.**

**Rule:** never parallel the BEC and the USB‑C PSU on the Pi's 5 V rail. Power
the Pi from **one** path. If both must feed it (e.g. battery fallback), use
proper power-ORing (ideal-diode / Schottky OR-ing controller), not a bare
parallel connection. The official **27 W (5 V/5 A)** USB‑C supply plugged
straight into the Pi's USB‑C port is the known-good path here.

## Diagnosing throttling

Run while the symptom is happening:

```bash
watch -n0.5 'vcgencmd measure_clock arm; vcgencmd measure_temp; \
             vcgencmd get_throttled; vcgencmd pmic_read_adc EXT5V_V'
```

`get_throttled` hex flags (OR-ed together):

| bit | now | since boot | meaning |
|-----|-----|-----------|---------|
| 0 | `0x1` | `0x10000` | under-voltage |
| 1 | `0x2` | `0x20000` | ARM frequency capped |
| 2 | `0x4` | `0x40000` | currently throttled |
| 3 | `0x8` | `0x80000` | soft temperature limit |

- under-voltage bits (`0x1`/`0x10000`) + a healthy `temp` ⇒ **power**, not heat.
  Check `EXT5V_V`: < ~4.8 V under load = a sagging rail (cable, PSU, or — see
  above — a parallel source fighting it).
- soft-temp bits (`0x8`/`0x80000`) + `temp` ≈ 80–85 °C ⇒ **cooling**.

## Keeping the clock pinned (max performance, no down-clock)

The Pi 5's default governor scales the clock down when idle and ramps it back up
on demand — that ramp adds a few ms of latency on a load spike, which matters for
a real-time control loop. To pin all cores at the stock 2.4 GHz (we don't care
about idle power draw here):

```bash
# one-shot (resets on reboot):
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
# verify:
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # -> performance
vcgencmd measure_clock arm                                  # -> ~2.4 GHz, steady
```

Persist it across reboots with the bundled service:

```bash
sudo cp scripts/cpu_performance.service /etc/systemd/system/
sudo systemctl enable --now cpu-performance.service
```

This is *not* an overclock — it just holds the stock max instead of letting DVFS
idle down. `force_turbo=1` in `config.txt` would also do it but sets the warranty
bit and disables thermal DVFS, so the governor route is preferred.
