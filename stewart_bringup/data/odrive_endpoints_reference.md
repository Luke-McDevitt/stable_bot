# ODrive Pro 0.6.11-1 endpoint reference (hw 4.4)

Reference table of CAN-SDO endpoints we might want to read or write from
`stewart_control_node` or the configurator scripts. Source of truth is
`odrive_flat_endpoints_0.6.11-1.json` in this same dir, fetched verbatim
from ODrive's CDN release. **Endpoint IDs are firmware-version-specific**
— if any drive on the bus is on a different firmware, none of these IDs
apply to it.

How to use these IDs in code: the SDO arbitration ID is
`(node_id << 5) | 0x04` for write (Tx_SDO) and the response uses
`0x05`. Payload format is `<BHB><value>` where the first byte is the
opcode (0x00 read / 0x01 write), next 2 bytes the endpoint id (LE),
then 1 byte reserved, then the value packed per its type.

Type → struct format on the wire:
| type     | struct fmt | bytes |
|----------|------------|-------|
| `bool`   | `<?`       | 1     |
| `uint8`  | `<B`       | 1     |
| `uint16` | `<H`       | 2     |
| `uint32` | `<I`       | 4     |
| `int32`  | `<i`       | 4     |
| `float`  | `<f`       | 4     |
| `function` | (no payload) | 0  |

## Endpoints currently used by the configurator (verified 2026-04-28)

These are also in `odrive_endpoints.json`, which is the file the
configurator reads. They match official 0.6.11-1 byte-for-byte.

| path | id | type | what it does |
|---|---|---|---|
| `axis0.controller.config.vel_integrator_gain` | 382 | float | inner velocity-loop integrator gain. ODrive stock is 0.333. |
| `axis0.config.motor.wL_FF_enable` | 305 | bool | inertia-/load-feedforward in the velocity controller. **Must be True** for level-loop tracking to hold ±0.1°. The wL_FF=False regression in 34d4589 cost us 30x tracking quality. |
| `axis0.config.can.encoder_msg_rate_ms` | 275 | uint32 | period of the cyclic Get_Encoder_Estimates broadcast. Default 10 ms (100 Hz); we run 2 ms (500 Hz) for inner-loop visibility. Bus math: 6 drives × 500 Hz × 130 bits ≈ 390 kbps on a 1 Mbps bus. |
| `axis0.controller.config.control_mode` | 378 | uint8 | 0=VOLTAGE, 1=TORQUE, 2=VELOCITY, 3=POSITION. Level loop needs 3. WebGUI's "Run Configuration Script" has historically persisted this as 2 → silent watchdog disarm. The runtime self-heal `_prepare_for_level()` re-asserts 3 via standard CAN cmd 0x00B before every Level ON. |
| `axis0.controller.config.input_mode` | 379 | uint8 | 0=INACTIVE, 1=PASSTHROUGH, 2=VEL_RAMP, 3=POS_FILTER, 4=MIX_CHANNELS, 5=TRAP_TRAJ, 6=TORQUE_RAMP, 7=MIRROR, 8=TUNING. Level loop needs 1 (PASSTHROUGH) so high-rate Set_Input_Pos commands aren't filtered. |
| `save_configuration` | 710 | function | persist runtime config to flash. Drive reboots (~18 s) — don't call mid-test. |
| `clear_errors` | 734 | function | clear `active_errors` + `disarm_reason` without rebooting. |

## Inner-loop gains (potential next-iteration knobs)

If we ever want to tune the ODrive controller's own inner loop from
the configurator instead of the WebGUI, these are the relevant knobs.
Re-derived per drive at calibration time, so values may differ across
nodes after recalibration.

| path | id | type | typical default | notes |
|---|---|---|---|---|
| `axis0.controller.config.pos_gain` | 380 | float | 20.0 | inner position-loop P gain (turns/s per turn of error) |
| `axis0.controller.config.vel_gain` | 381 | float | varies | inner velocity-loop P gain (Nm per turn/s of error) |
| `axis0.controller.config.vel_integrator_limit` | 383 | float | inf | clamp on inner velocity I term |
| `axis0.controller.config.vel_limit` | 384 | float | 2.0 | per-axis velocity cap (turns/s) |
| `axis0.controller.config.vel_limit_tolerance` | 385 | float | 1.2 | overshoot allowance before VEL_LIMIT_VIOLATION (bit 0x00008000) |

## Current limits (replaces deprecated `current_lim`)

ODrive 0.6.x split the old `current_lim` into soft + hard. We set
these via the existing `_arm_leg_internal` flow today (using standard
CAN command 0x00F = SET_LIMITS), but writing them via SDO is also
possible.

| path | id | type | notes |
|---|---|---|---|
| `axis0.config.motor.current_soft_max` | 315 | float | soft current cap (Amps). Standard CAN cmd 0x00F writes this. |
| `axis0.config.motor.current_hard_max` | 316 | float | hard current cap; firmware faults if exceeded |
| `axis0.config.motor.torque_constant` | 302 | float | Nm/A; updated by motor calibration. |
| `axis0.motor.effective_current_lim` | 579 | float (read-only) | the limit actually being applied this tick — useful diagnostic when current limiting is suspected. |

## Per-axis state and errors

Useful for SDO-based monitoring without parsing CAN broadcasts. Most
of these are exposed via standard CAN commands too (0x001 Heartbeat
gives state, 0x003 Get_Error gives active_errors + disarm_reason),
so prefer those for high-rate use.

| path | id | type | notes |
|---|---|---|---|
| `axis0.requested_state` | 224 | uint8 (rw) | write to command a state transition (1=IDLE, 8=CLOSED_LOOP). Standard CAN cmd 0x007 (Set_Axis_State) writes this. |
| `axis0.current_state` | 223 | uint8 (r) | live state. Heartbeat carries this byte already. |
| `axis0.active_errors` | 217 | uint32 (r) | currently-asserted error bits. Heartbeat / Get_Error carry this. |
| `axis0.disarm_reason` | 218 | uint32 (r) | why the drive last disarmed. Same. |

## Bus-config + per-axis CAN config

The CAN-related broadcast rate knobs, if we want to tune what each
drive emits cyclically.

| path | id | type | notes |
|---|---|---|---|
| `axis0.config.can.node_id` | 272 | uint32 | the drive's CAN node id. Reset to a default by `--erase-all`. **Path moved from `config.can.node_id` (0.6.11.0) to per-axis `axis0.config.can.node_id` (0.6.11-1)** — important when reading via odrivetool shell. |
| `axis0.config.can.heartbeat_msg_rate_ms` | 274 | uint32 | default 100 ms (10 Hz). Listener reads state from this. |
| `axis0.config.can.error_msg_rate_ms` | 277 | uint32 | default 0 (disabled, RTR-only). We use RTR-on-demand. |
| `axis0.config.can.iq_msg_rate_ms` | 276 | uint32 | period of cyclic Get_Iq broadcast. We poll via RTR instead. |

## Watchdog

The watchdog that has bitten us repeatedly. Not configured via the
configurator today, but worth knowing exists.

| path | id | type | notes |
|---|---|---|---|
| `axis0.config.watchdog_timeout` | 243 | float | seconds. Drive disarms with `WATCHDOG_TIMER_EXPIRED` (bit 24, 0x01000000) if no input command received within this window after STATE_CLOSED_LOOP. |
| `axis0.config.enable_watchdog` | 244 | bool | turns the watchdog on/off. Currently on; do not disable as a workaround for "the host is too slow" — fix the host instead. |
| `axis0.watchdog_feed` | 603 | function | call to feed without sending a motion command. We don't use this; the feeder's 50 Hz Set_Input_Pos / Set_Input_Vel feeds it implicitly. |

## Spinout protection

ODrive's automatic protection against runaway-control conditions.
If `active_errors` ever shows `SPINOUT_DETECTED` (bit 0x04000000),
these are the knobs to look at.

| path | id | type | notes |
|---|---|---|---|
| `axis0.controller.config.spinout_mechanical_power_bandwidth` | 398 | float | LPF on mechanical power for spinout detection |
| `axis0.controller.config.spinout_electrical_power_bandwidth` | 399 | float | LPF on electrical power |
| `axis0.controller.config.spinout_mechanical_power_threshold` | 400 | float | trip level (W) |
| `axis0.controller.config.spinout_electrical_power_threshold` | 401 | float | trip level (W) |
| `axis0.controller.spinout_mechanical_power` | 407 | float (r) | live filtered mechanical power |
| `axis0.controller.spinout_electrical_power` | 408 | float (r) | live filtered electrical power |

## Maintenance functions (no payload, side-effecting)

| path | id | type | notes |
|---|---|---|---|
| `save_configuration` | 710 | function | persist to flash; drive reboots. ~18 s. |
| `erase_configuration` | 712 | function | wipe flash config. Drive reboots into uncalibrated state. **Destructive.** |
| `reboot` | 713 | function | software reboot, config preserved. |
| `enter_dfu_mode2` | 715 | function | software-trigger DFU bootloader entry. Useful for CAN-DFU without touching the physical DFU/RUN switch. |
| `clear_errors` | 734 | function | clear `active_errors` + `disarm_reason`. Does not reboot. |

## Refreshing this table

If we move to a newer firmware, refetch the official endpoint table:

```bash
# Find the current release URL on https://docs.odriverobotics.com/releases/firmware
# (it's a per-build hashed CDN URL like the one below)
curl -sLo odrive_flat_endpoints_<version>.json \
    "https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/firmware/<hash>/flat_endpoints.json"
```

Then regenerate `odrive_endpoints.json` (the configurator's input) by
running `dump_odrive_endpoints.py` against any one drive over USB.
The dump checks `_<name>_property._info` per attribute, so it derives
the IDs straight from the live device — same source of truth as the
official table when both drive and tool are on the same firmware.
