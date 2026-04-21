# Stable-Bot Architecture

## Process graph

```
                                               ┌──────────────────────────┐
     HARDWARE                  PROCESSES       │   BROWSER (on any host)  │
  ──────────────            ─────────────────   │                          │
                                               │   web GUI (index.html)   │
                                               │   roslibjs + fetch()     │
                                               └────────┬─────────────────┘
                                                        │ ws: 9090   http: 8080
                                               ┌────────▼─────────┐
                                               │ rosbridge (9090) │
                                               │ gui_server (8080)│
                                               └────────┬─────────┘
                                                        │ ROS topics
                  ┌──────────────┐              ┌───────▼──────────────┐
  MTi-630 #1 ────►│ xsens node   │─/base/*─────►│                      │
  MTi-630 #2 ────►│ xsens node   │─/platform/*─►│                      │
                  └──────────────┘              │                      │
                                                │  stewart_control     │
  USB-CAN ─── can0 ────────┐                    │       _node          │
                           │                    │                      │
   ┌── ODrive 0 ──┐         │                    │  (rclpy executor +   │
   ┌── ODrive 1 ──┤         │                    │   background threads)│
   ┌── ODrive 2 ──┤ <─ CAN ─┘                    │                      │
   ┌── ODrive 3 ──┤         │                    │                      │
   ┌── ODrive 4 ──┤         │                    │                      │
   ┌── ODrive 5 ──┘         │                    └──────────────────────┘
                            │                           ▲
                            └──── ODriveFeeder ─────────┤
                            └──── EncoderListener ──────┤
                                                        │
                            (optional) stall_home subproc when homing
```

## Threads inside `stewart_control_node`

| Thread | Rate | Responsibility |
|---|---|---|
| rclpy executor | event-driven | service calls, topic callbacks, timers |
| ODriveFeeder | 50 Hz | per-leg `Set_Input_Pos` (with vel_ff) or `Set_Input_Vel` |
| EncoderListener RX | blocking `bus.recv` | absorb `Get_Encoder_Estimates` (0x009), `Get_Error` (0x003), `Get_Iq` (0x014) |
| EncoderListener TX | 10 Hz / 2 Hz | RTR request encoders + errors/Iq so cyclic broadcast isn't required |
| Level loop | 50 Hz | PI control with dynamic reference + velocity feedforward integration |
| Routine player | 100 Hz | linear interpolation between keyframes, forwards to `_do_set_pose` |
| Recording sampler | 50 Hz | append a row to `telemetry.csv` during `rec`-enabled playback |
| Homing pump | blocking | stream stall_home subprocess stdout to `/homing_output` |
| CAN reset worker | on-demand | background `ip link down/up` without blocking rclpy |

## Topics

| Topic | Type | Publisher | Rate | Purpose |
|---|---|---|---|---|
| `/status` | `std_msgs/String` (JSON) | control_node | 2 Hz | armed flags, IMU freshness, level errors, routine state |
| `/leg_encoders` | `Float64MultiArray` (6) | control_node | 20 Hz | motor positions in turns |
| `/leg_currents` | `Float64MultiArray` (6) | control_node | 20 Hz | Iq_measured in amps |
| `/platform_rpy` | `Float32MultiArray` (3) | control_node | 20 Hz | IMU roll/pitch/yaw |
| `/odrive_errors` | `std_msgs/String` (JSON) | control_node | 2 Hz | per-node active_errors + disarm_reason |
| `/platform/imu/data` | `sensor_msgs/Imu` | xsens (platform) | 400 Hz | main leveling feedback |
| `/base/imu/data` | `sensor_msgs/Imu` | xsens (base) | 400 Hz | not yet used (future: wave rejection) |
| `/control_cmd` | `std_msgs/String` (JSON) | browser | on-demand | **primary action bus** (see below) |
| `/control_result` | `std_msgs/String` (JSON) | control_node | after each cmd | replies to `/control_cmd` |
| `/jog_vel_cmd` | `std_msgs/Float32` | browser | ~10 Hz while dragging | dead-man velocity slider |
| `/homing_output` | `std_msgs/String` | control_node | per line | stall_home subprocess stdout |
| `/homing_stdin_in` | `std_msgs/String` | browser | on Enter | stdin piped to stall_home |

## `/control_cmd` dispatcher

Rather than rely on ROS services (which have an intermittent discovery
bug in rosbridge on WSL2), every GUI action is a JSON message on
`/control_cmd`:

```json
{"cmd": "jog_leg", "leg": 0, "delta_turns": 0.1}
{"cmd": "set_pose", "x": 0, "y": 0, "z": 50, "roll": 0, "pitch": 0, "yaw": 0}
{"cmd": "play_routine", "name": "demo_wave", "loop": false, "record": true}
```

Every command is dispatched to the same internal methods the
equivalent ROS service handler uses, so both paths behave identically.
Replies land on `/control_result` with the same `cmd` field plus
`success` and `message`. Supported `cmd` values include:

- arm / disarm: `activate`, `deactivate`, `e_stop`, `arm_leg`
- motion: `jog_leg`, `set_pose`, `go_to_rest`, `enable_level`
- limits: `set_speed_cap`, `set_leg_current`
- errors: `read_errors`, `clear_errors`
- homing: `start_homing`, `cancel_homing`
- routines: `list_routines`, `start_recording`, `record_keyframe`,
  `clear_recording`, `save_recording`, `play_routine`, `stop_routine`,
  `delete_routine`, `generate_rolling_ball_routine`
- diagnostics: `probe_graph`, `reset_can`, `restart_ros2_daemon`

## Recovery path (when things break)

1. `scripts/reset_stewart_stack.py --launch --verify` — the go-to.
2. GUI Diagnostics panel: **View launch log**, **Hard reset stack**,
   **(Re)launch stack**, **Reconnect rosbridge**, **Restart ros2 daemon**
   — all bypass rosbridge via the `gui_server.py` HTTP endpoints.
3. `scripts/read_odrive_errors.py --clear` — direct CAN to clear ODrive
   error latches.
4. USB cycle the CAN adapter via `usbipd detach/attach` (WSL2) or
   physical replug (Pi).

## Key algorithmic choices

- **Per-leg mode feeder** — each leg independently in `pos`, `vel`, or
  `idle`. Velocity-mode legs receive `Set_Input_Vel(shared_vel)`;
  position-mode legs receive `Set_Input_Pos(target, vel_ff=dpos/dt)`;
  idle legs get nothing. Avoids fighting between pose-hold and dead-man
  jog.
- **Velocity feed-forward from finite-diff target** — collapses dynamic
  tracking error. Required when the PI gain is too low to track a
  moving reference (see the rolling-ball session notes).
- **Dynamic level reference** — PI drives `IMU_rpy → level_ref +
  current_rpy` rather than a static `level_ref`. Means commanded tilt is
  actually achieved in the world frame; routine commands aren't
  cancelled by the PI.
- **Watchdog strategy** — feeder runs at 50 Hz *before* any axis enters
  CLOSED_LOOP and continues until *after* every axis reaches IDLE. This
  prevents the 500 ms ODrive watchdog from ever firing mid-transition.
  (See `feedback_odrive_watchdog.md` memory.)
- **Homing releases the bus** — stall_home.py runs as a subprocess with
  `python -u`; the node closes its CAN socket first, then reacquires
  after the subprocess exits. Stdout is pumped byte-by-byte with a
  150 ms idle flush so `input()` prompts show up in the GUI log
  without waiting for the reply.
