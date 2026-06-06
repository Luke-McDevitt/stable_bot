# System profile — demo2_postcuts  (20260606T011919Z)

- code git sha: `c578663`  ·  OAK depth subsystem: **unknown**
- window: 25.0s  ·  cores: 4  ·  **COMPUTE-BOUND (cores saturated at full clock)**
- CPU busy: **99.3%**  ·  iowait: 0.0%  ·  load1 (min/mean/max): 13.8/15.6/16.6
- ARM MHz (min/mean/max): 2400.0/2400.0/2400.0  ·  temp °C: 58.7/60.1/61.5
- throttle flags: none (0x0)
- mem used/total: 1278.2/15973.3 MB
- **tracked stack ≈ 3.38 of 4 cores**

| # | node | %core | %machine | RSS MB | thr  Δ%core |
|---|------|------:|---------:|-------:|----:-------:|
| 1 | 🐍 oak_driver | 60.5 | 15.1 | 260.5 | 36  +5.8 |
| 2 | 🐍 rosbridge_websocket | 46.8 | 11.7 | 90.0 | 20  +11.8 |
| 3 | 🐍 platform_pose | 41.4 | 10.4 | 133.2 | 21  -0.7 |
| 4 | 🐍 stewart_control_node | 40.8 | 10.2 | 89.8 | 22  +5.3 |
| 5 | 🐍 ball_kf | 26.8 | 6.7 | 74.6 | 18  +3.7 |
| 6 | 🐍 ref_generator | 24.4 | 6.1 | 74.8 | 18  +6.3 |
| 7 | 🐍 ball_localizer | 20.0 | 5.0 | 88.9 | 18  +1.8 |
| 8 | xsens_mti_platform | 19.7 | 4.9 | 40.4 | 20  +2.4 |
| 9 | 🐍 ros2 bag record | 18.4 | 4.6 | 81.0 | 21  new |
| 10 | xsens_mti_base | 15.4 | 3.8 | 40.8 | 20  +0.6 |
| 11 | 🐍 ros2 bag record | 15.2 | 3.8 | 81.0 | 22  new |
| 12 | [kworker/u9:0-brcmf_wq/mmc1:0001:1] | 2.7 | 0.7 | 0.0 | 1  +1.9 |
| 13 | 🐍 calibration_node | 2.2 | 0.5 | 89.3 | 18  -36.7 |
| 14 | [kworker/0:1-events] | 0.6 | 0.1 | 0.0 | 1  new |
| 15 | [ksoftirqd/0] | 0.4 | 0.1 | 0.0 | 1  +0.0 |
| 16 | [rcu_preempt] | 0.4 | 0.1 | 0.0 | 1  +0.0 |
| 17 | [kworker/u8:3-ext4-rsv-conversion] | 0.4 | 0.1 | 0.0 | 1  new |
| 18 | 🐍 profile_system.py | 0.4 | 0.1 | 13.4 | 1  +0.1 |
| 19 | [kworker/0:1H-mmc_complete] | 0.3 | 0.1 | 0.0 | 1  +0.3 |
| 20 | [kworker/u8:0-events_unbound] | 0.3 | 0.1 | 0.0 | 1  +0.2 |
| 21 | [kworker/0:0-events] | 0.3 | 0.1 | 0.0 | 1  -0.1 |
| 22 | 🐍 gui_server | 0.3 | 0.1 | 24.1 | 2  +0.2 |

## py-spy own-time (hottest Python nodes)

### oak_driver (pid 12563, 4185 samples)
| own% | function |
|-----:|----------|
| 20.5 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 18.4 | `_worker (concurrent/futures/thread.py:89)` |
| 9.8 | `_tick (stewart_vision/oak_driver_node.py:2511)` |
| 7.6 | `publish (rclpy/publisher.py:72)` |
| 5.9 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:238)` |
| 3.4 | `yolo_v1_decode (stewart_vision/oak_driver_node.py:117)` |
| 2.1 | `await_or_execute (rclpy/executors.py:138)` |
| 2.0 | `_wrapfunc (numpy/core/fromnumeric.py:59)` |
| 1.9 | `data (sensor_msgs/msg/_compressed_image.py:202)` |
| 1.9 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:239)` |
| 1.7 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:255)` |
| 1.5 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:252)` |

### stewart_control_node (pid 12562, 3821 samples)
| own% | function |
|-----:|----------|
| 12.1 | `_recv_internal (can/interfaces/socketcan/socketcan.py:827)` |
| 9.7 | `capture_message (can/interfaces/socketcan/socketcan.py:624)` |
| 7.1 | `_run (stewart_bringup/stewart_control_node.py:1038)` |
| 6.2 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 6.0 | `publish (rclpy/publisher.py:72)` |
| 5.6 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2796)` |
| 3.4 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2805)` |
| 3.3 | `orientation_covariance (sensor_msgs/msg/_imu.py:254)` |
| 3.1 | `_take_subscription (rclpy/executors.py:540)` |
| 1.8 | `_tx_loop (stewart_bringup/stewart_control_node.py:712)` |
| 1.7 | `set_pos_targets (stewart_bringup/stewart_control_node.py:941)` |
| 1.2 | `_send_once (can/interfaces/socketcan/socketcan.py:889)` |

### rosbridge_websocket (pid 12769, 1914 samples)
| own% | function |
|-----:|----------|
| 41.7 | `send (rosbridge_library/protocol.py:316)` |
| 10.2 | `_write_to_self (asyncio/selector_events.py:152)` |
| 6.8 | `_take_subscription (rclpy/executors.py:540)` |
| 3.6 | `write_to_fd (tornado/iostream.py:1124)` |
| 2.2 | `_read_from_self (asyncio/selector_events.py:132)` |
| 1.1 | `add_to_wait_set (rclpy/event_handler.py:176)` |
| 1.1 | `serialize (rosbridge_library/protocol.py:347)` |
| 0.6 | `can_execute (rclpy/callback_groups.py:114)` |
| 0.6 | `add_to_wait_set (rclpy/event_handler.py:177)` |
| 0.6 | `_has_coroutine_mark (inspect.py:409)` |
| 0.6 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 0.6 | `run_coroutine_threadsafe (asyncio/tasks.py:938)` |

### platform_pose (pid 12564, 1410 samples)
| own% | function |
|-----:|----------|
| 75.8 | `_on_image (stewart_vision/platform_pose_node.py:137)` |
| 10.1 | `_on_image (stewart_vision/platform_pose_node.py:129)` |
| 4.9 | `publish (rclpy/publisher.py:72)` |
| 2.4 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 1.3 | `_on_image (stewart_vision/platform_pose_node.py:167)` |
| 0.6 | `__init__ (std_msgs/msg/_header.py:95)` |
| 0.4 | `_on_image (stewart_vision/platform_pose_node.py:189)` |
| 0.4 | `wait_for_ready_callbacks (rclpy/executors.py:963)` |
| 0.3 | `_wait_for_ready_callbacks (rclpy/executors.py:781)` |
| 0.3 | `_on_image (stewart_vision/platform_pose_node.py:182)` |
| 0.3 | `publish (rclpy/publisher.py:70)` |
| 0.2 | `await_or_execute (rclpy/executors.py:138)` |

---
_Generated by profile_system.py. Re-run during a demo2/bench and at idle; pass --compare <old profile.json> for deltas._
