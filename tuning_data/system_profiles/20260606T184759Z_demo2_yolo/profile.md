# System profile — demo2_yolo  (20260606T184759Z)

- code git sha: `e2d374d`  ·  OAK depth subsystem: **unknown**
- window: 25.0s  ·  cores: 4  ·  **COMPUTE-BOUND (cores saturated at full clock)**
- CPU busy: **99.0%**  ·  iowait: 0.0%  ·  load1 (min/mean/max): 12.5/14.0/14.8
- ARM MHz (min/mean/max): 1000.0/2344.0/2400.0  ·  temp °C: 55.4/57.3/58.7
- throttle flags: under_voltage_since_boot, throttled_since_boot
- mem used/total: 1316.7/15973.3 MB
- **tracked stack ≈ 3.3 of 4 cores**

| # | node | %core | %machine | RSS MB | thr  Δ%core |
|---|------|------:|---------:|-------:|----:-------:|
| 1 | 🐍 platform_pose | 46.6 | 11.6 | 135.0 | 21  -1.5 |
| 2 | 🐍 stewart_control_node | 43.6 | 10.9 | 90.1 | 22  +6.2 |
| 3 | 🐍 rosbridge_websocket | 43.4 | 10.8 | 95.0 | 20  +3.2 |
| 4 | 🐍 oak_driver | 41.8 | 10.5 | 260.4 | 36  -12.9 |
| 5 | 🐍 ball_kf | 27.7 | 6.9 | 75.5 | 18  +1.6 |
| 6 | 🐍 ref_generator | 25.2 | 6.3 | 75.2 | 18  +1.5 |
| 7 | xsens_mti_platform | 20.9 | 5.2 | 40.7 | 20  +0.8 |
| 8 | 🐍 ball_localizer | 20.6 | 5.2 | 89.0 | 18  +1.5 |
| 9 | 🐍 ros2 bag record | 19.3 | 4.8 | 83.4 | 21  +4.1 |
| 10 | 🐍 ros2 bag record | 16.3 | 4.1 | 81.2 | 22  +1.1 |
| 11 | xsens_mti_base | 15.7 | 3.9 | 40.7 | 20  -0.9 |
| 12 | [kworker/u9:0-brcmf_wq/mmc1:0001:1] | 2.4 | 0.6 | 0.0 | 1  +0.1 |
| 13 | 🐍 calibration_node | 2.3 | 0.6 | 90.3 | 18  +0.0 |
| 14 | [kworker/0:2-events] | 0.7 | 0.2 | 0.0 | 1  +0.5 |
| 15 | [rcu_preempt] | 0.5 | 0.1 | 0.0 | 1  +0.0 |
| 16 | [ksoftirqd/0] | 0.4 | 0.1 | 0.0 | 1  +0.0 |
| 17 | [kworker/u8:5-ext4-rsv-conversion] | 0.4 | 0.1 | 0.0 | 1  new |
| 18 | [kworker/u8:0-events_unbound] | 0.4 | 0.1 | 0.0 | 1  new |
| 19 | 🐍 gui_server | 0.4 | 0.1 | 29.6 | 2  +0.0 |
| 20 | 🐍 profile_system.py | 0.4 | 0.1 | 13.4 | 1  +0.0 |
| 21 | [kworker/0:0H-mmc_complete] | 0.3 | 0.1 | 0.0 | 1  +0.0 |
| 22 | 🐍 bag_recorder | 0.3 | 0.1 | 72.9 | 18  +0.0 |

## py-spy own-time (hottest Python nodes)

### stewart_control_node (pid 7358, 3853 samples)
| own% | function |
|-----:|----------|
| 13.4 | `_recv_internal (can/interfaces/socketcan/socketcan.py:827)` |
| 7.4 | `capture_message (can/interfaces/socketcan/socketcan.py:624)` |
| 7.4 | `_run (stewart_bringup/stewart_control_node.py:1038)` |
| 7.4 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 6.4 | `publish (rclpy/publisher.py:72)` |
| 5.7 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2808)` |
| 3.3 | `orientation_covariance (sensor_msgs/msg/_imu.py:254)` |
| 3.1 | `_take_subscription (rclpy/executors.py:540)` |
| 2.8 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2817)` |
| 2.4 | `_send_once (can/interfaces/socketcan/socketcan.py:889)` |
| 2.1 | `set_pos_targets (stewart_bringup/stewart_control_node.py:941)` |
| 1.7 | `_tx_loop (stewart_bringup/stewart_control_node.py:712)` |

### rosbridge_websocket (pid 7562, 1910 samples)
| own% | function |
|-----:|----------|
| 38.6 | `send (rosbridge_library/protocol.py:316)` |
| 9.7 | `_write_to_self (asyncio/selector_events.py:152)` |
| 4.9 | `_take_subscription (rclpy/executors.py:540)` |
| 3.8 | `write_to_fd (tornado/iostream.py:1124)` |
| 2.4 | `_read_from_self (asyncio/selector_events.py:132)` |
| 1.3 | `add_to_wait_set (rclpy/event_handler.py:176)` |
| 0.9 | `serialize (rosbridge_library/protocol.py:347)` |
| 0.9 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 0.8 | `__exit__ (rclpy/event_handler.py:190)` |
| 0.7 | `is_ready (rclpy/event_handler.py:152)` |
| 0.7 | `add_to_wait_set (rclpy/event_handler.py:177)` |
| 0.7 | `_has_coroutine_mark (inspect.py:409)` |

### platform_pose (pid 7360, 1436 samples)
| own% | function |
|-----:|----------|
| 80.2 | `_on_image (stewart_vision/platform_pose_node.py:137)` |
| 7.9 | `_on_image (stewart_vision/platform_pose_node.py:129)` |
| 3.3 | `publish (rclpy/publisher.py:72)` |
| 2.2 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 1.7 | `_on_image (stewart_vision/platform_pose_node.py:167)` |
| 0.4 | `_take_subscription (rclpy/executors.py:540)` |
| 0.2 | `await_or_execute (rclpy/executors.py:138)` |
| 0.2 | `_rot_to_quat (stewart_vision/platform_pose_node.py:201)` |
| 0.1 | `_has_code_flag (inspect.py:393)` |
| 0.1 | `_wait_for_ready_callbacks (rclpy/executors.py:811)` |
| 0.1 | `_on_image (stewart_vision/platform_pose_node.py:184)` |
| 0.1 | `_wait_for_ready_callbacks (rclpy/executors.py:852)` |

### oak_driver (pid 7359, 3160 samples)
| own% | function |
|-----:|----------|
| 21.8 | `_worker (concurrent/futures/thread.py:89)` |
| 20.3 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 10.3 | `yolo_v1_decode (stewart_vision/oak_driver_node.py:117)` |
| 7.6 | `publish (rclpy/publisher.py:72)` |
| 5.7 | `data (sensor_msgs/msg/_compressed_image.py:202)` |
| 2.1 | `await_or_execute (rclpy/executors.py:138)` |
| 2.1 | `_wrapfunc (numpy/core/fromnumeric.py:59)` |
| 2.0 | `add_to_wait_set (rclpy/event_handler.py:176)` |
| 1.7 | `add_to_wait_set (rclpy/event_handler.py:177)` |
| 1.7 | `_tick_yolo (stewart_vision/oak_driver_node.py:2285)` |
| 1.6 | `__enter__ (rclpy/event_handler.py:181)` |
| 1.5 | `submit (concurrent/futures/thread.py:178)` |

---
_Generated by profile_system.py. Re-run during a demo2/bench and at idle; pass --compare <old profile.json> for deltas._
