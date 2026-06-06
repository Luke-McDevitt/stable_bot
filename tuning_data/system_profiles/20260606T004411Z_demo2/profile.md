# System profile — demo2  (20260606T004411Z)

- code git sha: `bd51cc1`  ·  OAK depth subsystem: **unknown**
- window: 25.0s  ·  cores: 4  ·  **COMPUTE-BOUND (cores saturated at full clock)**
- CPU busy: **99.8%**  ·  iowait: 0.0%  ·  load1 (min/mean/max): 23.2/27.9/29.9
- ARM MHz (min/mean/max): 2400.0/2400.0/2400.0  ·  temp °C: 59.3/60.5/61.5
- throttle flags: none (0x0)
- mem used/total: 1474.6/15973.3 MB
- **tracked stack ≈ 2.87 of 4 cores**

| # | node | %core | %machine | RSS MB | thr |
|---|------|------:|---------:|-------:|----:|
| 1 | 🐍 oak_driver | 39.3 | 9.8 | 264.2 | 36 |
| 2 | 🐍 platform_pose | 34.2 | 8.6 | 134.9 | 21 |
| 3 | 🐍 calibration_node | 31.8 | 7.9 | 132.9 | 21 |
| 4 | 🐍 stewart_control_node | 27.5 | 6.9 | 89.3 | 22 |
| 5 | 🐍 rosbridge_websocket | 23.9 | 6.0 | 175.2 | 21 |
| 6 | 🐍 ball_kf | 19.1 | 4.8 | 75.3 | 18 |
| 7 | 🐍 auto_tune | 17.1 | 4.3 | 79.3 | 18 |
| 8 | xsens_mti_platform | 16.8 | 4.2 | 40.9 | 20 |
| 9 | 🐍 ref_generator | 16.2 | 4.1 | 75.5 | 18 |
| 10 | 🐍 ball_localizer | 15.0 | 3.8 | 89.1 | 18 |
| 11 | 🐍 ros2 bag record | 14.0 | 3.5 | 79.3 | 21 |
| 12 | xsens_mti_base | 13.1 | 3.3 | 41.3 | 20 |
| 13 | 🐍 ros2 bag record | 12.8 | 3.2 | 78.5 | 22 |
| 14 | 🐍 rosapi | 1.5 | 0.4 | 88.8 | 18 |
| 15 | 🐍 bag_recorder | 1.2 | 0.3 | 73.2 | 18 |
| 16 | [rcu_preempt] | 0.6 | 0.1 | 0.0 | 1 |
| 17 | [kworker/u9:0-brcmf_wq/mmc1:0001:1] | 0.6 | 0.2 | 0.0 | 1 |
| 18 | [ksoftirqd/0] | 0.4 | 0.1 | 0.0 | 1 |
| 19 | [kworker/u8:4-events_unbound] | 0.4 | 0.1 | 0.0 | 1 |
| 20 | 🐍 profile_system.py | 0.4 | 0.1 | 13.7 | 1 |
| 21 | 🐍 gui_server | 0.3 | 0.1 | 25.9 | 2 |

## py-spy own-time (hottest Python nodes)

### oak_driver (pid 8759, 4359 samples)
| own% | function |
|-----:|----------|
| 12.6 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 11.4 | `_worker (concurrent/futures/thread.py:89)` |
| 11.2 | `publish (rclpy/publisher.py:72)` |
| 9.2 | `data (sensor_msgs/msg/_compressed_image.py:202)` |
| 8.9 | `_tick (stewart_vision/oak_driver_node.py:2498)` |
| 7.9 | `yolo_v1_decode (stewart_vision/oak_driver_node.py:117)` |
| 5.0 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:238)` |
| 2.2 | `_tick_yolo (stewart_vision/oak_driver_node.py:2272)` |
| 2.1 | `await_or_execute (rclpy/executors.py:138)` |
| 1.7 | `detect_ball_v0 (stewart_vision/oak_driver_node.py:239)` |
| 1.7 | `add_to_wait_set (rclpy/event_handler.py:176)` |
| 1.4 | `_wrapfunc (numpy/core/fromnumeric.py:59)` |

### stewart_control_node (pid 8757, 3853 samples)
| own% | function |
|-----:|----------|
| 12.4 | `publish (rclpy/publisher.py:72)` |
| 10.4 | `_run (stewart_bringup/stewart_control_node.py:1038)` |
| 9.4 | `_recv_internal (can/interfaces/socketcan/socketcan.py:827)` |
| 9.2 | `capture_message (can/interfaces/socketcan/socketcan.py:624)` |
| 3.5 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2796)` |
| 3.3 | `_save_persisted_pose (stewart_bringup/stewart_control_node.py:2805)` |
| 3.3 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 2.5 | `_tx_loop (stewart_bringup/stewart_control_node.py:712)` |
| 2.2 | `_take_subscription (rclpy/executors.py:540)` |
| 1.3 | `capture_message (can/interfaces/socketcan/socketcan.py:674)` |
| 1.3 | `set_pos_targets (stewart_bringup/stewart_control_node.py:941)` |
| 1.1 | `_send_once (can/interfaces/socketcan/socketcan.py:889)` |

### calibration_node (pid 8764, 1846 samples)
| own% | function |
|-----:|----------|
| 80.1 | `_on_image (stewart_vision/calibration_node.py:197)` |
| 11.3 | `_on_image (stewart_vision/calibration_node.py:190)` |
| 1.4 | `publish (rclpy/publisher.py:72)` |
| 0.9 | `_take_subscription (rclpy/executors.py:540)` |
| 0.7 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 0.6 | `_wait_for_ready_callbacks (rclpy/executors.py:872)` |
| 0.4 | `_signature_bound_method (inspect.py:2118)` |
| 0.4 | `can_execute (rclpy/callback_groups.py:114)` |
| 0.3 | `add_to_wait_set (rclpy/event_handler.py:176)` |
| 0.3 | `ismethod (inspect.py:301)` |
| 0.2 | `encode (json/encoder.py:201)` |
| 0.2 | `_on_image (stewart_vision/calibration_node.py:189)` |

### platform_pose (pid 8760, 1796 samples)
| own% | function |
|-----:|----------|
| 76.2 | `_on_image (stewart_vision/platform_pose_node.py:137)` |
| 9.0 | `_on_image (stewart_vision/platform_pose_node.py:129)` |
| 6.6 | `publish (rclpy/publisher.py:72)` |
| 1.7 | `_on_image (stewart_vision/platform_pose_node.py:167)` |
| 1.1 | `_take_subscription (rclpy/executors.py:540)` |
| 0.3 | `_on_image (stewart_vision/platform_pose_node.py:189)` |
| 0.3 | `__init__ (geometry_msgs/msg/_point.py:96)` |
| 0.3 | `_wait_for_ready_callbacks (rclpy/executors.py:865)` |
| 0.2 | `publish (rclpy/publisher.py:70)` |
| 0.2 | `await_or_execute (rclpy/executors.py:138)` |
| 0.2 | `handler (rclpy/executors.py:665)` |
| 0.2 | `can_execute (rclpy/callback_groups.py:115)` |

## topic rates (Hz)

| topic | Hz |
|-------|---:|
| `/ball_state` | 0.0 |
| `/ball_track/diagnostic` | 0.0 |
| `/ball_xy_mono` | 0.0 |
| `/oak/ball/v0/rgb_pixel` | 0.0 |
| `/oak/health` | 0.0 |
| `/oak/latency_ms` | 0.0 |
| `/platform/imu/data` | 0.0 |
| `/status` | 2.233 |

---
_Generated by profile_system.py. Re-run during a demo2/bench and at idle; pass --compare <old profile.json> for deltas._
