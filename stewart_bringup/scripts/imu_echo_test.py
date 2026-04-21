#!/usr/bin/env python3
"""
Subscribe to /imu/data with sensor-data QoS (best_effort/volatile) and
print every 400th quaternion reading. Used to verify the IMU driver is
publishing, bypassing the `ros2 topic` CLI / daemon entirely.

Usage:
  # Start the driver in another shell:
  #   ros2 launch stewart_bringup imu_launch.py
  # Then:
  python3 imu_echo_test.py               # subscribes to /imu/data
  python3 imu_echo_test.py /base/imu/data   # or any other topic
  python3 imu_echo_test.py --rate          # print every sample instead of every 400

Ctrl+C to stop.
"""
import argparse
import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
except ImportError:
    sys.exit("rclpy not found. Did you 'source /opt/ros/kilted/setup.bash'?")


def quat_to_rpy_deg(w, x, y, z):
    """Roll (X), Pitch (Y), Yaw (Z) in degrees from a quaternion.
    Standard intrinsic ZYX / Tait-Bryan convention."""
    # roll (X-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (Y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # yaw (Z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('topic', nargs='?', default='/imu/data',
                   help='Imu topic to subscribe to (default /imu/data)')
    p.add_argument('--every', type=int, default=400,
                   help='Print every Nth message (default 400 = 1/sec at 400 Hz)')
    p.add_argument('--rate', action='store_true',
                   help='Print every message (overrides --every)')
    args = p.parse_args()

    every = 1 if args.rate else args.every

    rclpy.init()
    node = Node('imu_echo_test')
    count = [0]

    def cb(msg):
        count[0] += 1
        if count[0] == 1 or count[0] % every == 0:
            q = msg.orientation
            av = msg.angular_velocity
            la = msg.linear_acceleration
            roll, pitch, yaw = quat_to_rpy_deg(q.w, q.x, q.y, q.z)
            print(f"[{count[0]:7}]  "
                  f"RPY=({roll:+7.2f}, {pitch:+7.2f}, {yaw:+7.2f}) deg  "
                  f"gyro=({av.x:+.3f}, {av.y:+.3f}, {av.z:+.3f})  "
                  f"accel=({la.x:+.2f}, {la.y:+.2f}, {la.z:+.2f})",
                  flush=True)

    node.create_subscription(Imu, args.topic, cb, qos_profile_sensor_data)
    print(f"subscribed to {args.topic}; waiting for data...", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nreceived {count[0]} messages total", flush=True)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
