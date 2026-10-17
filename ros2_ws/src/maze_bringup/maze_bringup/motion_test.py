#!/usr/bin/env python3
"""Supervised motion-and-stop test (Member 1 gate).

Drives forward at `speed` for `duration` seconds of SIMULATION time, then commands zero
velocity and checks that the robot (a) moved roughly speed*duration according to /odom and
(b) actually stopped. Aborts and stops immediately if /scan becomes stale or an obstacle is
closer than `stop_range` in front.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class MotionTest(Node):
    def __init__(self):
        super().__init__("motion_test")
        self.declare_parameter("speed", 0.10)
        self.declare_parameter("duration", 2.0)
        self.declare_parameter("stop_range", 0.20)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom = self.scan = None
        self.create_subscription(Odometry, "odom", lambda m: setattr(self, "odom", m), 20)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, m):
        self.scan = m

    def scan_age(self):
        """Age of the latest scan in SIMULATION seconds (wall time is meaningless at RTF < 1)."""
        st = self.scan.header.stamp
        return (self.get_clock().now().nanoseconds - (st.sec * 10**9 + st.nanosec)) * 1e-9

    def front_min(self):
        r = self.scan
        vals = []
        for k, v in enumerate(r.ranges):
            a = math.atan2(math.sin(r.angle_min + k * r.angle_increment), math.cos(r.angle_min + k * r.angle_increment))
            if abs(a) < 0.35 and r.range_min <= v <= r.range_max:
                vals.append(v)
        return min(vals) if vals else float("inf")

    def spin_until(self, pred, timeout):
        t0 = time.monotonic()
        while not pred() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return pred()

    def run(self):
        if not self.spin_until(lambda: self.odom is not None and self.scan is not None, 20.0):
            self.get_logger().error("FAIL: no /odom or /scan")
            return False
        p0 = self.odom.pose.pose.position
        speed, dur = self.get_parameter("speed").value, self.get_parameter("duration").value
        t0 = self.get_clock().now()
        reason = "completed"
        while (self.get_clock().now() - t0).nanoseconds * 1e-9 < dur:
            if self.scan_age() > 1.5:
                reason = "stale scan"
                break
            if self.front_min() < self.get_parameter("stop_range").value:
                reason = "obstacle"
                break
            cmd = Twist()
            cmd.linear.x = speed
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
        for _ in range(10):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        # wait 1 s of sim time for the robot to settle
        t1 = self.get_clock().now()
        self.spin_until(lambda: (self.get_clock().now() - t1).nanoseconds > 1e9, 30.0)
        p1 = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear.x
        d = math.hypot(p1.x - p0.x, p1.y - p0.y)
        ok = reason == "completed" and abs(d - speed * dur) < 0.5 * speed * dur and abs(v) < 0.01
        self.get_logger().info(f"{'PASS' if ok else 'FAIL'}: {reason}; moved {d:.3f} m "
                               f"(expected ~{speed * dur:.3f}), final speed {v:.4f} m/s")
        return ok


def main():
    rclpy.init()
    n = MotionTest()
    try:
        ok = n.run()
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
