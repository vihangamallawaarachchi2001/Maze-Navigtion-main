#!/usr/bin/env python3
"""Scripted mapping drive (replaces keyboard teleoperation during SLAM).

Follows config/mapping_route.yaml with a rotate-then-drive controller. The robot pose is taken
from TF <pose_frame>->base_footprint. Default pose_frame "odom": odometry is pose-derived
(docs/progress.md) so steering by it cannot collide, and SLAM pose errors do not feed back
into the drive. Waypoints are in the world frame == odom frame == initial SLAM map frame.
Safety: stops and aborts if the scan is stale (> 1.5 s simulation time) or an obstacle is closer than `stop_range` in the driving direction; always sends zero velocity on exit.
The map is produced only by SLAM Toolbox from /scan + odometry.
"""
import math
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MappingDrive(Node):
    def __init__(self):
        super().__init__("mapping_drive")
        self.declare_parameter("route", os.path.join(get_package_share_directory("maze_bringup"),
                                                     "config", "mapping_route.yaml"))
        self.declare_parameter("max_speed", 0.15)
        self.declare_parameter("stop_range", 0.13)
        self.declare_parameter("pose_frame", "odom")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pose = None
        self.scan = None
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)

    def update_pose(self):
        try:
            t = self.tf.lookup_transform(self.get_parameter("pose_frame").value, "base_footprint",
                                         rclpy.time.Time())
        except Exception:
            return
        q, p = t.transform.rotation, t.transform.translation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_scan(self, m):
        self.scan = m

    def scan_age(self):
        """Age of the latest scan in SIMULATION seconds (wall time is meaningless at RTF < 1)."""
        st = self.scan.header.stamp
        return (self.get_clock().now().nanoseconds - (st.sec * 10**9 + st.nanosec)) * 1e-9

    def front_min(self):
        r = self.scan
        rg = np.asarray(r.ranges)
        ang = np.arctan2(np.sin(r.angle_min + np.arange(len(rg)) * r.angle_increment),
                         np.cos(r.angle_min + np.arange(len(rg)) * r.angle_increment))
        ok = (np.abs(ang) < 0.3) & (rg >= r.range_min) & (rg <= r.range_max)
        return float(rg[ok].min()) if ok.any() else float("inf")

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.pub.publish(t)

    def run(self):
        with open(self.get_parameter("route").value) as f:
            wps = yaml.safe_load(f)["waypoints"]
        vmax, stop_range = self.get_parameter("max_speed").value, self.get_parameter("stop_range").value
        while self.pose is None or self.scan is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            self.update_pose()
        t0 = time.monotonic()
        for k, (gx, gy) in enumerate(wps):
            while True:
                rclpy.spin_once(self, timeout_sec=0.05)
                self.update_pose()
                if self.scan_age() > 1.5:
                    self.get_logger().error("stale scan: stopping")
                    return False
                x, y, th = self.pose
                d = math.hypot(gx - x, gy - y)
                if d < 0.04:
                    break
                err = wrap(math.atan2(gy - y, gx - x) - th)
                if abs(err) > 0.25:
                    self.cmd(0.0, max(-1.0, min(1.0, 2.0 * err)))
                    continue
                if self.front_min() < stop_range:
                    self.cmd(0.0, 0.0)
                    self.get_logger().error(f"obstacle {self.front_min():.2f} m ahead: aborting")
                    return False
                self.cmd(min(vmax, 0.8 * d + 0.03), 2.5 * err)
            self.get_logger().info(f"waypoint {k + 1}/{len(wps)} reached ({time.monotonic() - t0:.0f}s wall)")
        self.cmd(0.0, 0.0)
        return True


def main():
    rclpy.init()
    n = MappingDrive()
    ok = False
    try:
        ok = n.run()
    finally:
        for _ in range(5):
            n.cmd(0.0, 0.0)
            time.sleep(0.05)
        n.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
