#!/usr/bin/env python3
"""Require real sensor, transform, clock and map samples; run after SLAM or Nav2."""
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('maze_runtime_check')
    counts = dict(scan=0, odom=0, clock=0, tf=0, map=0, pubsub=0)
    details = {}

    def scan(msg):
        if len(msg.ranges) == 360 and msg.header.frame_id == 'base_scan':
            counts['scan'] += 1
            details['scan_beams'] = len(msg.ranges)

    def odom(msg):
        if msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_footprint':
            counts['odom'] += 1

    def clock(msg):
        if msg.clock.sec > 0:
            counts['clock'] += 1

    def tf(msg):
        if any(t.header.frame_id == 'odom' and t.child_frame_id == 'base_footprint'
               for t in msg.transforms):
            counts['tf'] += 1

    def mapping(msg):
        if msg.header.frame_id == 'map' and len(msg.data) == msg.info.width * msg.info.height:
            counts['map'] += 1
            details['map_cells'] = len(msg.data)
            details['known_map_cells'] = sum(v >= 0 for v in msg.data)

    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', odom, 10)
    node.create_subscription(Clock, '/clock', clock, 10)
    node.create_subscription(TFMessage, '/tf', tf, 10)
    node.create_subscription(OccupancyGrid, '/map', mapping,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub = node.create_publisher(String, '/maze_audit_pubsub', 10)
    node.create_subscription(String, '/maze_audit_pubsub',
                             lambda msg: counts.update(pubsub=counts['pubsub'] +
                                                       int(msg.data == 'maze-audit')), 10)
    end = time.monotonic() + 20
    while time.monotonic() < end and not all(counts.values()):
        pub.publish(String(data='maze-audit'))
        rclpy.spin_once(node, timeout_sec=0.1)
    result = dict(status='PASS' if all(counts.values()) else 'FAIL', counts=counts, details=details)
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['status'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
