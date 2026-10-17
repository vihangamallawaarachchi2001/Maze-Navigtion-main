#!/usr/bin/env python3
"""Relay /tf_bag -> /tf, dropping transforms whose parent is 'map' (offline SLAM re-runs from a bag)."""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
class R(Node):
    def __init__(self):
        super().__init__("tf_relay_drop_map")
        self.pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_bag", self.cb, 100)
    def cb(self, m):
        keep = [t for t in m.transforms if t.header.frame_id.lstrip("/") != "map"]
        if keep:
            self.pub.publish(TFMessage(transforms=keep))
rclpy.init(); rclpy.spin(R())
