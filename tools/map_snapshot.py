#!/usr/bin/env python3
"""Grab one /map message and write it as a PNG (for inspection), optional robot marker from TF."""
import sys, rclpy, numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from PIL import Image
rclpy.init(); n = rclpy.create_node("map_snapshot"); got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, "map", lambda m: got.update(m=m), qos)
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.2)
m = got["m"]; a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
img = np.full(a.shape, 205, np.uint8); img[a == 0] = 254; img[a > 50] = 0
img = img[::-1]
Image.fromarray(img).resize((img.shape[1] * 4, img.shape[0] * 4), Image.NEAREST).save(sys.argv[1])
print(f"{m.info.width}x{m.info.height} res {m.info.resolution} origin ({m.info.origin.position.x:.3f},{m.info.origin.position.y:.3f})")
