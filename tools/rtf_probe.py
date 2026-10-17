#!/usr/bin/env python3
"""Measure simulation real-time factor from /clock over N wall seconds."""
import sys, time, rclpy
from rosgraph_msgs.msg import Clock
rclpy.init(); n = rclpy.create_node('rtf_probe'); s = []
n.create_subscription(Clock, '/clock', lambda m: s.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
t0 = time.monotonic()
while time.monotonic() - t0 < dur + 2: rclpy.spin_once(n, timeout_sec=0.1)
s = [x for x in s if x[0] - t0 > 2]
print('RTF = %.3f' % ((s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0])) if len(s) > 2 else 'no /clock')
