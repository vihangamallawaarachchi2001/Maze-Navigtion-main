#!/usr/bin/env python3
"""Measure the LiDAR timestamp/pose consistency by matching scans against the reference map.

For each scan, the robot pose at the scan stamp is interpolated from /odom (pose-derived here),
then a yaw offset d is searched that best aligns scan endpoints with occupied reference-map cells.
At rest d should be ~0. While spinning at w, d/w estimates the scan time lag.
usage: scan_lag_probe.py <w> <seconds>
"""
import math, sys, time, bisect
import numpy as np, rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from maze_dqn.grid_map import OccupancyMap

ref = OccupancyMap(sys.argv[3] if len(sys.argv) > 3 else
                   __import__("os").path.expanduser("~/maze_testing/maps/reference/ground_truth_maze.yaml"))
rclpy.init(); n = rclpy.create_node("scan_lag", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
odo_t, odo_p, scans = [], [], []
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def on_odom(m):
    odo_t.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9); odo_p.append((m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation)))
n.create_subscription(Odometry, "odom", on_odom, 200)
n.create_subscription(LaserScan, "scan", lambda m: scans.append(m), qos_profile_sensor_data)
pub = n.create_publisher(Twist, "cmd_vel", 10)
w, T = float(sys.argv[1]), float(sys.argv[2])
while not odo_t: rclpy.spin_once(n, timeout_sec=0.1)
s = n.get_clock().now(); scans.clear()
while (n.get_clock().now() - s).nanoseconds*1e-9 < T:
    m = Twist(); m.angular.z = w; pub.publish(m); rclpy.spin_once(n, timeout_sec=0.01)
for _ in range(5): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.02)
occ = ref.occupied
def score(x, y, th, r):
    ang = r.angle_min + np.arange(len(r.ranges)) * r.angle_increment
    rg = np.array(r.ranges); ok = np.isfinite(rg) & (rg > r.range_min) & (rg < r.range_max)
    lx, ly = x - 0.032*math.cos(th), y - 0.032*math.sin(th)
    px = lx + rg[ok]*np.cos(th + ang[ok]); py = ly + rg[ok]*np.sin(th + ang[ok])
    ix = ((px - ref.origin[0]) / ref.resolution).astype(int); iy = ((py - ref.origin[1]) / ref.resolution).astype(int)
    v = (ix >= 0) & (iy >= 0) & (ix < ref.width) & (iy < ref.height)
    hit = np.zeros(len(px), bool); hit[v] = occ[iy[v], ix[v]]
    # tolerate 1-cell misses: also check neighbours
    for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
        jx, jy = ix+dx, iy+dy; vv = (jx >= 0) & (jy >= 0) & (jx < ref.width) & (jy < ref.height)
        hit[vv] |= occ[jy[vv], jx[vv]]
    return hit.mean()
res = []
for r in scans[1:]:
    t = r.header.stamp.sec + r.header.stamp.nanosec*1e-9
    k = bisect.bisect_left(odo_t, t)
    if k <= 0 or k >= len(odo_t): continue
    a, b = odo_p[k-1], odo_p[k]; f = (t - odo_t[k-1]) / max(1e-9, odo_t[k] - odo_t[k-1])
    x = a[0] + f*(b[0]-a[0]); y = a[1] + f*(b[1]-a[1]); th = a[2] + f*math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
    ds = np.arange(-0.6, 0.6001, 0.01)
    sc = [score(x, y, th + d, r) for d in ds]
    res.append((ds[int(np.argmax(sc))], max(sc), score(x, y, th, r)))
d = np.array([q[0] for q in res])
print("w=%.2f scans=%d best yaw offset mean=%.3f std=%.3f  (implied lag %.3f s)  hit@0=%.2f hit@best=%.2f" % (
    w, len(res), d.mean(), d.std(), (d.mean()/w if abs(w) > 1e-6 else float('nan')), np.mean([q[2] for q in res]), np.mean([q[1] for q in res])))
