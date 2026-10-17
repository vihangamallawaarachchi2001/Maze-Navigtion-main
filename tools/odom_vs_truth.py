#!/usr/bin/env python3
"""Compare DiffDrive odometry with Gazebo ground truth for a commanded motion.
usage: odom_vs_truth.py <v> <w> <sim_seconds> [<v> <w> <sim_seconds> ...]  (robot must have room)"""
import math, subprocess, sys, time, re, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def truth():
    out = subprocess.run(["gz", "model", "-m", "turtlebot3_burger", "-p"], capture_output=True, text=True, timeout=20).stdout
    nums = re.findall(r"\[([^\]]+)\]", out.split("Pose")[1])
    xyz = [float(v) for v in nums[2].split()]; rpy = [float(v) for v in nums[3].split()]
    return xyz[0], xyz[1], rpy[2]

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
rclpy.init(); n = rclpy.create_node("odom_vs_truth", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
pub = n.create_publisher(Twist, "cmd_vel", 10); od = {}
n.create_subscription(Odometry, "odom", lambda m: od.update(p=(m.pose.pose.position.x, m.pose.pose.position.y, yaw(m.pose.pose.orientation))), 10)
while "p" not in od: rclpy.spin_once(n, timeout_sec=0.1)
segs = [tuple(map(float, sys.argv[k:k + 3])) for k in range(1, len(sys.argv), 3)]
t0, o0 = truth(), od["p"]
acc_odom = 0.0; last = o0[2]; w_total = 0.0
for v, w, T in segs:
    s0 = n.get_clock().now(); w_total += w * T
    while (n.get_clock().now() - s0).nanoseconds * 1e-9 < T:
        m = Twist(); m.linear.x, m.angular.z = v, w; pub.publish(m); rclpy.spin_once(n, timeout_sec=0.02)
        d = math.atan2(math.sin(od["p"][2]-last), math.cos(od["p"][2]-last)); acc_odom += d; last = od["p"][2]
for _ in range(10): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
time.sleep(1.5); rclpy.spin_once(n, timeout_sec=0.1)
t1, o1 = truth(), od["p"]
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
print("truth  dpos %.3f  dyaw %.3f" % (math.hypot(t1[0]-t0[0], t1[1]-t0[1]), wrap(t1[2]-t0[2])))
print("odom   dpos %.3f  dyaw %.3f  (accumulated %.3f, commanded %.3f)" % (math.hypot(o1[0]-o0[0], o1[1]-o0[1]), wrap(o1[2]-o0[2]), acc_odom, w_total))
print("truth  end (%.3f, %.3f) ; odom end (%.3f, %.3f) relative to start" % (t1[0]-t0[0], t1[1]-t0[1], o1[0]-o0[0], o1[1]-o0[1]))
