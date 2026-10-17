#!/usr/bin/env python3
"""Send one goal on /clicked_goal, record /plan, /dqn_plan and the TF map->base_footprint trace, plot over the map.
usage: trace_trial.py <map.yaml> <gx> <gy> <gyaw> <sim_seconds> <out.png>"""
import sys, math, os, rclpy, numpy as np, yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
m = yaml.safe_load(open(sys.argv[1])); img = np.array(Image.open(os.path.join(os.path.dirname(sys.argv[1]), m["image"])))
res, (ox, oy) = m["resolution"], m["origin"][:2]
gx, gy, gyaw, T = map(float, sys.argv[2:6])
rclpy.init(); n = rclpy.create_node("trace", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
tf = Buffer(); TransformListener(tf, n); plans = {}
n.create_subscription(Path, "plan", lambda p: plans.setdefault("plan", [(q.pose.position.x, q.pose.position.y) for q in p.poses]), 5)
n.create_subscription(Path, "dqn_plan", lambda p: plans.setdefault("dqn", [(q.pose.position.x, q.pose.position.y) for q in p.poses]), 5)
pub = n.create_publisher(PoseStamped, "clicked_goal", 1)
t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds < 2e9: rclpy.spin_once(n, timeout_sec=0.05)
g = PoseStamped(); g.header.frame_id = "map"; g.pose.position.x, g.pose.position.y = gx, gy
g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2); pub.publish(g)
tr = []; t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds * 1e-9 < T:
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = tf.lookup_transform("map", "base_footprint", rclpy.time.Time()); tr.append((t.transform.translation.x, t.transform.translation.y))
    except Exception: pass
plt.figure(figsize=(6, 6)); plt.imshow(img, cmap="gray", extent=(ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res))
for k, c in (("plan", "b"), ("dqn", "#d9822b")):
    if k in plans: P = np.array(plans[k]); plt.plot(P[:, 0], P[:, 1], "-", color=c, lw=1.5, label=f"{k} (first)")
if tr: R = np.array(tr); plt.plot(R[:, 0], R[:, 1], "g.", ms=2, label="robot (AMCL)")
plt.plot(gx, gy, "r*", ms=12); plt.legend(fontsize=8, loc="upper left"); plt.savefig(sys.argv[6], dpi=110, bbox_inches="tight"); print("trace points", len(tr))
