#!/usr/bin/env python3
"""Request a Nav2 plan (ComputePathToPose) and render it over the map, with the robot's true pose.
usage: plot_plan.py <map.yaml> <gx> <gy> <out.png> [dqn_checkpoint]"""
import sys, math, rclpy, numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
import yaml, os
m = yaml.safe_load(open(sys.argv[1])); img = np.array(Image.open(os.path.join(os.path.dirname(sys.argv[1]), m["image"])))
res, (ox, oy) = m["resolution"], m["origin"][:2]
gx, gy = float(sys.argv[2]), float(sys.argv[3])
rclpy.init(); n = rclpy.create_node("plot_plan", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
c = ActionClient(n, ComputePathToPose, "compute_path_to_pose"); c.wait_for_server(timeout_sec=10)
g = ComputePathToPose.Goal(); g.goal.header.frame_id = "map"; g.goal.pose.position.x, g.goal.pose.position.y = gx, gy; g.goal.pose.orientation.w = 1.0; g.planner_id = "GridBased"
f = c.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=20); r = f.result().get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=20)
P = np.array([(p.pose.position.x, p.pose.position.y) for p in r.result().result.path.poses])
plt.figure(figsize=(6, 6)); plt.imshow(img, cmap="gray", extent=(ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res))
if len(P): plt.plot(P[:, 0], P[:, 1], "b-", lw=2, label=f"Nav2 Smac2D ({sum(math.dist(a, b) for a, b in zip(P, P[1:])):.2f} m)")
if len(sys.argv) > 5:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ros2_ws/src/maze_dqn")); from maze_dqn.planner import DQNPlanner
    d = DQNPlanner(sys.argv[5], map_yaml=sys.argv[1]); pts, _ = d.plan((-1.25, -1.25), (gx, gy))
    D = np.array([(p[0], p[1]) for p in pts]); plt.plot(D[:, 0], D[:, 1], "-", color="#d9822b", lw=2, label=f"DQN ({d.path_length(pts):.2f} m)")
plt.plot(gx, gy, "r*", ms=14); plt.legend(loc="upper left", fontsize=8); plt.title("planned routes on the SLAM map"); plt.savefig(sys.argv[4], dpi=110, bbox_inches="tight")
print("plan points", len(P))
