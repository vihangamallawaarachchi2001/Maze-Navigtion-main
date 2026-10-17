#!/usr/bin/env python3
"""Repeatable trial driver for the baseline vs DQN comparison.

For every (scenario, method, repeat):
  1. stop the robot, place it at the scenario start with the Gazebo set_pose service
     (outside the measured run; logged in the trial notes),
  2. re-initialise AMCL with /initialpose at the same pose (map frame),
  3. wait for localisation to settle,
  4. set goal_adapter's `method` parameter and publish the goal on /clicked_goal,
  5. wait for the JSON record on /trial_result.

usage: ros2 run maze_bringup run_trials --ros-args -p goals_file:=<evaluation_goals.yaml>
       -p methods:="['baseline','dqn']" -p repeats:=3
"""
import json
import math
import subprocess
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String


class TrialRunner(Node):
    def __init__(self):
        super().__init__("trial_runner")
        self.declare_parameter("goals_file", "")
        self.declare_parameter("methods", ["baseline", "dqn"])
        self.declare_parameter("repeats", 3)
        self.declare_parameter("settle_s", 4.0)
        self.declare_parameter("result_timeout_s", 400.0)
        self.declare_parameter("only", [""])  # optional scenario-name filter
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.goal_pub = self.create_publisher(PoseStamped, "clicked_goal", 1)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.create_subscription(String, "trial_result", self.on_result, 10)
        self.param_cli = self.create_client(SetParameters, "/goal_adapter/set_parameters")
        self.clear_cli = [self.create_client(ClearEntireCostmap, s) for s in
                          ("/local_costmap/clear_entirely_local_costmap", "/global_costmap/clear_entirely_global_costmap")]
        self.last_result = None

    def on_result(self, msg):
        self.last_result = json.loads(msg.data)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_method(self, method):
        self.param_cli.wait_for_service(timeout_sec=10.0)
        req = SetParameters.Request()
        req.parameters = [Parameter(name="method", value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                                       string_value=method))]
        fut = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)

    def teleport(self, world, model, x, y, yaw):
        req = (f'name: "{model}" position {{x: {x} y: {y} z: 0.01}} '
               f'orientation {{z: {math.sin(yaw / 2):.6f} w: {math.cos(yaw / 2):.6f}}}')
        out = subprocess.run(["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose",
                              "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                             capture_output=True, text=True, timeout=15)
        return "true" in out.stdout

    def initial_pose(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = x, y
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.0025
        m.pose.covariance[35] = 0.003
        self.init_pub.publish(m)

    def run(self):
        with open(self.get_parameter("goals_file").value) as f:
            cfg = yaml.safe_load(f)
        world, model = cfg["world_name"], cfg["model_name"]
        mw = cfg.get("world_minus_map", [0.0, 0.0])  # map frame == world frame (docs/map_provenance.md)
        methods = list(self.get_parameter("methods").value)
        repeats = int(self.get_parameter("repeats").value)
        for rep in range(repeats):
            only = [o for o in self.get_parameter("only").value if o]
            for sc in [c for c in cfg["scenarios"] if not only or c["name"] in only]:
                for method in methods:
                    sx, sy, syaw = sc["start_map"]
                    gx, gy, gyaw = sc["goal_map"]
                    for _ in range(5):
                        self.cmd_pub.publish(Twist())
                        self.spin_for(0.05)
                    ok = self.teleport(world, model, sx + mw[0], sy + mw[1], syaw)
                    if not ok:
                        self.get_logger().error("set_pose failed; aborting")
                        return
                    self.spin_for(1.0)
                    for _ in range(3):
                        self.initial_pose(sx, sy, syaw)
                        self.spin_for(0.5)
                    # stale obstacle marks from before the reset would appear as phantom obstacles
                    for c in self.clear_cli:
                        if c.wait_for_service(timeout_sec=5.0):
                            f = c.call_async(ClearEntireCostmap.Request())
                            rclpy.spin_until_future_complete(self, f, timeout_sec=5.0)
                    self.spin_for(float(self.get_parameter("settle_s").value))
                    self.set_method(method)
                    self.last_result = None
                    g = PoseStamped()
                    g.header.frame_id = "map"
                    g.header.stamp = self.get_clock().now().to_msg()
                    g.pose.position.x, g.pose.position.y = gx, gy
                    g.pose.orientation.z, g.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
                    self.get_logger().info(f"rep {rep + 1}/{repeats} {sc['name']} {method}")
                    self.goal_pub.publish(g)
                    t0 = time.monotonic()
                    while self.last_result is None and time.monotonic() - t0 < self.get_parameter(
                            "result_timeout_s").value:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    r = self.last_result
                    self.get_logger().info(f"  -> {r['action_result'] if r else 'NO RESULT'} "
                                           f"success={r['success'] if r else 0}")


def main():
    rclpy.init()
    n = TrialRunner()
    try:
        n.run()
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
