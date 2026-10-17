#!/usr/bin/env python3
"""Clicked-goal adapter, method selector and trial logger (Members 3 + 4).

* Subscribes to /clicked_goal (geometry_msgs/PoseStamped, frame "map"), which the RViz
  "2D Goal Pose" tool publishes in config rviz/maze.rviz. bt_navigator's own goal_pose
  topic is NOT used, so exactly one component turns clicks into motion.
* Parameter `method`:
    baseline -> ComputePathToPose (timed, same Nav2 planner) then NavigateToPose
    dqn      -> learned DQN route (maze_dqn.planner) -> validated nav_msgs/Path -> FollowPath
  Both modes use the same controller_server, footprint, limits and goal checker.
* Goals outside the map, in occupied/unknown space or too close to walls are rejected.
* A new goal cancels the running one; stop commands are sent on shutdown.
* Every trial is appended to the CSV (results_template.csv schema + extra columns) and
  published as JSON on /trial_result.
"""
import csv
import json
import math
import os
import queue
import re
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from maze_dqn.grid_map import OccupancyMap

CSV_FIELDS = ["method", "run_id", "seed", "world_revision", "map_source", "map_file", "start_x", "start_y",
              "start_yaw", "goal_x", "goal_y", "goal_yaw", "sim_elapsed_s", "wall_elapsed_s", "real_time_factor",
              "planning_time_s", "actual_path_m", "planned_path_m", "reference_shortest_path_m",
              "path_efficiency", "final_position_error_m", "final_heading_error_rad", "collision_count",
              "near_collision_count", "timeout", "action_result", "success", "notes",
              # extra columns (documented in docs/architecture.md)
              "total_rotation_rad", "angular_velocity_rms", "min_scan_range_m", "dqn_cells",
              "true_final_x", "true_final_y", "true_final_error_m", "localization_error_m"]

STATUS = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
          GoalStatus.STATUS_CANCELED: "canceled"}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gazebo_truth(model="turtlebot3_burger"):
    """Ground-truth (x, y) of the robot from Gazebo; map frame == world frame (docs/map_provenance.md)."""
    try:
        out = subprocess.run(["gz", "model", "-m", model, "-p"], capture_output=True, text=True, timeout=20).stdout
        nums = re.findall(r"\[([^\]]+)\]", out.split("Pose")[1])
        x, y = [float(v) for v in nums[2].split()][:2]
        return x, y
    except Exception:
        return None


class GoalAdapter(Node):
    def __init__(self):
        super().__init__("goal_adapter")
        p = self.declare_parameter
        p("method", "baseline")
        p("map_yaml", "")
        p("map_source", "slam")
        p("world_revision", "")
        p("dqn_checkpoint", "")
        p("results_csv", os.path.expanduser("~/maze_testing/results/trials.csv"))
        p("trial_timeout_s", 120.0)
        p("success_tolerance_m", 0.15)
        p("goal_min_clearance_m", 0.13)
        p("near_collision_range_m", 0.16)
        p("run_tag", "")
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.gp = gp

        self.omap = OccupancyMap(gp("map_yaml"))
        self.dqn = None
        ck = gp("dqn_checkpoint")
        if ck and ck != "none" and os.path.exists(ck):
            from maze_dqn.planner import DQNPlanner
            self.dqn = DQNPlanner(ck, map_yaml=gp("map_yaml"))
            self.get_logger().info(f"DQN planner loaded ({len(self.dqn.lat.cells)} lattice nodes) from {ck}")
        else:
            self.get_logger().warn(f"DQN disabled: checkpoint '{ck}' not found; only method=baseline works")

        cb = ReentrantCallbackGroup()
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.follow = ActionClient(self, FollowPath, "follow_path", callback_group=cb)
        self.compute = ActionClient(self, ComputePathToPose, "compute_path_to_pose", callback_group=cb)
        self.create_subscription(PoseStamped, "clicked_goal", self.on_goal, 10, callback_group=cb)
        self.create_subscription(Odometry, "odom", self.on_odom, 50, callback_group=cb)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Contacts, "bumper", self.on_contact, 50, callback_group=cb)
        self.path_pub = self.create_publisher(Path, "dqn_plan", 1)
        self.result_pub = self.create_publisher(String, "trial_result", 10)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 1)

        self.goals = queue.Queue()
        self.cancel_event = threading.Event()
        self.lock = threading.Lock()
        self.rec = None  # active trial recorder dict
        self.run_counter = 0
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f"goal_adapter ready, method={gp('method')}; click with '2D Goal Pose'")

    # ------------------------------------------------------------------ sensors
    def on_odom(self, msg):
        with self.lock:
            r = self.rec
            if r is None:
                return
            p = msg.pose.pose.position
            y = yaw_of(msg.pose.pose.orientation)
            if r["last_odom"] is not None:
                lx, ly, lyaw = r["last_odom"]
                r["length"] += math.hypot(p.x - lx, p.y - ly)
                r["rotation"] += abs(wrap(y - lyaw))
            r["last_odom"] = (p.x, p.y, y)
            w = msg.twist.twist.angular.z
            r["w_sq"] += w * w
            r["w_n"] += 1

    def on_scan(self, msg):
        with self.lock:
            r = self.rec
            if r is None:
                return
            vals = [v for v in msg.ranges if msg.range_min <= v <= msg.range_max]
            if not vals:
                return
            m = min(vals)
            r["min_range"] = min(r["min_range"], m)
            near = m < self.gp("near_collision_range_m")
            if near and not r["near_state"]:
                r["near_count"] += 1
            r["near_state"] = near

    def on_contact(self, msg):
        with self.lock:
            r = self.rec
            if r is None:
                return
            hit = any("maze" in c.collision2.name or "maze" in c.collision1.name for c in msg.contacts)
            t = time.monotonic()
            if hit and t - r["last_contact"] > 0.5:  # debounce: one event per contact episode
                r["collisions"] += 1
            if hit:
                r["last_contact"] = t

    # ------------------------------------------------------------------ goals
    def on_goal(self, msg):
        if msg.header.frame_id not in ("map", "/map"):
            self.get_logger().error(f"rejected goal in frame '{msg.header.frame_id}', expected 'map'")
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        x0, y0, x1, y1 = self.omap.bounds
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            self.get_logger().error(f"rejected goal ({x:.2f},{y:.2f}): outside the map")
            return
        if not self.omap.is_free(x, y) or self.omap.clearance([(x, y)])[0] < self.gp("goal_min_clearance_m"):
            self.get_logger().error(f"rejected goal ({x:.2f},{y:.2f}): occupied/unknown or too close to a wall")
            return
        if self.rec is not None:
            self.get_logger().warn("new goal received: cancelling current goal")
            self.cancel_event.set()
        self.goals.put(msg)

    def robot_pose(self):
        t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=1.0))
        tr = t.transform
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    @staticmethod
    def wait(fut, timeout, cancel_event=None):
        t0 = time.monotonic()
        while not fut.done():
            if cancel_event is not None and cancel_event.is_set():
                return False
            if time.monotonic() - t0 > timeout:
                return False
            time.sleep(0.02)
        return True

    def worker(self):
        while rclpy.ok():
            try:
                goal = self.goals.get(timeout=0.2)
            except queue.Empty:
                continue
            while not self.goals.empty():  # only the most recent click matters
                goal = self.goals.get()
            self.cancel_event.clear()
            try:
                self.run_trial(goal)
            except Exception as e:  # never leave the robot moving after an internal error
                self.get_logger().error(f"trial failed with exception: {e}")
                with self.lock:
                    self.rec = None
                self.stop_robot()

    def run_trial(self, goal):
        method = self.gp("method")
        self.run_counter += 1
        run_id = f"{self.gp('run_tag') or time.strftime('%Y%m%d-%H%M%S')}-{method}-{self.run_counter:03d}"
        gx, gy, gyaw = goal.pose.position.x, goal.pose.position.y, yaw_of(goal.pose.orientation)
        try:
            sx, sy, syaw = self.robot_pose()
        except Exception as e:
            self.get_logger().error(f"no map->base_footprint transform (localisation not ready?): {e}")
            return
        row = {k: "" for k in CSV_FIELDS}
        row.update(method=method, run_id=run_id, seed="", world_revision=self.gp("world_revision"),
                   map_source=self.gp("map_source"), map_file=os.path.basename(self.gp("map_yaml")),
                   start_x=f"{sx:.3f}", start_y=f"{sy:.3f}", start_yaw=f"{syaw:.3f}",
                   goal_x=f"{gx:.3f}", goal_y=f"{gy:.3f}", goal_yaw=f"{gyaw:.3f}")
        self.get_logger().info(f"[{run_id}] goal ({gx:.2f},{gy:.2f},{gyaw:.2f}) from ({sx:.2f},{sy:.2f})")

        # ---------------- planning
        goal.header.stamp = self.get_clock().now().to_msg()
        t_plan = time.perf_counter()
        notes = []
        if method == "dqn":
            if self.dqn is None:
                self.get_logger().error("method=dqn but no dqn_checkpoint loaded")
                return
            from maze_dqn.planner import PlanningError
            try:
                pts, cells = self.dqn.plan((sx, sy), (gx, gy), goal_yaw=gyaw)
            except PlanningError as e:
                row.update(planning_time_s=f"{time.perf_counter() - t_plan:.4f}", action_result="planning_failed",
                           success=0, timeout=0, notes=f"DQN: {e}")
                self.get_logger().error(f"[{run_id}] DQN planning failed: {e}")
                self.finish(row)
                return
            row["planning_time_s"] = f"{time.perf_counter() - t_plan:.4f}"
            row["planned_path_m"] = f"{self.dqn.path_length(pts):.3f}"
            row["dqn_cells"] = " ".join(f"{i}:{j}" for i, j in cells)
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = goal.header.stamp
            for x, y, yaw in pts:
                ps = PoseStamped()
                ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = x, y
                q = quat(yaw)
                ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
                path.poses.append(ps)
            self.path_pub.publish(path)
            client, action_goal = self.follow, FollowPath.Goal()
            action_goal.path = path
            action_goal.controller_id = "FollowPath"
            action_goal.goal_checker_id = "general_goal_checker"
        else:
            if self.compute.wait_for_server(timeout_sec=5.0):
                cg = ComputePathToPose.Goal()
                cg.goal = goal
                cg.planner_id = "GridBased"
                cg.use_start = False
                h = self.compute.send_goal_async(cg)
                if self.wait(h, 10.0) and h.result().accepted:
                    rf = h.result().get_result_async()
                    if self.wait(rf, 10.0) and rf.result().status == GoalStatus.STATUS_SUCCEEDED:
                        poses = rf.result().result.path.poses
                        row["planning_time_s"] = f"{time.perf_counter() - t_plan:.4f}"
                        row["planned_path_m"] = f"{sum(math.dist((a.pose.position.x, a.pose.position.y), (b.pose.position.x, b.pose.position.y)) for a, b in zip(poses, poses[1:])):.3f}"
                    else:
                        notes.append("ComputePathToPose failed")
            client, action_goal = self.nav, NavigateToPose.Goal()
            action_goal.pose = goal

        # ---------------- execution
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            return
        with self.lock:
            self.rec = dict(length=0.0, rotation=0.0, w_sq=0.0, w_n=0, last_odom=None, min_range=float("inf"),
                            near_state=False, near_count=0, collisions=0, last_contact=-1e9)
        sim0, wall0 = self.get_clock().now(), time.monotonic()
        h = client.send_goal_async(action_goal)
        if not self.wait(h, 10.0) or not h.result().accepted:
            with self.lock:
                self.rec = None
            row.update(action_result="rejected", success=0, timeout=0, notes="goal rejected by Nav2")
            self.finish(row)
            return
        gh = h.result()
        rf = gh.get_result_async()
        timeout_s, timed_out, preempted = self.gp("trial_timeout_s"), False, False
        while not rf.done():
            if self.cancel_event.is_set():
                preempted = True
                break
            if (self.get_clock().now() - sim0).nanoseconds * 1e-9 > timeout_s:
                timed_out = True
                break
            time.sleep(0.05)
        if not rf.done():
            cf = gh.cancel_goal_async()
            self.wait(cf, 5.0)
            self.wait(rf, 5.0)
            self.stop_robot()
        sim_dt = (self.get_clock().now() - sim0).nanoseconds * 1e-9
        wall_dt = time.monotonic() - wall0
        status = STATUS.get(rf.result().status, str(rf.result().status)) if rf.done() else "unknown"
        if preempted:
            status, notes = "preempted", notes + ["cancelled by a newer clicked goal"]
        if timed_out:
            status = "timeout"
        time.sleep(0.3)
        with self.lock:
            rec, self.rec = self.rec, None
        try:
            ex, ey, eyaw = self.robot_pose()
            err = math.hypot(ex - gx, ey - gy)
            herr = abs(wrap(eyaw - gyaw))
        except Exception:
            ex = ey = err = herr = float("nan")
        truth = gazebo_truth()
        if truth is not None:
            terr = math.hypot(truth[0] - gx, truth[1] - gy)
            row.update(true_final_x=f"{truth[0]:.3f}", true_final_y=f"{truth[1]:.3f}",
                       true_final_error_m=f"{terr:.3f}",
                       localization_error_m=f"{math.hypot(truth[0] - ex, truth[1] - ey):.3f}")
        else:
            terr = err
            notes.append("ground truth unavailable; success uses localisation error")
        # success: Nav2 action succeeded, TRUE position within tolerance, no physical contact
        ok = status == "succeeded" and terr <= self.gp("success_tolerance_m") and rec["collisions"] == 0
        row.update(sim_elapsed_s=f"{sim_dt:.2f}", wall_elapsed_s=f"{wall_dt:.2f}",
                   real_time_factor=f"{sim_dt / wall_dt:.3f}" if wall_dt > 0 else "",
                   actual_path_m=f"{rec['length']:.3f}", final_position_error_m=f"{err:.3f}",
                   final_heading_error_rad=f"{herr:.3f}", collision_count=rec["collisions"],
                   near_collision_count=rec["near_count"], timeout=int(timed_out), action_result=status,
                   success=int(ok), notes="; ".join(notes),
                   total_rotation_rad=f"{rec['rotation']:.3f}",
                   angular_velocity_rms=f"{math.sqrt(rec['w_sq'] / max(1, rec['w_n'])):.3f}",
                   min_scan_range_m=f"{rec['min_range']:.3f}")
        self.get_logger().info(f"[{run_id}] {status} success={int(ok)} sim={sim_dt:.1f}s wall={wall_dt:.1f}s "
                               f"len={rec['length']:.2f}m err={err:.3f}m true_err={terr:.3f}m collisions={rec['collisions']}")
        self.finish(row)

    def finish(self, row):
        path = os.path.expanduser(self.gp("results_csv"))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        with open(path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            if new:
                w.writeheader()
            w.writerow(row)
        self.result_pub.publish(String(data=json.dumps(row)))

    def stop_robot(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main():
    rclpy.init()
    node = GoalAdapter()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.rec is not None:
            node.cancel_event.set()
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
