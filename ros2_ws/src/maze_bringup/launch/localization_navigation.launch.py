"""Members 2-4: static map + AMCL localisation, Nav2 navigation, clicked-goal adapter, RViz.

Run after simulation.launch.py, with SLAM STOPPED (AMCL owns map->odom here).

  ros2 launch maze_bringup localization_navigation.launch.py            # baseline (Dijkstra) mode
  ros2 launch maze_bringup localization_navigation.launch.py method:=dqn
  ros2 param set /goal_adapter method dqn                                 # switch at runtime

Velocity chain (exactly one publisher on /cmd_vel during navigation):
  controller_server / behavior_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> ros_gz bridge
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOC_NODES = ["map_server", "amcl"]
NAV_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator", "velocity_smoother"]


def _setup(context):
    pkg = get_package_share_directory("maze_bringup")
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    params = lc("params_file")
    map_yaml = lc("map")
    common = {"use_sim_time": True}
    tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    to_nav = tf + [("cmd_vel", "cmd_vel_nav")]

    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[params, common, {"yaml_filename": map_yaml}], remappings=tf),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[params, common], remappings=tf),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             output="screen", parameters=[common, {"autostart": True, "node_names": LOC_NODES,
                                                  "bond_timeout": 0.0}]),
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[params, common], remappings=to_nav),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=[params, common], remappings=tf),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output="screen",
             parameters=[params, common], remappings=to_nav),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[params, common], remappings=tf),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother", name="velocity_smoother",
             output="screen", parameters=[params, common],
             remappings=tf + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[common, {"autostart": True, "node_names": NAV_NODES,
                                                  # bonds disabled: heartbeats timed out under VM CPU load and
                                                  # the manager deactivated Nav2 mid-goal (docs/progress.md)
                                                  "bond_timeout": 0.0}]),
        Node(package="maze_bringup", executable="goal_adapter", name="goal_adapter", output="screen",
             parameters=[common, {
                 "method": lc("method"),
                 "map_yaml": map_yaml,
                 "map_source": lc("map_source"),
                 "world_revision": lc("world_revision"),
                 "dqn_checkpoint": lc("dqn_checkpoint"),
                 "results_csv": lc("results_csv"),
             }]),
    ]
    if lc("rviz").lower() == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", os.path.join(pkg, "rviz", "maze.rviz")], parameters=[common]))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory("maze_bringup")
    dqn_share = get_package_share_directory("maze_dqn")
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(pkg, "maps", "slam", "maze_slam.yaml")),
        DeclareLaunchArgument("map_source", default_value="slam"),
        DeclareLaunchArgument("world_revision", default_value="maze_v1_18seg_t0.05_h0.30"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(pkg, "config", "nav2.yaml")),
        DeclareLaunchArgument("method", default_value="baseline", description="baseline | dqn"),
        DeclareLaunchArgument("dqn_checkpoint",
                              default_value=os.path.join(dqn_share, "checkpoints", "dqn_policy.pt")),
        DeclareLaunchArgument("results_csv",
                              default_value=os.path.expanduser("~/maze_testing/results/trials.csv")),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
