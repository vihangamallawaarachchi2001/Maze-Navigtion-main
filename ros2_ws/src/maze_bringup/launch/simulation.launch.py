"""Member 1: one-command simulation bring-up.

Starts Gazebo Harmonic with the assignment maze, spawns the Burger at the validated
safe pose from config/safe_spawn.yaml, starts the ROS<->Gazebo bridge and
robot_state_publisher. Ctrl+C shuts every process down together.

TF authorities:
  odom -> base_footprint             : Gazebo OdometryPublisher (pose-derived, bridged /tf)
  base_footprint -> base_link -> ... : robot_state_publisher (URDF)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    pkg = get_package_share_directory("maze_bringup")
    world = os.path.join(pkg, "worlds", "assignment_maze.sdf")
    robot_sdf = os.path.join(pkg, "models", "turtlebot3_burger", "model.sdf")
    with open(os.path.join(pkg, "config", "safe_spawn.yaml")) as f:
        spawn = yaml.safe_load(f)
    with open(os.path.join(pkg, "urdf", "turtlebot3_burger.urdf")) as f:
        urdf = f.read()

    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    engine = LaunchConfiguration("render_engine").perform(context)
    gz_args = f'-r -v 2 --render-engine {engine} {"-s " if headless else ""}"{world}"'

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
    )
    spawn_robot = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-world", "assignment_maze", "-file", robot_sdf, "-name", "turtlebot3_burger",
                   "-x", str(spawn["safe_x"]), "-y", str(spawn["safe_y"]),
                   "-z", str(spawn["safe_z"]), "-Y", str(spawn["safe_yaw"])],
    )
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="maze_bridge", output="screen",
        parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"), "use_sim_time": True}],
    )
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher", output="screen",
        parameters=[{"robot_description": urdf, "use_sim_time": True}],
    )
    return [gz, spawn_robot, bridge, rsp]


def generate_launch_description():
    pkg = get_package_share_directory("maze_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo server only (no GUI)"),
        DeclareLaunchArgument("render_engine", default_value="ogre2",
                              description="ogre2, or ogre for VM software-rendering fallback"),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg, "models")),
        OpaqueFunction(function=_setup),
    ])
