"""Member 2: SLAM Toolbox (online async) with the project parameter file. Run after simulation.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("maze_bringup")
    slam = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam),
        launch_arguments={"use_sim_time": "true",
                          "slam_params_file": os.path.join(pkg, "config", "slam_toolbox.yaml")}.items())])
