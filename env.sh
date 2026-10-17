# Source this in every terminal:  source ~/maze_testing/env.sh
export MAMBA_ROOT_PREFIX="$HOME/micromamba"
eval "$("$HOME/.local/bin/micromamba" shell hook -s bash)"
micromamba activate ros_env
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RCUTILS_COLORIZED_OUTPUT=1
if [ -f "$HOME/maze_testing/ros2_ws/install/setup.bash" ]; then
  source "$HOME/maze_testing/ros2_ws/install/setup.bash"
fi
