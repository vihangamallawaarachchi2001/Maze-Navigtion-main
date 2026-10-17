# Source from any directory: source /path/to/project/env.sh
export MAZE_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy is missing. Run: bash $MAZE_PROJECT_ROOT/scripts/setup.sh" >&2
  return 1
fi
source /opt/ros/jazzy/setup.bash
if [ -d "$MAZE_PROJECT_ROOT/.venv/lib/python3.12/site-packages" ]; then
  export PYTHONPATH="$MAZE_PROJECT_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RCUTILS_COLORIZED_OUTPUT=1
if [ -f "$MAZE_PROJECT_ROOT/ros2_ws/install/setup.bash" ]; then
  source "$MAZE_PROJECT_ROOT/ros2_ws/install/setup.bash"
fi
