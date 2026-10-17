#!/usr/bin/env bash
set -e
source "$(dirname "$0")/../env.sh"
cd "$MAZE_PROJECT_ROOT/ros2_ws"
colcon build
