#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
export PYTHONPATH="$ROOT/ros2_ws/src/maze_dqn${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q ros2_ws/src/maze_dqn/test tests "$@"
