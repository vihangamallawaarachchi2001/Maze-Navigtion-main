#!/usr/bin/env bash
# Start/stop owned background ROS launches with PID tracking and predictable logs.
# usage: tools/sim_ctl.sh start <name> <ros2 launch args...> | stop <name> | status

D="$HOME/maze_testing/logs"; mkdir -p "$D"
case "$1" in
  start) name=$2; shift 2
    source "$HOME/maze_testing/env.sh"
    setsid nohup ros2 launch "$@" > "$D/$name.log" 2>&1 &
    echo $! > "$D/$name.pid"; echo "started $name pid $(cat "$D/$name.pid") log $D/$name.log";;
  stop) name=$2; f="$D/$name.pid"
    [ -f "$f" ] || { echo "no pid for $name"; exit 0; }
    pgid=$(ps -o pgid= -p "$(cat "$f")" 2>/dev/null | tr -d ' ')
    if [ -n "$pgid" ]; then kill -INT -- "-$pgid"; for i in $(seq 1 20); do ps -p "$(cat "$f")" >/dev/null || break; sleep 0.5; done
      ps -p "$(cat "$f")" >/dev/null && kill -TERM -- "-$pgid"
      sleep 3; pgrep -g "$pgid" >/dev/null && kill -KILL -- "-$pgid"; fi
    rm -f "$f"
    # gz sim (ruby) may outlive the launch group: stop only servers running OUR installed world
    [ "$name" = sim ] && for p in $(pgrep -f "gz sim .*maze_bringup/worlds/assignment_maze.sdf"); do kill -TERM "$p"; done
    [ "$name" = sim ] && { sleep 2; for p in $(pgrep -f "gz sim .*maze_bringup/worlds/assignment_maze.sdf"); do kill -KILL "$p"; done; }
    echo "stopped $name";;
  status) for f in "$D"/*.pid; do [ -f "$f" ] && { n=$(basename "$f" .pid); ps -p "$(cat "$f")" >/dev/null && echo "$n running" || echo "$n dead"; }; done;;
esac
