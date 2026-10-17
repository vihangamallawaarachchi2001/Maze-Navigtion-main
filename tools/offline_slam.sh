#!/usr/bin/env bash
# Re-run SLAM Toolbox on a recorded mapping bag with a given parameter file, save and score the map.
# usage: tools/offline_slam.sh <bag_dir> <slam_params.yaml> <out_prefix> [rate]
BAG=$1; PARAMS=$2; OUT=$3; RATE=${4:-4}
source "$HOME/maze_testing/env.sh"
L="$HOME/maze_testing/logs"; mkdir -p "$(dirname "$OUT")"
python3 "$HOME/maze_testing/tools/tf_relay_drop_map.py" --ros-args -p use_sim_time:=true > "$L/offline_relay.log" 2>&1 & R=$!
"$CONDA_PREFIX/lib/slam_toolbox/async_slam_toolbox_node" --ros-args --params-file "$PARAMS" -p use_sim_time:=true > "$L/offline_slam.log" 2>&1 & S=$!
sleep 4
ros2 lifecycle set /slam_toolbox configure >/dev/null 2>&1; ros2 lifecycle set /slam_toolbox activate >/dev/null 2>&1
ros2 bag play "$BAG" --rate "$RATE" --topics /scan /tf /tf_static /clock --remap /tf:=/tf_bag > "$L/offline_play.log" 2>&1
sleep 5
timeout 60 ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p use_sim_time:=true > /dev/null 2>&1
kill -INT $S $R; sleep 2; kill -9 $S $R 2>/dev/null; sleep 1
ps -p $S >/dev/null && echo "WARNING: slam node still alive"
python3 "$HOME/maze_testing/tools/map_quality.py" "$OUT.yaml"
