# IE4060 Assignment 02: Autonomous navigation of a TurtleBot3 Burger in a Gazebo maze

ROS 2 Jazzy · Gazebo Harmonic · SLAM Toolbox · AMCL · Nav2 (Dijkstra + Regulated Pure Pursuit)
· goal-conditioned Deep Q-Network route planner (PyTorch).

Repository link: `<add GitHub URL here>`

## Layout

```
maps/reference/        geometry-derived map (NOT SLAM) - tests and scoring only
maps/slam/             LiDAR SLAM map used for localisation, Nav2 and the DQN
ros2_ws/src/maze_bringup   world, robot model, launch, params, RViz, goal adapter, trial tools
ros2_ws/src/maze_dqn       DQN environment, network, training, evaluation, planner, tests, checkpoint
tools/                 geometry generator, probes, offline SLAM replay, analysis, process control
results/               trial CSVs, DQN training logs, analysis tables and figures
docs/                  audit, progress log, provenance, architecture, member hand-offs, report
```

## One-time setup (no sudo)

```bash
curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xvj -C ~/.local bin/micromamba
export MAMBA_ROOT_PREFIX=~/micromamba
~/.local/bin/micromamba create -y -n ros_env -c conda-forge -c robostack-jazzy python=3.12 \
  ros-jazzy-ros-base ros-jazzy-rviz2 ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-tf2-tools ros-jazzy-rmw-cyclonedds-cpp colcon-common-extensions numpy matplotlib pyyaml \
  pytorch-cpu pillow pytest git
source ~/maze_testing/env.sh
cd ~/maze_testing/ros2_ws && colcon build        # (not --symlink-install: fails with this setuptools)
source ~/maze_testing/env.sh
```

`env.sh` must be sourced in **every** terminal.

## Daily use (each long-running command in its own terminal)

Terminal 1, simulation (Gazebo GUI; add `headless:=true` for server only):
```bash
source ~/maze_testing/env.sh && ros2 launch maze_bringup simulation.launch.py
```
Terminal 2, localisation + Nav2 + goal adapter + RViz (SLAM map, AMCL starts at the spawn pose):
```bash
source ~/maze_testing/env.sh && ros2 launch maze_bringup localization_navigation.launch.py
```
In RViz, use **2D Goal Pose** to click a destination. To switch the method:
```bash
ros2 param set /goal_adapter method dqn        # or baseline
```
If the robot was moved, use **2D Pose Estimate** first. Ctrl+C in each terminal stops everything.

### Re-mapping (SLAM mode; do not run Nav2 at the same time)
```bash
ros2 launch maze_bringup simulation.launch.py headless:=true
ros2 launch maze_bringup slam.launch.py
ros2 run maze_bringup mapping_drive --ros-args -p use_sim_time:=true      # or teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/maze_testing/maps/slam/maze_slam --ros-args -p use_sim_time:=true
```

### Background control with logs and PIDs (used for the experiments)
```bash
tools/sim_ctl.sh start sim maze_bringup simulation.launch.py headless:=true   # logs/sim.log
tools/sim_ctl.sh start nav maze_bringup localization_navigation.launch.py rviz:=false
tools/sim_ctl.sh stop nav ; tools/sim_ctl.sh stop sim
```

### DQN
```bash
cd ~/maze_testing/ros2_ws/src/maze_dqn
python -m pytest -q test/test_grid.py
python -m maze_dqn.train --config config/dqn.yaml --map ~/maze_testing/maps/slam/maze_slam.yaml --out ~/maze_testing/results/dqn_training
python -m maze_dqn.evaluate --ckpt ~/maze_testing/results/dqn_training/dqn_policy.pt --split ~/maze_testing/results/dqn_training/split.yaml --out ~/maze_testing/results/dqn_training
cp ~/maze_testing/results/dqn_training/dqn_policy.pt checkpoints/ && cd ~/maze_testing/ros2_ws && colcon build
```

### Comparison trials and analysis
```bash
ros2 run maze_bringup run_trials --ros-args -p use_sim_time:=true \
  -p goals_file:=$HOME/maze_testing/ros2_ws/src/maze_bringup/config/evaluation_goals.yaml -p repeats:=3
python tools/analyze_results.py --trials results/trials.csv \
  --goals ros2_ws/src/maze_bringup/config/evaluation_goals.yaml --map maps/slam/maze_slam.yaml \
  --train results/dqn_training --out results/analysis
```

See `docs/architecture.md`, `docs/map_provenance.md` and `docs/IMPLEMENTATION_REPORT.md`.
