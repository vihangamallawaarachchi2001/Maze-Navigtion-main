# IE4060 Assignment 02 — TurtleBot3 maze navigation

ROS 2 Jazzy, Gazebo Harmonic, SLAM Toolbox, AMCL, Nav2 Smac A* and a PyTorch
Double DQN route planner. Ubuntu 24.04, Python 3.12. Repository URL: **add your GitHub URL**.

The supplied `image.png` is the maze reference. The original import is preserved
in Git commit `1952a2e`. Imported `results/*.csv` and older documents describe
another machine. **Use `results/current/` and `deliverables/FINAL_VERIFICATION.md`
for this audit's measurements and limitations.**

## Install and build

From the project folder (paths may contain spaces):

```bash
bash scripts/setup.sh                 # apt needs your sudo password; CPU torch in .venv
bash scripts/build.sh
source env.sh                        # do this in every new terminal
```

Setup adds the official ROS apt source, installs missing dependencies and does
not upgrade the system. Native Jazzy installs its matched Harmonic through
`ros-jazzy-ros-gz`. The included Burger is a simplified primitive model using
Burger dimensions; no external TurtleBot download is required.

## Run the demonstration

Use separate terminals, each starting with `source env.sh` from this folder:

```bash
ros2 launch maze_bringup simulation.launch.py render_engine:=ogre
ros2 launch maze_bringup localization_navigation.launch.py
```

`ogre` is the tested fallback for this VM: default `ogre2` crashed in Mesa.
On a working GPU you can omit the override. For server only, add `headless:=true`;
GPU LiDAR still needs a working rendering context. Run graphical commands in the
VM desktop terminal; SSH does not automatically set DISPLAY/XAUTHORITY.

In RViz use **2D Goal Pose**, click and drag to choose position and orientation.
It publishes `/clicked_goal`. Use **2D Pose Estimate** if the robot has been moved.

```bash
ros2 param set /goal_adapter method dqn      # switch to learned route planning
ros2 param set /goal_adapter method baseline # switch back to A*
```

Three goals, in metres in map frame: G1 `(0.25,-1.25,0)`, G2 `(-1.25,0.75,1.5708)`,
G3 `(1.25,0.25,0)`. Trial start is `(-1.25,-1.25,0)`.
Ctrl+C stops each launch. Do not run mapping and AMCL/navigation together.

## Reproducible trials and plots

With simulation and navigation active:

```bash
ros2 run maze_bringup run_trials --ros-args -p use_sim_time:=true \
  -p goals_file:="$MAZE_PROJECT_ROOT/ros2_ws/src/maze_bringup/config/evaluation_goals.yaml" -p repeats:=3
python3 tools/analyze_results.py --trials results/current/trials.csv \
  --goals ros2_ws/src/maze_bringup/config/evaluation_goals.yaml \
  --map maps/slam/maze_slam.yaml --train results/dqn_training --out results/current/analysis
```

The runner teleports to the common start and resets AMCL between trials; these
operations occur outside measurement. It publishes goals on the clicked-goal
interface, so repeat the actual RViz mouse demonstration for your VIVA. Logs
append to `results/current/trials.csv`; use a new `results_csv:=...` launch argument
for a separate experiment. Default trial limit: 120 simulation seconds and 900
wall seconds. Near-obstacle scan events are distinct from physical chassis contacts.

## Mapping

Start a fresh simulation at the spawn pose. Stop navigation first:

```bash
ros2 launch maze_bringup slam.launch.py
ros2 run maze_bringup mapping_drive --ros-args -p use_sim_time:=true
mkdir -p maps/current
ros2 run nav2_map_server map_saver_cli -f "$MAZE_PROJECT_ROOT/maps/current/maze" --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
```

Open RViz with `rviz2 -d ros2_ws/src/maze_bringup/rviz/maze.rviz --ros-args -p use_sim_time:=true`.
Load a saved map using `localization_navigation.launch.py map:=/absolute/path/map.yaml`.
A changed map needs a retrained DQN checkpoint; its lattice signature is checked.
`maps/reference/` is generated geometry, never SLAM evidence. The original
`maps/slam/` is preserved separately from fresh mapping.

## Deep Q-learning

```bash
export PYTHONPATH="$MAZE_PROJECT_ROOT/ros2_ws/src/maze_dqn:$PYTHONPATH"
python3 -m maze_dqn.train --config ros2_ws/src/maze_dqn/config/dqn.yaml \
  --map maps/slam/maze_slam.yaml --out results/current/dqn_training
python3 -m maze_dqn.evaluate --ckpt ros2_ws/src/maze_dqn/checkpoints/dqn_policy.pt \
  --map maps/slam/maze_slam.yaml --split results/dqn_training/split.yaml --out results/current/dqn_eval
```

Training uses a map-derived grid; the DQN chooses routes and Nav2's shared
controller executes them in Gazebo. DQN does not directly learn wheel velocities.
No classical route fallback is used. The supplied checkpoint is preserved; new
training writes to a separate directory. Select a new checkpoint with
`dqn_checkpoint:=/absolute/path/dqn_policy.pt` in the navigation launch.

## Tests, evidence and limitations

```bash
source env.sh
bash scripts/run_tests.sh
cd ros2_ws && colcon test --python-testing pytest && colcon test-result --verbose
```

Logs: `evidence/logs/`; genuine window captures: `evidence/screenshots/`;
report and VIVA notes: `deliverables/`. Capture manually with the VM screenshot
shortcut if Wayland blocks screenshots. `tools/capture_windows.py` captures
XComposite application buffers: pass pairs of X window IDs and PNG filenames
(obtain IDs with `xwininfo -root -tree`). It requires DISPLAY and XAUTHORITY.

Odometry uses Gazebo model pose plus noise, not wheel integration. Loop closing
is disabled in the existing SLAM configuration. These simplify the experiment
and limit transfer to real robots. The bumper covers chassis contact only;
zero recorded contacts does not prove every robot part avoided every object.
Grid hold-out evaluation uses one maze and does not prove generalization to new
mazes. Original assignment PDF is absent; the user supplied the maze image.
