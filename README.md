# IE4060 Assignment 02 — Autonomous Maze Navigation

A TurtleBot3 Burger simulation that maps a maze and navigates to destinations selected
on a map. Built with **ROS 2 Jazzy, Gazebo Harmonic, SLAM Toolbox, AMCL, Nav2 and
Deep Q-learning** on Ubuntu 24.04.

**Repository URL:** add your GitHub URL before submission.

## Start here

For a normal demonstration, follow [Run the project](#run-the-project) and
[Select goals and switch methods](#select-goals-and-switch-methods).
You do not need to install dependencies, rebuild, remap or train the DQN every time.

| Guide or artifact | Location |
|---|---|
| Final six-page report | [deliverables/final_report.pdf](deliverables/final_report.pdf) |
| Editable report | [deliverables/final_report.md](deliverables/final_report.md) |
| Beginner explanation | [deliverables/FINAL_SUMMARY.md](deliverables/FINAL_SUMMARY.md) |
| Detailed verification and limitations | [deliverables/FINAL_VERIFICATION.md](deliverables/FINAL_VERIFICATION.md) |
| VIVA questions and answers | [deliverables/viva_preparation.md](deliverables/viva_preparation.md) |
| Real application screenshots | [evidence/screenshots/](evidence/screenshots/) |
| Recorded measurements and plots | [results/current/](results/current/) |

## How the project works

The basic sequence is:

**LiDAR scans → mapping/localization → selected goal → route planning → robot movement**

| Component | Purpose |
|---|---|
| Gazebo | Simulates the maze, robot, sensors and physics |
| ROS 2 | Exchanges sensor data, commands and navigation results between nodes |
| LiDAR | Measures distances to nearby walls |
| SLAM Toolbox | Builds an occupancy map from scans and motion information |
| AMCL | Estimates the robot's position on an existing map |
| Nav2 | Plans paths, checks obstacles and controls movement |
| RViz | Displays the map, robot, scans and paths; accepts clicked goals |
| DQN | Uses a trained neural network to choose a route through a map-derived grid |

The maze reference is [image.png](image.png): six by six cells with 0.5 m spacing.
The included robot is a simplified model using Burger dimensions.

There are two separate operating modes:

- **Mapping:** SLAM builds a map while the robot explores.
- **Navigation:** AMCL localizes on a saved map while Nav2 drives to a goal.

**Do not run SLAM and AMCL/navigation together.** Both would try to provide the
`map → odom` transform.

### A* and DQN

The `baseline` method uses Nav2's **Smac A*** planner to search the map.
The `dqn` method uses a trained **Deep Q-Network** to choose moves between safe
locations in the maze. Both use the same Nav2 controller to execute the route.
DQN chooses a route; it does not directly predict wheel speeds. There is no
classical route-planning fallback inside the DQN planner.

## Environment and first-time setup

Requirements: Ubuntu 24.04, Python 3.12, ROS 2 Jazzy and Gazebo Harmonic.
Use terminals inside the **Ubuntu VM desktop** for Gazebo and RViz. An SSH terminal
does not automatically have access to the desktop display.

On the current VM, installation is already complete. On a fresh machine:

```bash
cd /home/vihanga/Maze-Navigtion-main
bash scripts/setup.sh
bash scripts/build.sh
```

If you cloned the repository elsewhere, replace the project path in this guide.
The scripts themselves find the project root automatically.

Setup requests your sudo password in the terminal, installs native ROS/Gazebo
packages, and installs CPU PyTorch in `.venv`. It does not perform a system upgrade.
The simulated Burger is included; no external robot model download is needed.

After changing source, launch files or configuration, rebuild:

```bash
cd /home/vihanga/Maze-Navigtion-main
bash scripts/build.sh
```

## Run the project

Use **two separate terminals**. Source `env.sh` in every new terminal.
A running launch command stays active; it will not return to the normal prompt.

### Terminal 1 — Gazebo simulation

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 launch maze_bringup simulation.launch.py render_engine:=ogre
```

Wait until Gazebo shows the maze and robot.

**Keep `render_engine:=ogre` on this VM.** The default OGRE2 renderer crashed in
Mesa during verification; the OGRE fallback worked with software rendering.

### Terminal 2 — Localization, navigation and RViz

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 launch maze_bringup localization_navigation.launch.py
```

Wait for the maze map, robot and red LiDAR points to appear in RViz. Keep both
terminals running. Normal navigation loads the supplied map in `maps/slam/`.

## Select goals and switch methods

In RViz:

1. Select **2D Goal Pose**.
2. Click a clear corridor centre.
3. Hold and drag slightly to choose the final orientation.
4. Release the mouse and watch the robot navigate.
5. Wait for the goal to finish before selecting the next one or switching methods.

Avoid walls, unknown areas and locations very close to obstacles.
Use **2D Pose Estimate** only when the displayed robot pose is incorrect, such as
after moving the simulated robot. It corrects localization; it does not move the robot.

The default planning method is A*. To switch, open **Terminal 3**:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 param set /goal_adapter method dqn
```

Select another goal in RViz to use DQN. Return to A* with:

```bash
ros2 param set /goal_adapter method baseline
```

### Three demonstration goals

Coordinates are in metres in the map frame; yaw is in radians.

| Goal | X | Y | Final yaw | Purpose |
|---|---:|---:|---:|---|
| G1 | 0.25 | -1.25 | 0 | Short corridor |
| G2 | -1.25 | 0.75 | 1.5708 | Route with turns |
| G3 | 1.25 | 0.25 | 0 | Longer, more complex route |

The comparison experiments start at `(-1.25, -1.25, 0)` before each trial.
Consecutive manual clicks start from the robot's current position, so travel
times will differ from those in the report.

### Stop or restart

Press **Ctrl+C in Terminal 2**, then **Ctrl+C in Terminal 1**. Close any separate
RViz windows. To reset the demonstration, launch both terminals again using the
commands above. Avoid starting duplicate simulation or navigation launches.

## Run the repeatable comparison

Keep simulation and navigation running. In Terminal 3:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 run maze_bringup run_trials --ros-args \
  -p use_sim_time:=true \
  -p goals_file:="$MAZE_PROJECT_ROOT/ros2_ws/src/maze_bringup/config/evaluation_goals.yaml" \
  -p repeats:=1
```

This runs three goals with each method: **six trials**. The runner teleports the
robot to the common start, resets AMCL and clears costmaps before each trial.
Those resets occur outside the measured navigation time. Do not click extra goals
while the runner is active. Use `-p repeats:=3` for three runs per method and goal.

Results append to `results/current/trials.csv`. To preserve a separate experiment,
start navigation with a different output filename instead of its normal launch:

```bash
ros2 launch maze_bringup localization_navigation.launch.py \
  results_csv:="$MAZE_PROJECT_ROOT/results/current/rehearsal_trials.csv"
```

Then run the trial command. Choose another filename for another experiment.

Generate comparison plots from the selected CSV:

```bash
python3 tools/analyze_results.py \
  --trials results/current/trials.csv \
  --goals ros2_ws/src/maze_bringup/config/evaluation_goals.yaml \
  --map maps/slam/maze_slam.yaml \
  --train results/dqn_training \
  --out results/current/analysis
```

Replace `--trials` and `--out` when analysing a separate rehearsal.
The `--train` directory above contains the **inherited** full training log.
The runner publishes to the same `/clicked_goal` interface as RViz, but an automated
trial is different from selecting a goal with the GUI. Demonstrate actual clicks
for the VIVA as well.

## Build a new map with SLAM

This is a separate demonstration. Stop navigation and simulation first, then
restart the simulation at its original spawn using the Terminal 1 command.
Do not start the navigation launch during mapping.

### Terminal 2 — SLAM Toolbox

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 launch maze_bringup slam.launch.py
```

### Terminal 3 — Mapping tour

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
ros2 run maze_bringup mapping_drive --ros-args \
  -p use_sim_time:=true \
  -p max_speed:=0.18
```

The robot follows 55 waypoints while SLAM builds the map from LiDAR.
The verified tour took approximately six minutes of wall time. Wait for
`waypoint 55/55 reached` before saving the completed map.

### Terminal 4 — Mapping display and map saving

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
rviz2 -d ros2_ws/src/maze_bringup/rviz/mapping.rviz \
  --ros-args -p use_sim_time:=true
```

After the tour, use another sourced terminal to save a new map without overwriting
the verified map already in `maps/current/`:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
mkdir -p maps/rehearsal
ros2 run nav2_map_server map_saver_cli \
  -f "$MAZE_PROJECT_ROOT/maps/rehearsal/maze" --ros-args \
  -p use_sim_time:=true \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
```

This writes `maze.yaml` and `maze.pgm`. The transient-local setting and timeout
avoid the missed-map-publication problem observed on this VM.

### Which map is which?

| Directory | Meaning |
|---|---|
| `maps/reference/` | Generated from wall geometry; **not a SLAM result** |
| `maps/slam/` | Supplied map used by normal navigation and the trained DQN |
| `maps/current/` | Fresh SLAM map produced during verification |
| `maps/rehearsal/` | Your new map when following the commands above |

To navigate using your new map, stop SLAM and restart simulation, then use:

```bash
ros2 launch maze_bringup localization_navigation.launch.py \
  map:="$MAZE_PROJECT_ROOT/maps/rehearsal/maze.yaml" \
  dqn_checkpoint:=none
```

This starts **A* navigation** without loading the old DQN checkpoint. Set the
initial pose in RViz to match the robot on the new map. The old checkpoint may
reject the new map because its lattice signature differs; retrain on that map
before using DQN with it.

## Deep Q-learning: evaluate or train

You do not need training for the normal demonstration: a trained checkpoint is
already included. Grid training/evaluation can run without Gazebo.

Prepare a terminal:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
export PYTHONPATH="$MAZE_PROJECT_ROOT/ros2_ws/src/maze_dqn:$PYTHONPATH"
```

Evaluate the supplied policy on its saved training/held-out split:

```bash
python3 -m maze_dqn.evaluate \
  --ckpt ros2_ws/src/maze_dqn/checkpoints/dqn_policy.pt \
  --map maps/slam/maze_slam.yaml \
  --split results/dqn_training/split.yaml \
  --out results/current/dqn_eval
```

Start a full new training run:

```bash
python3 -m maze_dqn.train \
  --config ros2_ws/src/maze_dqn/config/dqn.yaml \
  --map maps/slam/maze_slam.yaml \
  --out results/current/dqn_training
```

The configuration requests 30,000 episodes. Training saves a checkpoint,
hyperparameters, pair split and episode CSV. Use a new output directory if you
want to keep a previous training run.

After training finishes, evaluate the new checkpoint using its own split:

```bash
python3 -m maze_dqn.evaluate \
  --ckpt results/current/dqn_training/dqn_policy.pt \
  --map maps/slam/maze_slam.yaml \
  --split results/current/dqn_training/split.yaml \
  --out results/current/dqn_training_eval
```

Only after evaluation, select that checkpoint in the navigation launch with
`dqn_checkpoint:=/absolute/path/to/dqn_policy.pt`. Stop the previous navigation
launch first. Training a checkpoint does not automatically replace the supplied one.

The network uses an 80-element state, two 256-unit hidden layers and four movement
actions. Replay, epsilon-greedy exploration, a target network and Double DQN updates
support learning. Rewards favour goal completion and progress, and penalize
blocked moves and unnecessary steps. See the report for the detailed formulation.

## Tests and runtime checks

Run the source tests:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
bash scripts/run_tests.sh
```

Run ROS package tests:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
cd ros2_ws
colcon test --python-testing pytest
colcon test-result --verbose
```

Explicitly selecting pytest matters: the default command initially discovered
zero tests. The eight package tests are a subset of the eleven root tests.

With simulation and either SLAM or navigation running, check real message traffic:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
python3 tools/check_runtime.py
```

This checks scans, odometry, clock, transforms, map messages and ROS publish/subscribe
delivery. Unit tests alone do not demonstrate successful robot navigation.

## Verified results and limitations

The completed audit recorded:

| Check | Observed result |
|---|---|
| Normal and independent clean builds | Both packages built |
| Root automated suite | 11 tests passed |
| Package tests through colcon | 8 tests passed |
| Fresh SLAM | 55/55 waypoints completed; map saved |
| A* comparison goals | 3/3 succeeded |
| DQN comparison goals | 3/3 succeeded with the supplied checkpoint |
| Separate actual RViz mouse-goal demonstration | All three goals succeeded |
| Supplied DQN held-out grid evaluation | 189/189 succeeded |
| New 300-episode training smoke run | 21/189 held-out successes; not converged |

| Goal | A* simulation time | DQN simulation time |
|---|---:|---:|
| G1 | 11.09 s | 10.16 s |
| G2 | 26.44 s | 22.57 s |
| G3 | 84.34 s | 72.56 s |

These are recorded audit results, not a promise that every new run will match.
DQN was faster in those trials; A* travelled less on G2/G3. The paired comparison
has one run per case, so it does not establish statistical superiority.
New runs append to the CSV; use the saved verification document for the original
experiment records.

Important limitations to explain honestly:

- Odometry uses Gazebo's model pose plus noise rather than wheel integration.
- Zero recorded chassis contacts does not certify contact-free motion of every part.
- SLAM loop closing is disabled in this configuration.
- DQN held-out tests use different start/goal pairs in the same maze, not new mazes.
- Full training convergence was not rerun; the successful supplied policy and the
  weak short-training checkpoint are separate artifacts.
- The reference-distance/actual-distance ratio can exceed one because of goal
  tolerance and different clearance/endpoint assumptions. It is an approximate
  diagnostic, not a bounded optimality score.
- The original assignment PDF was unavailable; the supplied requirements and maze
  image were used.

## Presentation and VIVA guide

A simple eight-slide structure:

| Slide | Main point |
|---|---|
| 1. Objective | Navigate the robot to a user-selected destination |
| 2. Maze and robot | Show the reference figure and real Gazebo screenshot |
| 3. Architecture | Explain sensing, mapping/localization, planning and control |
| 4. SLAM | Show the fresh map and explain how LiDAR builds it |
| 5. A* | Explain map search and Nav2 path tracking |
| 6. DQN | Explain state, actions, rewards, replay and target network |
| 7. Results | Show real comparison graphs and measured outcomes |
| 8. Limitations and improvements | More repetitions, better odometry and unseen-maze tests |

Suggested opening:

> Our project implements autonomous navigation for a TurtleBot3 Burger in a Gazebo
> maze using ROS 2 Jazzy. LiDAR supports mapping, AMCL provides localization, and
> we compare A* route planning with a trained Deep Q-Network. Both methods use
> Nav2's controller to move the robot.

For the live demonstration:

1. Launch Gazebo and navigation using the two-terminal sequence.
2. Show the maze, robot, map and live LiDAR scans.
3. Select a goal using A* and wait for completion.
4. Switch to DQN, then select another goal.
5. Demonstrate the remaining goal and show the saved results/graphs.
6. Show the fresh SLAM evidence; run live mapping separately if time permits.

Know the difference between mapping and localization, A* and DQN, and a ROS topic
and action. Explain `map → odom → base_footprint` and simulation time.
Read [viva_preparation.md](deliverables/viva_preparation.md) for detailed answers.

Before submission, add student names/IDs and your GitHub URL, check the original
assignment PDF, and rehearse the three-click demonstration. After editing the
report, regenerate its PDF:

```bash
cd /home/vihanga/Maze-Navigtion-main
source env.sh
python3 tools/build_report.py
pdfinfo deliverables/final_report.pdf
```

Confirm that it remains within the required 5–7 pages.

## Troubleshooting

| Problem | First action |
|---|---|
| `ros2: command not found` | Change to the project folder and run `source env.sh` |
| Gazebo crashes | Include `render_engine:=ogre`; use the VM desktop terminal |
| GUI does not open from SSH | Run the graphical launch in a VM desktop terminal |
| Robot does not move | Confirm navigation is running; choose a clear corridor centre |
| Robot position is wrong in RViz | Correct it with 2D Pose Estimate or restart both launches |
| Map/TF becomes inconsistent | Stop duplicate launches; run either SLAM or AMCL/navigation |
| New changes do not appear | Rebuild, source env.sh again, then restart the launches |
| DQN rejects a changed map | Use the supplied map/checkpoint pair or retrain on the new map |
| Map saver misses the map | Use the transient-local subscription and timeout shown above |
| `torch` cannot be imported | Run setup if needed, then source env.sh to expose the local environment |
| Screenshots are black or denied | Use the VM screenshot shortcut; Wayland restricts capture |

Server-only simulation is available with `headless:=true`, but LiDAR still needs
a working renderer. Desktop OGRE is the verified configuration on this VM;
headless rendering after recovery was not independently verified.

## Project layout and evidence

```text
image.png                    Supplied maze reference
ros2_ws/src/maze_bringup/     World, robot model, launch/configuration and trial tools
ros2_ws/src/maze_dqn/         DQN, grid environment, training, evaluation and checkpoint
maps/                        Separate reference, supplied SLAM and fresh maps
scripts/                     Setup, build and test commands
tools/                       Runtime checks, geometry, analysis and report utilities
tests/                       Root project checks
results/current/             Fresh measurements, evaluations and plots
evidence/logs/               Actual build, test and runtime logs
evidence/screenshots/        Real Gazebo/RViz window captures
deliverables/                Report, beginner guide, verification and VIVA notes
docs/PROGRESS.md              Work completed and remaining submission tasks
```

The original import is preserved in Git commit `1952a2e`. Older documents and
`results/*.csv` describe earlier work on another machine. Consult
[FINAL_VERIFICATION.md](deliverables/FINAL_VERIFICATION.md) for the audit's evidence.

Screenshots were captured from real running applications. Some older navigation
captures retain a Dijkstra display label although the configured planner is Smac
A*. Orange DQN overlays can remain from earlier trials; the current baseline path
is blue. CSVs and action logs establish the corresponding goal results.

For optional XComposite capture, `tools/capture_windows.py` takes pairs of X window
IDs and output filenames. Obtain IDs with `xwininfo -root -tree`; a working
DISPLAY/XAUTHORITY is required. Manual desktop screenshots are sufficient for
normal presentation preparation.
