# How this project was implemented, step by step

This document records, in order, how the IE4060 Assignment 02 project was built on this machine: what was
done, why, which commands were used, what went wrong and how it was fixed. Every number comes from a run on
this VM on 16 October 2026. Detailed logs are in `docs/progress.md`.

Project root: `~/maze_testing` (git repository). Activate the environment in every terminal with
`source ~/maze_testing/env.sh`.

---

## Step 0: Audit the machine (Stage 0)

1. **Read the pack:** `AI_MASTER_PROMPT.md`, `START_HERE.md`, `PROJECT_STATE.md`, `MAZE_REFERENCE.md` and the
   brief PDF.
2. **Inspect the VM:**
   - Ubuntu 22.04.5 in VirtualBox, 1 vCPU, 1.9 GB RAM, 8.5 GB free disk.
   - No ROS (`/opt/ros` missing), no `~/maze_testing`, no `~/turtlebot3_ws`, no pip, no git.
   - `sudo` needs a password.
3. **Conclusion:** the historical project described in `PROJECT_STATE.md` does not exist here. ROS 2 Jazzy
   has no apt packages for Ubuntu 22.04, so the project had to be built fresh, in user space.
4. **Decisions with you:**
   - You raised the VM to 5 vCPU / 8 GB RAM.
   - The first install filled the disk (it needs about 10–11 GB). I stopped it at 99 % and freed 1.5 GB.
   - You grew the VirtualBox disk to 50 GB and ran `growpart` / `resize2fs` yourself, since that needs your
     sudo password.

## Step 1: Install the software stack without sudo

1. Downloaded micromamba to `~/.local/bin/micromamba`.
2. Created the conda environment `ros_env` from the `conda-forge` + `robostack-jazzy` channels. It contains
   ROS 2 Jazzy, rviz2, navigation2, nav2_bringup, slam_toolbox, ros_gz (Gazebo Harmonic, gz-sim 8.10),
   robot_state_publisher, tf2_tools, colcon, numpy, matplotlib, pytorch-cpu 2.14, pillow, pytest and git.
   The exact command is in `README.md`.
3. Wrote `~/maze_testing/env.sh`, which activates the environment, sources the workspace overlay and sets
   `ROS_DOMAIN_ID=42` and localhost-only discovery.
4. Created the git repository with a `.gitignore` for build, install, log and bag folders, and committed a
   baseline.

Gotcha: `colcon build --symlink-install` fails with this setuptools ("option --editable not recognized"),
so plain `colcon build` is used.

## Step 2: Reproduce the maze (Member 1, Task 1)

1. **Checked the maze description against Figure 1.** I rendered page 2 of the brief with `pdftoppm` and
   measured how much of each grid edge is dark: about 1.00 means a solid wall, about 0.75 a dotted guide.
   - All horizontal walls matched `MAZE_REFERENCE.md`.
   - One vertical wall was missing from the reference: x = 1.0 m, y 0.5 → 1.0 m. I added it as
     `upper_right_v`, giving 18 segments.
2. **Wrote `tools/maze_geometry.py`** as the single source of truth for the wall segments. It generates
   - the Gazebo world `ros2_ws/src/maze_bringup/worlds/assignment_maze.sdf` (walls 0.05 m × 0.30 m, plus the
     physics, sensors, contact and IMU systems);
   - a **reference geometry map** `maps/reference/ground_truth_maze.{pgm,yaml}`, labelled NOT SLAM;
   - self-checks: a known wall point is occupied and a known cell centre is free.
3. **Checked connectivity** (`scratch/maze_topology.py`): all 36 cells are reachable, with 3 dead ends.

## Step 3: Robot model, bridge, TF and one-command startup (Member 1)

1. **Robot model.** `models/turtlebot3_burger/model.sdf` (Gazebo) and `urdf/turtlebot3_burger.urdf`
   (robot_state_publisher) use the same ROBOTIS Burger dimensions:
   - wheel radius 0.033 m, separation 0.160 m;
   - `base_link` 0.010 m above `base_footprint`;
   - `base_scan` at (−0.032, 0, 0.172) from `base_link`;
   - LiDAR: 360 beams, 0.12–3.5 m, 5 Hz;
   - contact sensor on the chassis, used to count collisions.
2. **Bridge.** `config/bridge.yaml` connects clock, scan, odom, tf, joint_states, imu and bumper (Gazebo →
   ROS) and `cmd_vel` (ROS → Gazebo, `geometry_msgs/Twist`).
3. **Safe spawn.** `config/safe_spawn.yaml` puts the robot at (−1.25, −1.25, 0). Wall clearance there is
   0.225 m, more than the 0.124 m circumscribed radius.
4. **Startup.** `launch/simulation.launch.py` starts Gazebo, spawns the robot at the safe pose, and starts
   the bridge and robot_state_publisher. Ctrl+C stops all of them.
5. **Background control.** `tools/sim_ctl.sh start|stop|status` runs launches in the background with PID
   files and logs in `logs/`.
6. **Verification:**
   - Live `/clock`, `/scan` (frame `base_scan`), `/odom` (`odom → base_footprint`), `/tf`, `/joint_states`.
   - `/cmd_vel` is subscribed only by the bridge.
   - `motion_test` passed: 0.15 m/s for 3 s moved 0.4365 m in odometry and 0.437 m in Gazebo, then stopped.

## Step 4: Simulation speed and fidelity

1. **Real-time factor** (`tools/rtf_probe.py`): 0.27 with a 4 ms physics step. The desktop and the Claude
   app use 2–3 of the 5 cores. I chose a **6 ms** step after comparing arc-odometry error at 4, 6 and 10 ms:
   2 %, 2 % and 10 % respectively.
2. **Wheel slip.** `tools/odom_vs_truth.py` and the joint probes showed that spins with direction reversal
   make the wheels slip in DART: the body turns only 50–90 % of the wheel-implied angle. I tried, without
   success:
   - higher friction;
   - the bullet-featherstone engine (it rejected the model);
   - a raised chassis collision and a ball-joint caster;
   - sphere wheel collisions;
   - explicit DiffDrive deceleration limits.
3. **Decision:** odometry is published by Gazebo's **OdometryPublisher** (model pose + 0.001 noise). DiffDrive
   still drives the wheels, and its slipping wheel odometry is kept on `/wheel_odom` as evidence. The `odom`
   frame therefore equals the world frame.
4. **Tooling bugs found and fixed along the way:**
   - `gz sim` outlived its launch, so two worlds once ran at the same time.
   - `pkill -f` patterns matched my own shell.
   - stale Fast DDS shared-memory files after a VM restart cut the bridge out of the ROS graph.

## Step 5: SLAM map (Member 2, Task 2)

1. **Configuration.** `config/slam_toolbox.yaml` was copied from the installed Jazzy
   `mapper_params_online_async.yaml`. Every change is marked `(project change)`. `launch/slam.launch.py`
   runs it.
2. **Mapping drive.** `maze_bringup/mapping_drive.py` with `config/mapping_route.yaml` (generated by
   `tools/make_mapping_route.py`) is a depth-first tour of all 36 cells: 55 waypoints, 0.18 m/s, a LiDAR stop
   below 0.13 m and a stale-scan stop in simulation time. It replaces keyboard teleoperation only.
3. **Early attempts distorted the centre and left of the maze.**
   - `tools/scan_lag_probe.py` matched every scan against the reference map: yaw offset 0.000 rad at rest and
     while spinning, so the sensor data and timestamps were fine.
   - The cause was SLAM loop closure and wide scan-matching searches in a repetitive maze.
4. **Offline tuning.** `tools/offline_slam.sh` replays a recorded bag with different parameter sets. The final
   set turns loop closing off and uses a 0.3 m correlation search, 0.03 smear, ±0.1 rad coarse angle and
   variance penalties 2/4. Walls within 0.10 m of the truth rose from 75.9 % to 98.8 %.
5. **Final live run**, saved with `map_saver_cli` as `maps/slam/maze_slam.{yaml,pgm}`:
   - 99.8 % of SLAM wall cells within 0.10 m of the true walls, median 0.019 m;
   - final SLAM correction 0.012 m / 0.011 rad.

   Evidence: `evidence_bags/slam_mapping_run` (rosbag), `docs/evidence/`, `docs/map_provenance.md`.

## Step 6: Localisation and Nav2 baseline (Members 2–3, Task 3a)

1. **Launch.** `launch/localization_navigation.launch.py` runs map_server + AMCL, then controller, planner,
   behaviour, BT navigator and velocity smoother, then `goal_adapter` and RViz. The velocity chain is
   controller → `/cmd_vel_nav` → smoother → `/cmd_vel` (Twist), checked against the installed
   `nav2_util/twist_publisher.hpp`, where stamped output defaults to false.
2. **Parameters.** `config/nav2.yaml` was derived from the installed Jazzy `nav2_params.yaml`. Problems were
   fixed one at a time, each from a logged failure:
   - BT `default_server_timeout` 20 ms → 2000 ms. The planner acknowledgement was timing out.
   - AMCL tightened (alphas 0.05, 120 beams, σ 0.1), because odometry is accurate.
   - NavFn (Dijkstra) could not extract a plan for any route with a turn, so the baseline is **Smac 2D
     (A\*)**. Its class and parameter names were verified in the installed plugin files.
   - Removed the static layer from the local costmap. It was being shifted by AMCL error and narrowing the
     corridors.
   - Lifecycle bonds disabled; heartbeats timed out under VM load.
   - Planning footprint is a circle of 0.13 m (circumscribed radius). Local collision checking uses the
     true polygon. Steep global inflation keeps paths in corridor centres.
   - RPP: lookahead 0.10 m, pivot when more than 0.25 rad off, collision check horizon 0.25 s.
   - Trials clear the costmaps after each teleport, because stale marks blocked the planner.
3. **Clicked goals.** RViz "2D Goal Pose" publishes on `/clicked_goal`. `goal_adapter` validates the goal,
   cancels any running one and dispatches it.

## Step 7: Deep Q-Network (Member 4, Task 3b)

1. **Package.** `ros2_ws/src/maze_dqn`:
   - `grid_map.py`: map loading with row inversion and origin; a 0.5 m lattice whose nodes are refined to the
     observed corridor centre, with clearance ≥ 0.154 m and every edge checked as a continuous clear segment.
   - `grid_env.py`: the goal-conditioned environment, 80-dimensional state, 4 actions; rewards +100 goal,
     −100 wall, −1 step, plus progress shaping.
   - `dqn.py`: MLP 256×256, replay buffer, target network, Double-DQN Bellman targets, Huber loss.
   - `train.py`, `evaluate.py`, and `planner.py` (greedy inference, loop/timeout/collision detection,
     validated connectors, **no classical fallback**).
   - `test/test_grid.py`: 5 tests, all passing.
2. **Smoke test.** Training on the reference map confirmed that the code learns: 72 % held-out success after
   3,000 episodes.
3. **Real training** on the **SLAM-map** lattice (36 nodes / 36 edges, identical to the reference topology):
   30,000 episodes, 162,280 steps, 63 minutes. Greedy evaluation: **1071/1071 training and 189/189 held-out
   start/goal pairs** are solved with the optimal number of moves; inference takes about 0.5 ms.
4. **Integration.** In `method:=dqn`, `goal_adapter` plans with the network, publishes `/dqn_plan` and sends
   the path to Nav2 FollowPath, using the same RPP controller as the baseline. The robot in Gazebo is driven
   by the learned route.

## Step 8: Comparison experiments (Task 5)

1. **Goals.** `config/evaluation_goals.yaml` defines G1 (short corridor), G2 (turns) and G3 (farthest cell),
   all starting from the spawn.
2. **Trial runner.** `maze_bringup/run_trials.py` repeats this for each trial:
   - stop the robot;
   - teleport it to the start (outside the timed run);
   - re-initialise AMCL;
   - clear the costmaps;
   - set the method;
   - publish the goal;
   - wait for the result.
3. **Logging.** `goal_adapter` writes `results/trials.csv` with the template columns plus ground-truth final
   error, localisation error, total heading change, angular-velocity RMS, minimum LiDAR range and the DQN
   cells used.
4. **Final run** (tag `final`), 18 trials: **18/18 successful, 0 collisions.** The DQN was faster in
   simulation time on every goal (G3: 72.9 s vs 80.9 s). The baseline drove shorter paths on the turning
   routes (G3: 6.96 m vs 7.82 m) but had more near-collision events (40 vs 9 on G3). Full table:
   `results/analysis/summary.md`.

   Figures: `results/analysis/comparison.png`, `dqn_training.png`, `routes_G1-3.png`.
5. **Failed earlier runs are kept:** `results/trials_attempt1_navfn.csv` (NavFn failures) and
   `results/tuning_trials.csv` (the tuning history).

## Step 9: Documentation

- `README.md`: install and run commands.
- `docs/environment_audit.md`, `docs/progress.md` (timestamped log), `docs/map_provenance.md`,
  `docs/architecture.md`.
- `docs/member1-4_handoff.md`, `docs/report_draft.md` (5–7 page report draft), and this file.

## What is NOT done or not verified here (honest status)

- **Live RViz clicking was not performed.** All runs were headless. Goals went through the same
  `/clicked_goal` topic the RViz tool uses, but you must rehearse three clicked goals with the GUI:
  `ros2 launch maze_bringup simulation.launch.py` then `ros2 launch maze_bringup localization_navigation.launch.py`.
- **No Gazebo overhead screenshot or video recording** was captured. Take these from the GUI.
- **Odometry is pose-derived** because of the DART wheel-slip issue. This is disclosed in the report; mention
  it at the VIVA.
- **The DQN is a hybrid** (learned global route + classical controller), not direct velocity control.
  Confirm with the lecturer that this satisfies Task 3b.
- **Still to fill in:** team names, IDs and contributions, the GitHub link (nothing has been pushed), and the
  cohort deadline.
- **Real-time factor is about 0.3 on this VM**, so wall-clock times are about 3× the simulation times.

## Quick demo checklist (for the VIVA)

1. `source ~/maze_testing/env.sh`
2. Terminal 1: `ros2 launch maze_bringup simulation.launch.py` (Gazebo GUI)
3. Terminal 2: `ros2 launch maze_bringup localization_navigation.launch.py` (RViz)
4. In RViz, confirm the scan overlays the map walls. If it doesn't, use **2D Pose Estimate**.
5. Click three **2D Goal Pose** destinations with the baseline.
6. `ros2 param set /goal_adapter method dqn`, then click again. The orange `/dqn_plan` route appears and the
   robot follows it.
7. Show `results/analysis/summary.md` and the figures.
