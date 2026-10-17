# Progress log

## 2026-10-16 Stage 0 audit (pre-resize)
- Host: Ubuntu 22.04.5 LTS, KVM VM, 1 vCPU, 1.9 GiB RAM, 8.6 GB free disk.
- Not present: /opt/ros, ~/maze_testing (created now, empty), ~/turtlebot3_ws, pip, numpy, torch. sudo needs password.
- PROJECT_STATE.md paths are historical and do NOT exist on this machine; project is built fresh from MAZE_REFERENCE.md.
- Internet OK (pypi, packages.ros.org).
- Installed: micromamba 2.9.0 at ~/.local/bin/micromamba.
- Decision: no-sudo install of ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM Toolbox via RoboStack (conda-forge), user-space.
- User is shutting down the VM to raise CPU/RAM.

## Next step
After reboot: verify `nproc`, `free -h`, `df -h ~`, then create micromamba env `ros_env` from robostack-jazzy channel.

## 2026-10-16 after VM resize (5 vCPU, 8 GiB RAM)
Done (code only; nothing ROS-related tested yet):
- Maze geometry verified against Figure 1 by pixel measurement of the brief (pdftoppm, 100 dpi):
  all horizontal walls match MAZE_REFERENCE.md; one vertical wall (x=1.0, y 0.5..1.0) is in Figure 1
  but missing from MAZE_REFERENCE.md -> added as `upper_right_v`. 18 segments; all 36 cells connected.
- tools/maze_geometry.py -> worlds/assignment_maze.sdf + maps/reference/ground_truth_maze.* (geometry, NOT SLAM).
- Burger model SDF + URDF, bridge.yaml, safe_spawn.yaml (-1.25,-1.25,0; clearance 0.225 m > 0.124 m radius),
  simulation.launch.py, maze_dqn package (grid_map, grid_env, dqn, train, evaluate, planner, tests),
  goal_adapter.py, run_trials.py.
BLOCKER: disk. RoboStack env (ros-jazzy + gz + nav2 + slam + pytorch) needs ~10-11 GB; only 8.5 GB was free.
Install stopped at 99% disk. Extracted package cache kept in ~/micromamba/pkgs (6.8 GB) for reuse; already-extracted
archives deleted (1.5 GB freed). git is not installed (will come from conda env).

## Next step
User enlarges the VirtualBox disk (+30 GB) and grows /dev/sda3. Then re-run the same
`micromamba create -n ros_env ...` command (log: logs/env_install.log), dropping `compilers cmake make` (only
ament_python packages are built) and adding `git`.

## 2026-10-16 Member 1 verified
- Disk grown to 50 GB by user. RoboStack env `ros_env` installed: ROS 2 Jazzy, gz-sim 8.10.0 (Harmonic), Nav2, slam_toolbox, ros_gz, torch 2.14.1 (CPU), git.
- `source ~/maze_testing/env.sh` activates env + overlay. colcon `--symlink-install` fails with this setuptools (`--editable` not recognized) -> use plain `colcon build`.
- `tools/sim_ctl.sh start sim maze_bringup simulation.launch.py headless:=true` -> spawn OK at (-1.25,-1.25,0).
- Topics live: /clock, /scan (frame base_scan), /odom (odom->base_footprint), /tf, /joint_states; /cmd_vel Twist subscribed only by maze_bridge.
- RTF 0.27 with 4 ms physics step -> 0.61 with 10 ms step (tools/rtf_probe.py).
- motion_test PASS: 0.15 m/s x 3 s -> odom 0.4365 m, Gazebo pose x -1.25 -> -0.8128 (0.437 m), final speed 0.
- TF odom->base_scan = odom pose + (-0.032, 0, 0.182): single chain, no duplicate authorities.

## 2026-10-16 Member 2 - SLAM attempts and simulation fidelity investigation
SLAM attempts 1-4 with the scripted mapping drive all distorted the centre of the maze. Evidence and fixes, in order:
1. RTF: 4 ms physics step RTF ~0.21-0.27; 10 ms ~0.41-0.61 (desktop + Claude app use ~3 of 5 cores).
2. Tooling bugs found and fixed: `sim_ctl.sh stop` left `gz sim` servers alive (two worlds ran at once
   and shared gz topics) -> now kills our world's gz server explicitly, only for the `sim` launch.
   `pkill -f <pattern>` matched our own shell; we now stop processes by PID.
3. Wheel odometry vs Gazebo ground truth (tools/odom_vs_truth.py, scratch/phase_test.py, joint_probe.py):
   straight driving exact; arcs at 10 ms step +10 % yaw; spins with direction reversal slipped 0.5-1.1 rad
   even at 2 ms. Tried: wheel friction mu 1e5 (worse), bullet-featherstone (rejects model: "Multiple
   sub-trees"), raised chassis collision + ball-joint caster (body now returns to start heading, wheels
   still slip), sphere wheel collisions, explicit DiffDrive deceleration limits. Wheel joints and DiffDrive
   odometry agree exactly, so the slip is between wheel and ground in DART on this VM.
   Decision: DiffDrive still actuates; /odom + odom->base_footprint TF come from gz OdometryPublisher
   (model pose + 0.001 Gaussian noise); wheel odometry is still bridged as /wheel_odom for evidence.
   Consequence: odom frame == Gazebo world frame; SLAM map->odom starts at identity, so map == world.
4. Attempt 4 (pose-derived odom): map of right/centre maze clean (scratch/slam_try4_partial.png) but
   SLAM still applied a -0.6 rad map->odom correction in the left corridor while odom == truth.
   => investigating LiDAR scan timestamp consistency (tools/scan_lag_probe.py).

## 2026-10-16 Member 2 complete
- Loop closure caused the rotations (scans verified consistent: tools/scan_lag_probe.py, yaw offset 0.000 rad at
  0 / +-0.5 / 1.0 rad/s). Offline bag replays (tools/offline_slam.sh): A original params 75.9 % walls within 0.10 m,
  C tuned 98.8 %. Live run with C: maps/slam/maze_slam.* 99.8 % within 0.10 m, median 0.019 m; final SLAM
  correction 0.012 m / 0.011 rad. Lattice topology identical to reference. Details: docs/map_provenance.md.

## 2026-10-16 Member 4 training
- 30,000 episodes on the SLAM lattice (36 nodes, node refinement 8 cm), 162,280 steps, 3,794 s wall
  (partly shared CPU). Greedy eval: 1071/1071 training pairs and 189/189 held-out pairs reach the goal with
  BFS-optimal length; mean inference 0.5-0.7 ms. results/dqn_training/.
- Gazebo DQN smoke test (G1) via FollowPath: success, true final error 0.071 m, 0 collisions.

## 2026-10-16 Member 3 Nav2 issues found during the first comparison run
- bt_navigator default_server_timeout 20 ms -> planner acknowledgement timed out on the loaded VM -> 2000 ms.
- AMCL final pose 4-6 cm from ground truth -> alphas 0.05, 90 beams; success now judged on Gazebo ground truth.
- Comparison attempt 1 (results/trials_attempt1_navfn.csv, 12 trials before stopping): NavFn (Dijkstra) failed to
  plan every G2/G3 route ("Failed to create a plan from potential when a legal potential was found"), G1 OK.
  DQN G3 aborted twice: RPP "detected collision ahead" when its lookahead arc cut 90-degree corners -> patience exceeded.
- Changes: baseline planner -> nav2_smac_planner::SmacPlanner2D (cost-aware A*), RPP lookahead 0.25 (0.15-0.30),
  rotate_to_heading_min_angle 0.35. Both methods use the same controller settings.

## 2026-10-16 Final comparison and documentation
- After the VM restart: stale Fast DDS SHM files in /dev/shm broke the bridge's ROS participant -> removed (owned files only,
  with no ROS processes running).
- Final configuration (committed): Smac2D, circle r=0.13 planning / true polygon local, global inflation 0.35/6.0,
  RPP lookahead 0.10, collision horizon 0.25 s, AMCL sigma_hit 0.1 / z_hit 0.8 / 120 beams, bonds off.
- Final run `final`: 18/18 successes, 0 collisions (results/trials.csv, results/analysis/summary.md).
- Docs: member1-4 hand-offs, report_draft.md, IMPLEMENTATION_REPORT.md.

## Next step
Run the GUI demo (README "Daily use"): three clicked baseline goals + DQN goals in RViz, take Gazebo/RViz
screenshots for the report, fill in names/IDs, push to GitHub when the team decides.
