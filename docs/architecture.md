# System architecture

```
 Gazebo Harmonic (gz-sim 8, DART, 6 ms step)                           ROS 2 Jazzy
 ┌──────────────────────────────────────┐   ros_gz_bridge   ┌───────────────────────────────────────────────┐
 │ world assignment_maze (18 walls)      │ ───────────────▶ │ /clock /scan /odom /tf /joint_states /bumper   │
 │ turtlebot3_burger                     │                   │ /wheel_odom (evidence only) /imu              │
 │  - DiffDrive (actuation, cmd_vel)     │ ◀─────────────── │ /cmd_vel (geometry_msgs/Twist)                │
 │  - OdometryPublisher (odom->base_fp)  │                   └───────────────────────────────────────────────┘
 │  - gpu_lidar 360 beams 5 Hz           │
 │  - contact sensor on chassis          │      robot_state_publisher: base_footprint->base_link->base_scan ...
 └──────────────────────────────────────┘
      mapping mode:  slam_toolbox (map->odom)   +  mapping_drive (scripted tour)       -> maps/slam/maze_slam
      navigation:    map_server + AMCL (map->odom) + Nav2 (planner, controller, behaviours, BT, smoother)
                      ▲                         ▲
            NavigateToPose (baseline)    FollowPath (DQN path)
                      └────────── goal_adapter ◀── /clicked_goal (RViz "2D Goal Pose")
                                     │  method = baseline | dqn
                                     └─ maze_dqn.DQNPlanner (PyTorch Q-network, inference only)
```

## Packages

| Package | Contents |
|---|---|
| `maze_bringup` | world, robot SDF/URDF, bridge, launch files (simulation, slam, localization_navigation), Nav2/SLAM/AMCL params, RViz config, `goal_adapter`, `run_trials`, `mapping_drive`, `motion_test` |
| `maze_dqn` | occupancy-map loader and planning lattice, grid environment, DQN (network, replay, agent), training, evaluation, inference planner, tests |
| `tools/` | geometry generator, map quality, RTF/odometry/scan probes, offline SLAM replay, process control (`sim_ctl.sh`), analysis/plots |

## Conventional navigation (baseline)

- **Global planner (baseline): `nav2_smac_planner::SmacPlanner2D`**, a cost-aware **A\*** search
  on the 8-connected costmap with path smoothing (`cost_travel_multiplier` 3.0, `allow_unknown` false).
  Planner selection history:
  1. `NavfnPlanner` with `use_astar: false` (Dijkstra) was the first choice. In the first comparison
     run it failed on **every** route with a turn ("Failed to create a plan from potential when a
     legal potential was found", `results/trials_attempt1_navfn.csv`) and only planned the straight
     G1 corridor.
  2. Smac 2D was then selected. Its class name and parameter names were verified in the installed
     plugin XML and library. It is an identified A\* planner, which the brief allows.
- **Controller (shared by both methods):** `RegulatedPurePursuitController`. The stock Jazzy file
  uses MPPI (2000 samples × 56 steps at 20 Hz), which this VM could not run alongside Gazebo at
  RTF 0.3. RPP is light and follows a given path closely, which is also needed to execute DQN paths.
  Settings tuned for 0.45 m corridors with 90° turns:
  - desired 0.18 m/s, lookahead 0.10 m (0.08–0.15); pure-pursuit corner cut is ≈ L²/2R;
  - rotate-to-heading when > 0.25 rad off, slow down on curves with radius < 0.9 m;
  - collision detection up to the carrot limited to 0.25 s, about 4.5 cm. Longer horizons flagged
    corner geometry the path never crosses.
- **Footprints (the robot is never shrunk):**
  - Planning (global costmap): circle r = 0.13 m, the circumscribed radius of the true outline
    (0.124 m) + 6 mm. With the true polygon (inscribed radius 0.04 m) Smac cut wall ends to ~5 cm,
    which a pivoting Burger cannot follow.
  - Local collision checking: the true polygon, x ∈ [−0.105, 0.040], y ∈ [−0.090, 0.090]
    (base box −0.102…0.038, wheels |y| ≤ 0.089, ~3 mm margin).
  - Global inflation 0.35 m with cost scaling 6.0 keeps paths in corridor centres. The local costmap
    uses the stock-style live-LiDAR obstacle layer + inflation 0.30 m / scaling 3.0, without the
    static map, which would be shifted by AMCL error.
- **AMCL:** odometry is accurate (pose-derived), so alpha1–4 are 0.05, with 120 beams, likelihood
  field σ = 0.1, z_hit 0.8, z_rand 0.2 and update every 0.05 m.
- **Lifecycle:** bonds disabled (`bond_timeout: 0`). Heartbeats timed out under VM load and the
  manager deactivated Nav2 mid-goal.
- **Velocity chain:** `controller_server`/`behavior_server` → `/cmd_vel_nav` → `velocity_smoother`
  (0.22 m/s, 2.0 rad/s, 1.0 m/s², 3.0 rad/s²) → `/cmd_vel` (Twist) → bridge. `goal_adapter` only
  publishes zero Twists when cancelling or shutting down.
- **Recoveries:** stock Jazzy BT (`navigate_to_pose_w_replanning_and_recovery.xml`): clear
  costmaps, spin, wait, back up. `default_server_timeout` was raised from 20 ms to 2000 ms because
  the planner acknowledgement took longer than 20 ms on the loaded VM.

## DQN method (hybrid learned global planner)

The learned part is the **global route**. Low-level execution uses the **same Nav2 controller**
as the baseline. This is not end-to-end learned motor control.

1. `Lattice` is built from the **SLAM map**. Nodes are 0.5 m apart, refined by up to 8 cm toward the
   observed corridor centre. A node is valid if its clearance is ≥ 0.154 m (circumscribed radius
   0.124 m + 0.03 m). An edge is valid only if every 2 cm sample of the straight segment is known-free
   with the same clearance. Unknown cells are blocked. The result has 36 nodes and 36 edges,
   identical to the reference-geometry lattice.
2. State: one-hot agent node (36) + one-hot goal node (36) + 4 local move-validity flags +
   normalised agent/goal indices (4) = 80 inputs. Actions: 4 cardinal moves.
3. Q-network: MLP 80 → 256 → 256 → 4 (ReLU), Adam, Huber loss, replay buffer 100k, batch 128,
   target network synced every 1000 steps, Double-DQN targets, ε-greedy 1.0 → 0.05 over 60 % of
   episodes, γ = 0.97, seed 42 (`config/dqn.yaml`).
4. Reward: +100 goal, −100 invalid move (terminal), −1 per step, plus Euclidean progress shaping
   (1 × reduction in cell distance). Timeout 60 steps; timeouts bootstrap, they are not terminal.
5. Split: 15 % of the 1260 ordered start/goal pairs are held out and never used for training.
   Generalisation is within this fixed maze only.
6. Inference (`DQNPlanner`): greedy ε = 0, no learning. Failure on blocked move, revisited node
   (loop) or timeout raises `PlanningError`. There is **no** classical fallback. The actual robot
   pose and the clicked goal are attached to their nearest node through connectors checked for
   clearance (0.13 m). Every segment is re-checked. The path is densified to 5 cm with headings
   along each segment, and the final heading is the clicked heading.
7. Execution: `goal_adapter` publishes the path on `/dqn_plan` and sends it to Nav2 **FollowPath**
   (`controller_id FollowPath`, `goal_checker_id general_goal_checker`): same controller, limits,
   footprint, costmaps and goal checker as the baseline.

The BFS shortest path on the lattice is computed **only** as an evaluation reference
(`bfs_length`). It is never used to plan.

## Goal interface

RViz "2D Goal Pose" publishes on `/clicked_goal` (set in `rviz/maze.rviz`). Nav2's own
`goal_pose` topic is not used, so exactly one component turns a click into motion. `goal_adapter`:

- rejects goals not in the `map` frame, outside the map, in occupied/unknown space, or with less
  than 0.13 m clearance;
- cancels the running goal when a new click arrives (logged as `preempted`);
- in baseline mode, times `ComputePathToPose` (planning time and planned length), then sends
  `NavigateToPose`;
- in DQN mode, runs inference (planning time = inference + validation), then `FollowPath`;
- switch modes at runtime: `ros2 param set /goal_adapter method dqn`.

## Trial logging (`results/*.csv`)

These are the `results_template.csv` columns plus extras: `total_rotation_rad` (Σ|Δyaw| from /odom),
`angular_velocity_rms` (smoothness), `min_scan_range_m`, `dqn_cells`, `true_final_x/y`,
`true_final_error_m` (Gazebo ground truth) and `localization_error_m` (AMCL vs truth).

- **Collision:** a rising edge of chassis contact with a `maze` wall from the Gazebo contact sensor
  (`/bumper`), debounced 0.5 s. This is a physical collision.
- **Near collision:** a rising edge of minimum LiDAR range < 0.16 m (counted separately, not a collision).
- **Success:** Nav2 action succeeded **and** true final position error ≤ 0.15 m **and** 0 collisions.
- **Travelled length:** Σ of planar /odom displacements during the trial.
- **Time:** simulation elapsed (from /clock) and wall elapsed. RTF = sim/wall.
