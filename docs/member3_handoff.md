# Member 3 hand-off: Nav2 baseline, clicked goals, collision avoidance, logging

Owner: `<Member 3 name / ID>`

## Delivered
- `config/nav2.yaml`, derived from the installed Jazzy `nav2_params.yaml`. Every change is commented; rationale in docs/architecture.md.
- `launch/localization_navigation.launch.py`: map_server + AMCL + controller/planner/behavior servers + BT navigator
  + velocity smoother + goal_adapter + RViz. Exactly one `/cmd_vel` publisher during motion (velocity_smoother).
- `goal_adapter` (clicked goals, validation, cancellation, method switch, trial CSV), `run_trials` (repeatable protocol),
  `config/evaluation_goals.yaml`, `rviz/maze.rviz` (2D Goal Pose → `/clicked_goal`).
- Baseline: **Smac 2D (A\*)** global planner + Regulated Pure Pursuit. NavFn/Dijkstra was tried first and failed on
  every route with turns (`results/trials_attempt1_navfn.csv`).

## Commands
```bash
ros2 launch maze_bringup simulation.launch.py
ros2 launch maze_bringup localization_navigation.launch.py            # RViz opens; click "2D Goal Pose"
ros2 param set /goal_adapter method baseline|dqn
```
Goals in a wall, in unknown space, outside the map or < 0.13 m from a wall are rejected (logged). A new click cancels
the running goal (`preempted` in the CSV).

## Evaluation goals (map frame == world frame)
G1 (0.25, −1.25, 0), G2 (−1.25, 0.75, π/2), G3 (1.25, 0.25, 0); all from the spawn (−1.25, −1.25, 0).

## Baseline results (results/trials.csv, 3 repeats each, run tag `final`)
G1 3/3, G2 3/3, G3 3/3; 0 physical collisions; sim time 11.3 / 25.4 / 80.9 s; travelled 1.42 / 2.47 / 6.96 m;
near-collision events (LiDAR < 0.16 m) 0 / 11 / 40. Full table: results/analysis/summary.md.

## Problems solved (details in progress.md)
NavFn path extraction failure → Smac 2D · BT `default_server_timeout` 20 ms → 2000 ms · lifecycle bond timeouts under
load → bonds off · RPP false "collision ahead" (static layer in local costmap, long collision horizon, corner cutting)
→ live-only local costmap, 0.25 s horizon, 0.10 m lookahead · Smac hugging wall ends with the polygon footprint →
circumscribed circle for planning · stale costmap marks after teleport → costmaps cleared in `run_trials`.

## Remaining issues
- Live **RViz clicking was not exercised on this machine** (headless runs). Goals were published on the same
  `/clicked_goal` topic that the RViz tool uses. Rehearse three clicks before the VIVA.
- After any teleport, clear the costmaps (`ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap`).
