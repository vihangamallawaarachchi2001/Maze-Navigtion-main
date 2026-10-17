# Member 4 hand-off: DQN, Gazebo integration, comparison

Owner: `<Member 4 name / ID>`

## Delivered
- `maze_dqn` package: `grid_map.py` (map loading with row inversion/origin, lattice + node refinement + continuous-edge
  validation), `grid_env.py`, `dqn.py` (MLP, replay, target net, Double-DQN update), `train.py`, `evaluate.py`,
  `planner.py` (greedy inference, loop/timeout/collision detection, connector + segment validation, no fallback),
  `test/test_grid.py` (5 tests pass), `config/dqn.yaml`, `checkpoints/dqn_policy.pt`.
- Integration through `goal_adapter` (method `dqn`): learned route → `/dqn_plan` → Nav2 **FollowPath**, using the same
  controller as the baseline.
- `tools/analyze_results.py` → `results/analysis/{summary.md, trials_with_reference.csv, comparison.png,
  dqn_training.png, routes_G*.png}`.

## Training (results/dqn_training)
SLAM-map lattice: 36 nodes / 36 edges (signature a0e4a38e41ccd977). 30,000 episodes, 162,280 steps, 3,794 s wall
(CPU shared with the desktop). 1,260 ordered pairs: 1,071 train / 189 held out (seed 42).
Greedy evaluation (ε = 0): **1071/1071 train and 189/189 held-out reach the goal with BFS-optimal length**; mean
inference 0.5–0.7 ms per route. Generalisation is only within this fixed maze.

## Gazebo results (3 repeats each)
G1 3/3, G2 3/3, G3 3/3; 0 collisions; sim time 9.9 / 23.2 / 72.9 s; travelled 1.42 / 2.68 / 7.82 m; planning
0.46 / 0.49 / 1.38 s (dominated by Python clearance validation of the path, not by the network).

## Architecture statement
Hybrid: the DQN chooses the global cell route, and Nav2 RPP executes it. This is **not** end-to-end learned motor control.
If the lecturer requires direct DQN velocity control, see START_HERE.md "Alternate direct-control DQN" (not implemented).
