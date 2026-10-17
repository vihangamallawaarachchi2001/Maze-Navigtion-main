# VIVA preparation

From the repository folder, run `source env.sh` in each terminal.

1. `ros2 launch maze_bringup simulation.launch.py render_engine:=ogre`
2. `ros2 launch maze_bringup localization_navigation.launch.py`
3. RViz: select **2D Goal Pose**, click and drag for goal orientation.
   Start (-1.25,-1.25,0). Goals: G1 (0.25,-1.25,0), G2 (-1.25,0.75,1.5708),
   G3 (1.25,0.25,0). Use 2D Pose Estimate only to correct localization after moving.
4. Switch with `ros2 param set /goal_adapter method dqn`; return with `method baseline`.
5. Show `results/current/trials.csv`, `results/current/analysis/comparison.png`,
   `evidence/logs/runtime.json` and `evidence/screenshots/`.

For repeatable comparisons, the trial runner resets simulation pose AND AMCL
between runs; see README. It publishes to the same topic as RViz. Explain which
runs were scripted and which were actual mouse interactions.

## Concepts and likely questions

**What are nodes/topics/actions?** Nodes perform tasks; topics carry sensor data;
Nav2 actions provide goal acceptance, feedback, cancellation and a result. A topic
name existing does not prove data arrived. Runtime evidence checks real messages.

**Which frames matter?** map -> odom comes from AMCL or SLAM, never both together.
odom -> base_footprint comes from Gazebo pose-derived odometry. URDF and
robot_state_publisher supply body/sensor transforms. All nodes use simulation time.

**How does SLAM work?** SLAM Toolbox aligns LiDAR scans using odometry and builds an
occupancy grid. Our scripted tour replaces teleoperation, not SLAM. Loading the
old map through map_server is localization, not mapping. Loop closing is disabled
in this inherited repetitive-maze configuration.

**Why is odometry unusually accurate?** It uses the simulated model pose plus noise,
not wheel integration. This was inherited to work around wheel slip. It is a
simulation simplification, not evidence of performance on a real Burger.

**How does Nav2 work?** AMCL estimates pose on the saved map. SmacPlanner2D performs
cost-aware A* on the global costmap. Regulated Pure Pursuit tracks the route;
local obstacle checking and velocity smoothing constrain motion. Inflation keeps
routes away from walls. The circle planning radius is 0.13 m.

**What makes this a genuine DQN?** An 80-input, 256/256-hidden-layer neural network
outputs four Q values. Epsilon-greedy exploration supplies replay transitions;
Adam minimizes Huber Bellman error. Double DQN chooses a next action with the online
network and evaluates it using a periodically copied target network. Checkpoints
store weights and the map lattice signature. Training occurs on a map-derived grid;
Nav2's controller executes the selected route in Gazebo.

**Reward and termination?** +100 on goal, -100 on blocked move, -1 per move plus
Euclidean progress shaping. Goal/collision terminate; time limits truncate and
bootstrap in training. The lattice has 36 nodes and four cardinal actions.

**A* versus DQN?** A* searches a graph from current costs; DQN uses learned Q values.
Both here rely on an existing map and share the physical controller. DQN has no
classical route fallback. Training a different map requires retraining/checkpoint
compatibility. The held-out test is other start/goal pairs in the same maze.

**What passed?** Eleven automated tests; both packages build; real scan/odom/TF/map
traffic; six Gazebo trials succeeded, three per method, zero chassis contacts.
Supplied DQN achieved 189/189 held-out grid successes. Use final verification for
fresh mapping and screenshot details. The short training smoke run tests the
pipeline and is not expected to converge.

**Which method is better?** In one run per scenario DQN was faster; A* travelled
less on the turning/far routes. This sample is too small for broad claims. Near
scan events occurred despite zero recorded chassis contacts.

**Why can reference/travelled ratio exceed one?** The reference reaches an exact
snapped goal, while the controller stops within tolerance; the reference also
uses a different clearance restriction. It is an approximate diagnostic, not a
bounded fair optimality score. State this limitation explicitly.

**Before submitting?** Add names/student IDs and GitHub URL, check the original PDF,
review the six-page PDF and repeat the live clicked-goal demonstration.
