# Final verification — IE4060 Assignment 02

Audit date: 17 October 2026. Read this document in preference to inherited reports.

## Executive summary and readiness

The project genuinely runs on this Ubuntu 24.04 VM. Native ROS 2 Jazzy and Gazebo
Harmonic execute the maze and primitive Burger model with real LiDAR/odometry/TF.
Both A* and the supplied DQN checkpoint reached all three comparison goals:
6/6 successes, zero recorded chassis contacts. Fresh SLAM completed all 55
waypoints in 346 wall seconds and saved a new map. Actual RViz mouse-goal status:
**PASS**, 4 completed mouse trials (including an initial G1 check).
The technical report is six pages. Submission still needs student identities,
GitHub URL/upload and checking against the original assignment PDF.

## Original condition and modifications

Baseline Git commit: `1952a2e`. The folder originally had no Git history. A baseline
archive is also at /tmp/maze-original-20261017.tar.gz (before image.png arrived).
The imported implementation already contained a genuine Double DQN, checkpoint,
Nav2, SLAM, worlds, maps and historical results. Old docs describe another machine;
they were retained, not accepted as proof. See docs/01_initial_code_audit.md.

Changes: native Jazzy setup and portable env.sh; relative project paths in tools,
portable result defaults; explicit map override for relocated DQN checkpoints;
direct ROS/Pillow dependency declarations; replay batch warmup guard; corrected
checkpoint YAML-hash field name; logged missing-checkpoint/action-server failures;
wall-time navigation bound; corrected A* labels; alternate renderer launch option;
.world link to SDF; mapping-only RViz view; geometry/network/checkpoint tests;
real runtime probe and XComposite screenshot utility; README, report, VIVA and
verification deliverables. Existing architecture and old data remain intact.

## Installed dependencies and versions

User ran scripts/setup.sh with their own sudo password. Initial python3-torch apt
request had no candidate; corrected to CPU torch in .venv. Installed stack:
Ubuntu 24.04.5; Python 3.12; Harmonic 8.15.0; Nav2 1.3.13; SLAM Toolbox 2.8.5;
ros_gz 1.0.24; PyTorch 2.14.1+cpu. Exact package versions:
evidence/logs/versions.txt, installed-ros-packages.txt and python-versions.txt.
Python numerical, YAML, Pillow, pytest, matplotlib, ReportLab and colcon installed.
No major OS upgrade or unrelated package removal was performed.

## Build and automated test results

| Check / command | Expected | Observed | Status / evidence |
|---|---|---|---|
| bash scripts/build.sh | Both packages build | 2 packages built | PASS, evidence/logs/build.txt |
| colcon build with separate /tmp build/install/log bases | Clean build | 2 packages, 21.8 s | PASS, evidence/logs/clean-build.txt |
| bash scripts/run_tests.sh | Geometry, maps, network, checkpoint valid | 11 passed | PASS, evidence/logs/tests.txt and tests.xml |
| python3 -m pytest -q tests ros2_ws/src/maze_dqn/test | Final source tests | 11 passed | PASS, evidence/logs/final-tests.txt |
| colcon test --python-testing pytest | Execute package tests | 8 tests, no failures/errors/skips | PASS, evidence/logs/colcon-test-results.txt |
| Initial colcon test without selector | Discover tests | 0 discovered | NOT TESTED by that command; explicit pytest run above fixes discovery |
| python3 tools/check_runtime.py | Typed messages and pub/sub delivery | All six checks received samples | PASS, evidence/logs/runtime.json |
| bash -n env.sh scripts/*.sh tools/*.sh | Shell syntax | Exit 0 | PASS |
| pdfinfo deliverables/final_report.pdf | 5–7 pages | 6 pages | PASS |

Package tests are a subset of the 11 root tests, not 19 independent tests.

## Gazebo execution and graphics

PASS with render_engine:=ogre. Maze, floor and robot collision geometry execute;
entity creation succeeded and 360-beam base_scan messages, odom pose, clock,
TF and TF-static were received. Bridge data, action names and lifecycle status
are preserved in evidence/logs. Navigation became active.

The default OGRE2 renderer FAILED with Mesa EGL segmentation fault in both initial
headless and desktop attempts. Logs: simulation.txt, simulation-gui.txt. The OGRE
fallback ran successfully on the desktop with llvmpipe; simulation-ogre.txt and
graphics.txt record this. GUI is functional with software rendering, not accelerated.
Headless fallback without a display was not revalidated after renderer recovery.

## SLAM verification and map provenance

PASS: slam_toolbox alone published /map (slam-map-publisher.txt), after AMCL/Nav2
were stopped. The live tour completed 55/55 waypoints (mapping-drive.txt).
New map: maps/current/maze.yaml and maze.pgm, 121x122 at 0.025 m. Map save first
FAILED with a two-second subscription window; transient-local map subscription
and save_map_timeout:=10.0 succeeded (map-save.txt; failed attempt retained).
A real mapping-only RViz capture shows this new occupancy grid.

Fresh map quality versus geometry (fresh-map-quality.txt): median wall distance
0.012 m, p95 0.056 m, 90.3% within 0.05 m, 100% within 0.10 m; reference free-space
coverage 78.9%, reference centre-line wall cells falsely free 0%. These metrics
are geometric diagnostics and do not prove every free cell was observed.

Comparison navigation uses the supplied maps/slam map, loaded and exercised in
this audit; its original generation cannot be independently recovered. The fresh
map was not substituted into the checkpoint or the six comparison runs. The
geometry reference is not SLAM. Odometry is model-pose-derived plus noise, not
wheel-integrated; this materially simplifies localization. Loop closing is off.

## Three-goal navigation and metrics

Common start (-1.25,-1.25,0), map/world frames coincide under the inherited setup.
One trial per method/scenario. The runner resets pose, AMCL and costmaps outside
measurement. These six comparison goals use the same topic as RViz, not mouse
input. Raw data: results/current/trials.csv; driver: three-goal-trials.txt.

| Method | Goal x,y | Sim s | Wall s | Planned m | Actual m | True error m | Contacts | Outcome |
|---|---|---|---|---|---|---|---|---|
| baseline | 0.250,-1.250 | 11.09 | 13.54 | 1.503 | 1.420 | 0.084 | 0 | succeeded |
| dqn | 0.250,-1.250 | 10.16 | 11.75 | 1.497 | 1.419 | 0.084 | 0 | succeeded |
| baseline | -1.250,0.750 | 26.44 | 32.75 | 2.589 | 2.446 | 0.110 | 0 | succeeded |
| dqn | -1.250,0.750 | 22.57 | 26.64 | 2.888 | 2.680 | 0.092 | 0 | succeeded |
| baseline | 1.250,0.250 | 84.34 | 97.15 | 7.097 | 6.949 | 0.129 | 0 | succeeded |
| dqn | 1.250,0.250 | 72.56 | 82.45 | 8.326 | 7.778 | 0.144 | 0 | succeeded |


Success means action succeeded, true final position <=0.15 m from requested goal,
and no chassis-wall contact recorded. All six satisfy it. Near-scan events were
A*: 0,1,16; DQN: 0,4,4. Chassis-only contacts do not certify all robot parts.

Actual mouse trials are separate: XTest did not deliver goals under Wayland;
direct X11 mouse events through RViz's SetGoal tool did. This uses the GUI tool,
not direct goal-topic injection. Screen-pixel selection causes small differences
from nominal goal coordinates. Actual requested coordinates are below.
Logs: rviz-mouse-goals.txt (unsuccessful XTest), rviz-mouse-goals-retry.txt,
navigation-clicked.txt and results/current/clicked_trials.csv.

| Run | Actual clicked goal x,y | Sim s | Actual m | True error m | Success |
|---|---|---|---|---|---|
| 20261017-232924-baseline-001 | 0.281,-1.234 | 11.60 | 1.423 | 0.113 | 1 |
| 20261017-232958-baseline-002 | 0.281,-1.234 | 11.30 | 1.411 | 0.126 | 1 |
| 20261017-233023-baseline-003 | -1.219,0.766 | 25.12 | 2.507 | 0.101 | 1 |
| 20261017-233102-baseline-004 | 1.281,0.266 | 81.29 | 6.898 | 0.137 | 1 |


Metrics: L=sum Euclidean odometry increments; time=simulation/wall elapsed;
RTF=simulation/wall time; final error=Gazebo pose-to-goal distance; smoothness=
sum absolute wrapped yaw increments and angular velocity RMS. Reference path is
8-connected Dijkstra at 0.154 m clearance with snapped endpoints. Reference/L can
exceed one due to tolerance, clearance and endpoint mismatch; it is an approximate
diagnostic, not a bounded fair optimality score. DQN's BFS lattice metric is separate.

DQN was faster in all three comparisons; A* travelled less on G2/G3. One sample
per case cannot establish statistical superiority or repeatability. New plots
in results/current/analysis are derived from these six rows. Historical CSVs
were never appended to or treated as newly executed observations.

## DQN training and evaluation

Genuine MLP Q-network, replay, Double DQN target, epsilon exploration, Huber loss,
Adam, persistence and map-signature checking are implemented. No A* fallback.
The supplied checkpoint was reevaluated: train 1071/1071, held-out 189/189,
all routes BFS shortest on the same 36-node lattice. Results: dqn_eval.
Gazebo separately executed three DQN routes with the shared Nav2 controller.

Fresh 300-episode training smoke: 4371 steps, 181 seconds, checkpoint saved.
Held-out greedy result 21/189 (11.11%); 168 loops. Train 117/1071 (10.92%).
This short policy is NOT converged and is NOT used in navigation. Full 30,000-
episode training was not rerun. The inherited full training log remains labelled
historical. Plots distinguish inherited and fresh training. A premature smoke
checkpoint evaluation failed before training finished; log retained, then rerun.

## Screenshot inventory

Every image listed below is a real application-window pixel capture via
XComposite, visually checked where noted. Qt screen grabs returned black and
were replaced/deleted; GNOME's desktop screenshot API denied access. No generated
illustrations were used. Some final screenshots show map/robot state rather
than action-result text; CSV/action logs supply the matching result evidence.
The older navigation RViz window still displays its inherited Dijkstra legend;
the actual tested planner is Smac A* and source RViz configuration is corrected.
Orange paths can persist from the preceding DQN trial; the blue path belongs to
the current baseline goal. These real captures were not edited to change labels.

- evidence/screenshots/clicked_G1_navigating.png
- evidence/screenshots/clicked_G1_result.png
- evidence/screenshots/clicked_G2_navigating.png
- evidence/screenshots/clicked_G2_result.png
- evidence/screenshots/clicked_G3_navigating.png
- evidence/screenshots/clicked_G3_result.png
- evidence/screenshots/gazebo_maze_desktop.png
- evidence/screenshots/rviz_fresh_slam_map.png
- evidence/screenshots/rviz_navigation_G3.png

DQN learning and performance charts are data plots, not application screenshots:
results/current/analysis/learning_curves.png and comparison.png.

## Outstanding problems and requirement compliance

| Requirement | Status | Evidence / limitation |
|---|---|---|
| Maze and .world deliverable | PASS | SDF/world, collision tests, real Gazebo; image.png visually compared |
| Full original PDF compliance | BLOCKED | Original PDF unavailable; detailed user brief and figure used |
| ROS packages compile | PASS | Normal and independent clean build |
| Robot spawns and sensors bridge | PASS | Entity success, scan/odom/TF/clock samples |
| GUI | PASS with fallback | OGRE desktop/llvmpipe; OGRE2 failed |
| LiDAR SLAM and map save | PASS | 55-waypoint tour, fresh map, runtime/publisher logs, real RViz capture |
| Static-map localization | PASS | AMCL and all navigation trials |
| A* navigation, three goals | PASS | 3/3 comparison routes |
| DQN navigation, three goals | PASS | 3/3 comparison routes with supplied checkpoint |
| Actual clicked-goal demo | PASS | See mouse CSV and window captures |
| Avoid collisions | PARTIAL | Zero chassis contacts in six trials; other parts not independently monitored |
| DQN learning implementation | PASS | Learning/target tests, new smoke checkpoint |
| Fresh full-training convergence | NOT TESTED | Only 300 episodes rerun; smoke policy is weak |
| Held-out saved-policy evaluation | PASS | 189/189 pairs in same maze, not unseen mazes |
| Efficiency experiments | PARTIAL | Real paired results/plots; one sample per case, approximate reference ratio |
| Report | PASS | Markdown and six-page PDF |
| VIVA material | PASS | viva_preparation.md; learner must rehearse |
| GitHub-ready source | PARTIAL | Baseline/history retained; add identities, remote URL and upload |

## Exact reproduction commands

From the project root, use separate terminals and source env.sh in every one:

```bash
bash scripts/setup.sh
bash scripts/build.sh
source env.sh
ros2 launch maze_bringup simulation.launch.py render_engine:=ogre
```

```bash
source env.sh
ros2 launch maze_bringup localization_navigation.launch.py
# RViz: 2D Goal Pose; click/drag the three goal regions from README.
ros2 param set /goal_adapter method dqn
```

```bash
source env.sh
ros2 run maze_bringup run_trials --ros-args -p use_sim_time:=true   -p goals_file:="$MAZE_PROJECT_ROOT/ros2_ws/src/maze_bringup/config/evaluation_goals.yaml" -p repeats:=3
bash scripts/run_tests.sh
```

For mapping, stop navigation first; restart simulation at its spawn pose:

```bash
source env.sh
ros2 launch maze_bringup slam.launch.py
ros2 run maze_bringup mapping_drive --ros-args -p use_sim_time:=true -p max_speed:=0.18
ros2 run nav2_map_server map_saver_cli -f "$MAZE_PROJECT_ROOT/maps/current/maze" --ros-args   -p use_sim_time:=true -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
```

Training/evaluation, map override, analysis, screenshots and troubleshooting are
in README. Preserve each new experiment separately. No GitHub publication has
been performed. Submission readiness is technically functional with the above
limitations; personal submission details and the full brief remain unchecked.
