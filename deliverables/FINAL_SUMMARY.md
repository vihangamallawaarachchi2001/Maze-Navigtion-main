# Beginner-friendly final summary

The project genuinely runs on this VM. It simulates a Burger-sized robot in the
maze from image.png. LiDAR measures walls; SLAM builds a map; AMCL localizes the
robot on a saved map; Nav2 controls its movement to a clicked goal.

Two route methods work: Smac A* searches the map, while the supplied trained DQN
chooses map-grid moves using a neural network. Both use the same Nav2 controller
to move the Gazebo robot. DQN is not directly controlling wheel velocities.

## What was verified

- Both packages build, including an independent clean build.
- Eleven automated tests pass; colcon executes the eight package-level tests.
- Real LiDAR, odometry, transforms, clock and occupancy messages arrive.
- Fresh SLAM completed all 55 mapping waypoints and saved maps/current/maze.yaml
  and maze.pgm. An actual RViz capture shows the map.
- A* and the supplied DQN each reached all three comparison goals: six successes,
  no recorded chassis contacts. Times and distances are in results/current/trials.csv.
- Actual RViz goal-tool mouse events selected three destinations and all passed.
  These extra runs are in results/current/clicked_trials.csv.
- The supplied DQN achieved 189/189 held-out grid routes. A new short training run
  saved a checkpoint but achieved only 21/189; use the supplied trained checkpoint
  for navigation. Full training convergence was not rerun.

## Run it

Open separate VM desktop terminals in this project folder. In each, `source env.sh`.

Terminal 1: `ros2 launch maze_bringup simulation.launch.py render_engine:=ogre`

Terminal 2: `ros2 launch maze_bringup localization_navigation.launch.py`

RViz: choose **2D Goal Pose**, click and drag to select position and orientation.
Use `ros2 param set /goal_adapter method dqn` to switch, or `method baseline` to
return to A*. Ctrl+C stops each launch. The tested OGRE renderer is necessary on
this VM because default OGRE2 crashed. Run GUI commands on the VM desktop.

For the VIVA, show the maze and robot, real scans, the fresh mapping screenshot,
and navigation to G1 (0.25,-1.25), G2 (-1.25,0.75), G3 (1.25,0.25). Explain the
frames, difference between mapping/localization, and difference between A* and
DQN. Read viva_preparation.md before the demonstration.

## Files to use

- Six-page report: deliverables/final_report.pdf; editable source: final_report.md.
- Full evidence and limitations: deliverables/FINAL_VERIFICATION.md.
- Actual screenshots: evidence/screenshots/; measurements: results/current/.
- Fresh SLAM map: maps/current/; supplied map used in trials: maps/slam/.
- Full commands and troubleshooting: README.md.
- Original project: Git commit 1952a2e. Old results remain preserved separately.

## Before submission

Add student names/IDs and your GitHub repository URL, regenerate the PDF with
`source env.sh && python3 tools/build_report.py`, and publish your repository.
Check the original assignment PDF: only its detailed pasted requirements and
maze image were available. Rehearse the live three-click VIVA demonstration.

State the limitations honestly: odometry uses Gazebo model pose, only chassis
contacts are recorded, mapping loop closure is disabled, and the paired comparison
has one trial per method/goal. The DQN far trial finished close to the position
limit; more repeats are worthwhile. The fresh map was saved separately and was
not used to retrain the supplied checkpoint or replace its navigation map.
