# Initial code audit — 17 October 2026

Baseline: Git commit 1952a2e; archive `/tmp/maze-original-20261017.tar.gz`.
The archive predates image.png; the baseline commit includes it.

## Structure and implementation

Two ament Python packages: maze_bringup (SDF world, primitive Burger model,
URDF, bridge, SLAM, AMCL, Nav2, RViz, clicked-goal adapter and CSV trials) and
maze_dqn (PyTorch Double DQN, replay, target network, training, checkpoint,
grid evaluation and continuous route connectors). Tools provide geometry/map
generation, mapping tours, probes and plots. Maps, checkpoints and CSVs exist.
No CMake or compiled project code is used.

## Dependency assessment

Observed: Ubuntu 24.04.5 x86_64, 7.8 GiB RAM, 61 GiB free disk. ros2, gz,
colcon, micromamba and /opt/ros absent; ROS_DISTRO unset. System Python 3.12
has neither pip nor working venv ensurepip. sudo requires the user's password.
DISPLAY and WAYLAND_DISPLAY are unset in this SSH session. Existing environment
report describes another Ubuntu 22.04 VirtualBox VM. Its live claims and imported
CSV results are historical and unverified here.

## Critical defects and compatibility problems

1. env.sh requires absent micromamba and hardcodes ~/maze_testing.
2. README, tools and result defaults hardcode the previous project path.
3. DQN evaluation uses an absolute checkpoint map path on another machine.
4. Manifests omit direct message dependencies and Pillow/PyTorch.
5. Some trial failures return without a result; simulation-only timeout hangs
   if Gazebo stops publishing clock.
6. Config uses Smac A*, while some comments and README say Dijkstra.
7. No .world deliverable or current 5–7 page report/verification package.
8. Tests cover maps/environment; neural updates/checkpoint tests missing.

## Missing assignment requirements

Original PDF unavailable. User supplied image.png, inspected directly: six by
six cells, 0.5 m pitch. Thick walls visually agree with the 18 existing segments,
including upper-right x=1.0, y=0.5..1.0. Thin grid guides are not walls.
Walls are 0.05 m thick: clear straight corridor width 0.45 m; spawn clearance
0.225 m. Live robot clearance remains untested.

Odometry derives from Gazebo model pose with noise; wheel odometry is separate.
This localization simplification must be disclosed. Imported SLAM map provenance
is claimed by old documentation, not freshly verified. No current GUI evidence.

## Initial build/tests and proposed corrective actions

ROS build/runtime BLOCKED: missing dependencies. Python tests initially BLOCKED:
missing numerical/test dependencies. Plan: preserve baseline; repair paths and
packaging; run geometry/DQN tests and saved policy evaluation; install native
Jazzy/Harmonic with user sudo; build and verify bridge/SLAM/Nav2; collect three-goal
results and real screenshots; generate report and compliance matrix. Retain the
existing architecture and avoid additional frameworks.
