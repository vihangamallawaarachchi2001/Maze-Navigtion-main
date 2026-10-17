# Map provenance

Two kinds of map exist in this project. They are stored separately and must never be confused.

| File | Kind | Made by | Used for |
|---|---|---|---|
| `maps/reference/ground_truth_maze.{yaml,pgm}` | **Geometry-derived reference. NOT SLAM** | `tools/maze_geometry.py` rasterises the world's wall boxes | unit tests, map-quality scoring, DQN smoke test |
| `maps/slam/maze_slam.{yaml,pgm}` | **LiDAR SLAM result** | SLAM Toolbox (online async), live Gazebo run on 16 Oct 2026 | AMCL localisation, Nav2 costmaps, DQN training and planning, all trials |

## World revision

`maze_v1_18seg_t0.05_h0.30`: 18 wall centre-line segments, walls 0.05 m thick and 0.30 m high,
DART physics with a 6 ms step. The segments come from MAZE_REFERENCE.md, checked against Figure 1
of the brief by pixel measurement (`pdftoppm` at 100 dpi; solid walls fill 1.00 of a grid edge,
dotted guides 0.75). One wall present in Figure 1 was missing from MAZE_REFERENCE.md and was added:
`upper_right_v` (1.0, 0.5) → (1.0, 1.0). All 36 cells are reachable.

## Coordinate frames

The maze centre is the world origin, with +x right and +y up as in Figure 1. The robot spawns at
(−1.25, −1.25, yaw 0) (`config/safe_spawn.yaml`).

Odometry is published by Gazebo's OdometryPublisher system. It reports the model pose in the
world frame plus 0.001 Gaussian noise, so the `odom` frame **coincides with the Gazebo world frame**.
SLAM Toolbox starts with `map → odom` = identity, so the SLAM `map` frame also coincides with the
world frame. At the end of the mapping run its correction was only (0.012 m, −0.006 m, −0.011 rad).
Evaluation goals in `config/evaluation_goals.yaml` are therefore world coordinates.

Why odometry is pose-derived: DART wheel odometry on this VM slipped badly in rotation. Wheel
joints and DiffDrive odometry agreed exactly, but the body turned 50–90 % of the wheel-implied
angle (see progress.md for the test sequence). DiffDrive still drives the wheels, and its wheel
odometry is still bridged as `/wheel_odom` (and recorded in the mapping bag) as evidence.
This is a simulation simplification and is disclosed in the report.

## SLAM run (the map we use)

- Mapping method: scripted waypoint tour (`mapping_drive`, `config/mapping_route.yaml`), a
  depth-first visit of all 36 cells (55 waypoints), at a maximum of 0.18 m/s, with LiDAR safety
  stop. It replaces keyboard teleoperation only. It steers by odometry; the map comes solely from
  `/scan` + TF through SLAM Toolbox.
- SLAM parameters: `config/slam_toolbox.yaml`, derived from the installed Jazzy
  `mapper_params_online_async.yaml`. Every changed line is marked `(project change)`:
  resolution 0.025 m, max range 3.5 m, travel thresholds 0.15 m / 0.25 rad, loop closing **off**,
  correlation search 0.3 m, smear 0.03, coarse angle ±0.1 rad, variance penalties 2.0 / 4.0.
- Why these changes: five earlier live attempts produced rotated or sheared walls in the centre and
  left of the maze. `tools/scan_lag_probe.py` proved the scans are consistent with the odometry
  pose at their timestamps (99–100 % of beam endpoints on reference walls, yaw offset 0.000 rad,
  even while spinning at 1 rad/s). So the errors came from the scan matcher/loop closure in this
  repetitive maze. Parameters were tuned **offline** by replaying the recorded attempt-6 bag
  (`tools/offline_slam.sh`, A = original parameters vs C = final). The final map was then produced
  **live** with the final parameters.
- Evidence: `evidence_bags/slam_mapping_run` (scan, odom, wheel_odom, tf, tf_static, clock, map;
  kept outside git, 40 MB), `docs/evidence/slam_mapping_drive.log`, `docs/evidence/maze_slam_x4.png`.

## Validation (`tools/map_quality.py maps/slam/maze_slam.yaml`)

| Metric | Live SLAM map | Bag replay, original params (A) | Bag replay, final params (C) |
|---|---|---|---|
| wall cells within 0.05 m of reference wall | 87.7 % | 52.0 % | 83.1 % |
| wall cells within 0.10 m | 99.8 % | 75.9 % | 98.8 % |
| median / p95 wall distance | 0.019 / 0.074 m | 0.036 / 0.205 m | 0.019 / 0.079 m |
| reference free space observed free | 76.6 % | 66.1 % | 76.6 % |

The remaining "not observed" free space is mostly the half-cell band next to each wall, where the
SLAM walls are about one cell thicker than the true 0.05 m walls. Corridor connectivity: the DQN
lattice built from the SLAM map has 36 nodes and 36 edges, **identical** to the lattice built
from the reference geometry.

Map size 123 × 124 cells at 0.025 m, origin (−1.514, −1.536, 0), `free_thresh` 0.196,
`occupied_thresh` 0.65 (as written by `map_saver_cli`).
