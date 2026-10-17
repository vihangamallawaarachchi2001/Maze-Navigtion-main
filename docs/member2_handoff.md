# Member 2 hand-off: SLAM, map provenance, AMCL

Owner: `<Member 2 name / ID>`

## Delivered
- `config/slam_toolbox.yaml` (from installed Jazzy defaults; changes marked), `launch/slam.launch.py`.
- `mapping_drive` (scripted tour of all 36 cells) + `config/mapping_route.yaml` (`tools/make_mapping_route.py`).
- **SLAM map** `maps/slam/maze_slam.{yaml,pgm}` (live run). Reference geometry map is kept separately in `maps/reference/`.
- Evidence: `evidence_bags/slam_mapping_run` (rosbag, 40 MB, not in git), `docs/evidence/maze_slam_x4.png`,
  `docs/evidence/slam_mapping_drive.log`, `docs/map_provenance.md` (quality metrics, attempts, tuning).
- Tools: `map_snapshot.py`, `map_quality.py`, `scan_lag_probe.py`, `offline_slam.sh` + `tf_relay_drop_map.py`.

## Mode switch
SLAM mode = `simulation.launch.py` + `slam.launch.py` (SLAM owns `map→odom`).
Localisation mode = `simulation.launch.py` + `localization_navigation.launch.py` (map_server + AMCL own `map→odom`).
Never run both: stop SLAM (Ctrl+C) before starting localisation.

## Pose initialisation
The map frame equals the Gazebo world frame. AMCL starts with `set_initial_pose: true` at the spawn
(−1.25, −1.25, 0). After moving the robot or restarting Gazebo, use RViz **2D Pose Estimate** in map
coordinates (`run_trials` does this automatically with `/initialpose` after each reset).

## Results
Wall alignment: 99.8 % of SLAM wall cells within 0.10 m of the true walls (median 0.019 m). The SLAM
lattice topology is identical to the reference. AMCL vs ground truth at the end of each trial is logged
as `localization_error_m` in the trial CSVs.
