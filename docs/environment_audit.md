# Environment audit

Audit date: 16 October 2026. Every value below was checked on this machine; nothing is copied from
PROJECT_STATE.md without verification.

## Host

| Item | Value (verified) |
|---|---|
| OS | Ubuntu 22.04.5 LTS (Jammy), VirtualBox VM (`/sys/class/dmi/id/sys_vendor` = innotek GmbH) |
| CPU / RAM | Initially 1 vCPU / 1.9 GiB. User resized the VM to 5 vCPU / 8 GiB |
| Disk | Initially 25 GB (8.5 GB free). User grew the disk and `/dev/sda3` to 50 GB |
| Graphics | Software rendering (Wayland session, no GPU passthrough) |
| sudo | Requires the user's password. Claude never entered it; the user ran the partition grow |
| Background load | GNOME shell + Claude desktop use about 2–3 of the 5 cores (from `ps`) |

## Historical paths from PROJECT_STATE.md

None of these existed on this machine: `/opt/ros`, `~/maze_testing`, `~/turtlebot3_ws`,
`~/Downloads/generate_ground_truth_map.py`. The project was therefore built fresh in `~/maze_testing`.
Historical claims about safe spawn, LiDAR offsets or the old world were **not** reused.

## Software stack (installed without sudo)

ROS 2 Jazzy has no apt packages for Ubuntu 22.04, so the stack was installed in user space with
RoboStack (conda-forge, `robostack-jazzy` channel) through micromamba:

| Component | Version / source |
|---|---|
| micromamba | 2.9.0 at `~/.local/bin/micromamba`, root `~/micromamba` |
| Environment | `ros_env` (Python 3.12) |
| ROS 2 | Jazzy (`ROS_DISTRO=jazzy`) |
| Gazebo | gz-sim 8.10.0 (Harmonic), physics engine DART (`gz-physics-dartsim-plugin`) |
| Nav2 | 1.3.x (navfn, RPP, MPPI, DWB, AMCL, map server present) |
| SLAM | slam_toolbox (Jazzy) |
| Bridge | ros_gz_bridge / ros_gz_sim (Jazzy) |
| Learning | PyTorch 2.14.1 (CPU), numpy 2.5.4 |
| git | from the conda environment |

Activate in every terminal with `source ~/maze_testing/env.sh`. This sets
`ROS_DOMAIN_ID=42` and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

Known tool quirk: `colcon build --symlink-install` fails with this setuptools
(`option --editable not recognized`), so use plain `colcon build`.

## Topic / frame contract (verified live)

| Topic | Type | Direction | Frame / notes |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | GZ→ROS | simulation time; all nodes `use_sim_time:=true` |
| `/scan` | sensor_msgs/LaserScan | GZ→ROS | `frame_id: base_scan`, 360 beams, 0.12–3.5 m, 5 Hz |
| `/odom` | nav_msgs/Odometry | GZ→ROS | `odom` → `base_footprint` (gz OdometryPublisher, see map_provenance.md) |
| `/wheel_odom` | nav_msgs/Odometry | GZ→ROS | DiffDrive wheel odometry, evidence only (not in TF) |
| `/tf` | tf2_msgs/TFMessage | GZ→ROS | only `odom → base_footprint` |
| `/tf_static`, `/tf` | | ROS | robot_state_publisher: `base_footprint → base_link → {base_scan, imu_link, wheels}` |
| `/joint_states` | sensor_msgs/JointState | GZ→ROS | wheel joints |
| `/bumper` | ros_gz_interfaces/Contacts | GZ→ROS | chassis contact sensor (collision counting) |
| `/cmd_vel` | geometry_msgs/**Twist** | ROS→GZ | Nav2 `enable_stamped_cmd_vel` default is false (checked in `nav2_util/twist_publisher.hpp`) |

TF chain: `map → odom` (SLAM *or* AMCL, never both) → `base_footprint` (OdometryPublisher) →
`base_link` (+0.010 m) → `base_scan` (−0.032, 0, +0.172). Measured `odom → base_scan` =
odom pose + (−0.032, 0, 0.182).

## Simulation performance

Real-time factor (`tools/rtf_probe.py`) is 0.2–0.4 depending on desktop load. All timing in
the results is reported in **both** simulation and wall time.
