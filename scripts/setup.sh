#!/usr/bin/env bash
# Ubuntu 24.04 only. sudo prompts in the user's terminal; no system upgrade.
set -e
. /etc/os-release
[ "$ID:$VERSION_ID" = ubuntu:24.04 ] || { echo 'Requires Ubuntu 24.04'; exit 1; }
sudo apt-get update
sudo apt-get install -y curl ca-certificates software-properties-common
sudo add-apt-repository -y universe
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
  release=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
    python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb" -o /tmp/maze-ros2-apt-source.deb
  sudo dpkg -i /tmp/maze-ros2-apt-source.deb
fi
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-demo-nodes-cpp ros-jazzy-rviz2 \
  ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-robot-state-publisher ros-jazzy-tf2-tools \
  ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-pil python3-pytest python3-matplotlib \
  python3-venv python3-reportlab mesa-utils
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
python3 -m venv --system-site-packages "$ROOT/.venv"
"$ROOT/.venv/bin/python" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
echo 'Setup complete. Source env.sh, then run bash scripts/build.sh.'
