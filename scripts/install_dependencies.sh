#!/usr/bin/env bash
set -euo pipefail

if [[ "$(lsb_release -sc)" != "noble" ]]; then
  echo "警告：本项目目标平台是 Ubuntu 24.04 (noble)。" >&2
fi

sudo apt-get update
sudo apt-get install -y \
  can-utils \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

source /opt/ros/jazzy/setup.bash
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rosdep install --from-paths "${workspace_root}/src" --ignore-src -r -y \
  --rosdistro jazzy
