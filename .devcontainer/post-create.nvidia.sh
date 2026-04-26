#!/usr/bin/env bash
set -euo pipefail

sudo apt update

# ROS2 Jazzy dependencies for sensor drivers
sudo apt install -y \
  ros-jazzy-camera-info-manager \
  ros-jazzy-image-transport \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-geometry \
  ros-jazzy-pcl-ros \
  ros-jazzy-sophus \
  libgtsam-dev

mkdir -p /ws/build /ws/install /ws/log
sudo chmod 666 /dev/input/event* /dev/input/js* 2>/dev/null || true

cd /ws/src/creating_autonomous_car
bash build_packages_on_local_pc.sh
