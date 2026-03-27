#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y wget gnupg

wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | sudo apt-key add -
echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu24 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list

sudo apt update
sudo apt install -y openvino libze1 libze-intel-gpu1 intel-opencl-icd clinfo

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
