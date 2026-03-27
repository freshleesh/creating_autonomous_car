#!/bin/bash
# Quick test script for camera_lidar_calibration

set -e

echo "=== Testing camera_lidar_calibration ==="
cd /home/nuc14/creating_autonomous_car_ws/src/creating_autonomous_car
source install/setup.bash

echo ""
echo "1. Checking package installation..."
ros2 pkg list | grep camera_lidar_calibration && echo "✓ Package found" || (echo "✗ Package not found" && exit 1)

echo ""
echo "2. Checking config files..."
CONFIG_DIR="install/camera_lidar_calibration/share/camera_lidar_calibration/config"
ls -1 $CONFIG_DIR/*.yaml | while read f; do
    echo "  - $(basename $f)"
done

echo ""
echo "3. Checking parameter structure in config..."
grep -E "extrinsic\.(translation|rotation)\." $CONFIG_DIR/camera_extrinsic_calibration.yaml && echo "✓ New parameter structure" || echo "✗ Old parameter structure"

echo ""
echo "4. Verifying cy parameter is removed from README..."
grep -i "^.*cy.*:" perception/camera_lidar_calibration/README.md && echo "✗ cy still in README!" || echo "✓ cy removed from README"

echo ""
echo "=== Setup complete! ==="
echo "To run the calibration node:"
echo "  ros2 launch camera_lidar_calibration calibration.launch.py"
echo ""
echo "Make sure camera and lidar nodes are running first!"
