#!/bin/bash
cd /home/nuc14/creating_autonomous_car_ws/src/creating_autonomous_car
source install/setup.bash
ros2 launch camera_lidar_calibration calibration.launch.py
