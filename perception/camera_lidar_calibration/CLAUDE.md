# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Package Overview

ROS 2 Humble C++ node for interactive hand-tuning of camera-LiDAR extrinsic calibration. Projects 3D LiDAR point clouds onto 2D camera images in real time so the operator can visually verify and adjust the transform. Parameters can be tuned live via `rqt` and saved by pressing ENTER in the terminal.

**Hardware targets**: see3cam_24cug (1280×720) + Livox Mid-360.

## Build & Run

Build from the workspace root (two levels up from this package):

```bash
colcon build --packages-select camera_lidar_calibration
source install/setup.bash
ros2 launch camera_lidar_calibration calibration.launch.py
```

Live parameter tuning (no restart needed):

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

Press **ENTER** in the node terminal to persist current parameters to both the `install/` and `src/` config directories.

There are no automated tests — validation is visual inspection of the `/extrinsic_debug_image` topic.

## Key Configuration Files

| File | Purpose |
|------|---------|
| [config/camera_intrinsic_calibration.yaml](config/camera_intrinsic_calibration.yaml) | Camera matrix, distortion coeffs (output of `ros2 run camera_calibration cameracalibrator`) |
| [config/camera_extrinsic_calibration.yaml](config/camera_extrinsic_calibration.yaml) | LiDAR→camera translation (m) + rotation (roll/pitch/yaw degrees) |
| [config/general.yaml](config/general.yaml) | Topic names, ROI bounds, voxel sizes, decay time, render settings |

Config files are installed to `share/camera_lidar_calibration/config/` and the launch file passes their absolute paths to the node as parameters.

## Architecture

Single ROS 2 node (`ExtrinsicCalibrationNode`) defined in [include/camera_lidar_calibration/node.hpp](include/camera_lidar_calibration/node.hpp) and implemented in [src/node.cpp](src/node.cpp).

**Processing pipeline per image frame:**

```
PointCloud2 → ROI filter → Voxel filter → Decay buffer (deque<FrameEntry>)
                                                    ↓
Image → merge all buffered clouds → optional post-merge voxel filter
     → sample if > max_projected_points
     → matrix-multiply with T_lidar_to_cam_ (cached 4×4 SE(3))
     → cv::projectPoints() with camera matrix + distortion
     → intensity→color mapping (blue=low, red=high)
     → draw circles + HUD overlay → publish /extrinsic_debug_image
```

**Threading**: `rclcpp::spin()` on the main thread; a detached background thread reads `/dev/tty` and calls `saveCurrentParamsToYaml()` on ENTER.

**Extrinsic representation**: The transform is stored as a 4×4 `Eigen::Matrix4d` (`T_lidar_to_cam_`), rebuilt by `buildExtrinsicMatrix()` whenever roll/pitch/yaw/tx/ty/tz parameters change via the dynamic-reconfigure callback `onParamsChanged()`.

**Parameter save**: `saveCurrentParamsToYaml()` writes to two locations — the installed share directory and the source `config/` directory — so iterative tuning survives rebuilds.

**Timing profile**: When `profile_timing: true` in `general.yaml`, the node logs wall-clock durations for 9 pipeline stages. Useful for diagnosing latency.

## ROS 2 Topics

| Direction | Topic (default) | Type |
|-----------|----------------|------|
| Subscribe | `/image_raw` | `sensor_msgs/Image` |
| Subscribe | `/livox/lidar` | `sensor_msgs/PointCloud2` |
| Publish | `/extrinsic_debug_image` | `sensor_msgs/Image` |

All subscriptions use **BestEffort** QoS to match Livox driver defaults.

## Workflow for New Calibration

1. Run `ros2 run camera_calibration cameracalibrator` on the camera; copy output to `camera_intrinsic_calibration.yaml`.
2. Start the node and open `rqt_reconfigure`.
3. Adjust tx/ty/tz (meters) and roll/pitch/yaw (degrees) until projected points align with image edges.
4. Press ENTER in the node terminal to save.
