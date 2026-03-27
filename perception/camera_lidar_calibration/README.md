# camera_lidar_calibration

Interactive hand-tuning tool for camera-LiDAR extrinsic calibration in ROS 2.

<img src="assets/image.png" alt="camera lidar calibration demo" width="1280" />

## 1. Prerequisites and Installation

Tested environment:

- ROS 2 Humble
- Camera: `see3cam_24cug`
- LiDAR: `Livox Mid-360`

### Dependencies

Version floor recommendation (`>=`) based on current verified setup:

- `cv_bridge >= 3.2.1`
- `pcl_conversions >= 2.4.5`
- `pcl_ros >= 2.4.5`
- `image_transport >= 3.1.12`
- `OpenCV >= 4.5.4`
- `PCL >= 1.12.1`
- `Eigen3 >= 3.4.0`
- `yaml-cpp >= 0.7.0`

Install dependencies from your workspace root:

```bash
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
colcon build --packages-select camera_lidar_calibration
source install/setup.bash
```

### Camera intrinsic parameter file (recommended: `camera_calibration` package)

This package reads camera intrinsics from:

- `config/camera_intrinsic_calibration.yaml`

Recommended workflow:

1. Run ROS 2 `camera_calibration` to calibrate your camera.
2. Save/export the result YAML.
3. Copy values into `config/camera_intrinsic_calibration.yaml`.

The loader expects at least these fields:

```yaml
camera_matrix:
  data: [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
distortion_coefficients:
  data: [k1, k2, p1, p2, k3]
```

## 2. How to Use

Run:

```bash
source /ros2_ws/install/setup.bash
ros2 launch camera_lidar_calibration calibration.launch.py
```

### Tune parameters in `rqt`

Open dynamic parameter UI:

```bash
rqt
```

In `rqt`, open `Plugins > Configuration > Dynamic Reconfigure`, then select node:

- `extrinsic_calibration_by_hand`

Adjust parameters and check overlay in real time.

### Save parameters with Enter key

While the node is running, press `Enter` in the launch terminal.

Current values are saved to:

- `config/general.yaml`
- `config/camera_extrinsic_calibration.yaml`

### Parameter meanings

Extrinsic parameters (`config/camera_extrinsic_calibration.yaml`):

- `extrinsic.translation.x`, `y`, `z`: LiDAR-to-camera translation (meters)
- `extrinsic.rotation.roll`, `pitch`, `yaw`: LiDAR-to-camera rotation (degrees)

General processing parameters (`config/general.yaml`):

- `image_topic`: input camera image topic
- `lidar_topic`: input point cloud topic
- `debug_topic`: output overlay image topic
- `roi_x_min/max`, `roi_y_min/max`, `roi_z_min/max`: 3D ROI bounds in LiDAR frame (meters)
- `voxel_size`: pre-projection voxel downsampling size (m)
- `decay_time`: temporal accumulation window (seconds, `0` disables accumulation)
- `post_merge_voxel_size`: voxel size after merge (`0` disables)
- `max_projected_points`: cap on projected points per frame (`0` = unlimited)
- `draw_point_radius`: projected point radius in pixels
- `profile_timing`: print timing profile logs
