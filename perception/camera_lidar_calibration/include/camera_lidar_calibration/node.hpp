#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>

#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace camera_lidar_calibration
{

class ExtrinsicCalibrationNode : public rclcpp::Node
{
public:
  explicit ExtrinsicCalibrationNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Calibration loading ──────────────────────────────────────────────────
  void loadCameraIntrinsics(const std::string & yaml_path);

  // ── Parameter helpers ────────────────────────────────────────────────────
  rcl_interfaces::msg::SetParametersResult onParamsChanged(
    const std::vector<rclcpp::Parameter> & params);
  void updateExtrinsic();
  Eigen::Matrix4d buildExtrinsicMatrix(
    double x, double y, double z,
    double roll_deg, double pitch_deg, double yaw_deg) const;

  // ── Processing ───────────────────────────────────────────────────────────
  struct RoiBounds
  {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
  };

  pcl::PointCloud<pcl::PointXYZI>::Ptr applyRoiFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    const RoiBounds & roi) const;
  pcl::PointCloud<pcl::PointXYZI>::Ptr applyVoxelFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
    float leaf_size) const;

  // ── Callbacks ────────────────────────────────────────────────────────────
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  // ── Rendering ────────────────────────────────────────────────────────────
  void drawOverlay(
    cv::Mat & img, int n_pts, double fps, double decay, double cy,
    double voxel_size, const RoiBounds & roi) const;
  bool saveCurrentParamsToYaml();
  void stdinSaveLoop();

  // ── Subscribers / publisher ──────────────────────────────────────────────
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  // ── Calibration data ─────────────────────────────────────────────────────
  cv::Mat camera_matrix_;   ///< 3×3 double intrinsic matrix
  cv::Mat dist_coeffs_;     ///< 1×N double distortion coefficients

  Eigen::Matrix4d      T_lidar_to_cam_;   ///< 4×4 extrinsic (LiDAR→camera)
  mutable std::mutex   extrinsic_mutex_;

  // ── Decay buffer ─────────────────────────────────────────────────────────
  struct FrameEntry {
    double timestamp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  };
  std::deque<FrameEntry> point_buffer_;
  std::mutex             buffer_mutex_;

  // ── FPS tracking ─────────────────────────────────────────────────────────
  std::optional<double> last_stamp_;

  // Keep the callback handle alive for the node's lifetime
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::thread stdin_thread_;
};

}  // namespace camera_lidar_calibration
