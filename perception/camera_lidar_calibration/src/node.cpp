#include "camera_lidar_calibration/node.hpp"

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

#include <yaml-cpp/yaml.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <sstream>

namespace camera_lidar_calibration
{

// ──────────────────────────────────────────────────────────────────────────────
// Utility: build a parameter descriptor with a float range (step=0 → free)
// ──────────────────────────────────────────────────────────────────────────────
static rcl_interfaces::msg::ParameterDescriptor fp_desc(
  const std::string & description, double from, double to)
{
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value   = to;
  range.step       = 0.0;

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description          = description;
  desc.floating_point_range = {range};
  return desc;
}

static rcl_interfaces::msg::ParameterDescriptor int_desc(
  const std::string & description, int64_t from, int64_t to)
{
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value   = to;
  range.step       = 1;

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description   = description;
  desc.integer_range = {range};
  return desc;
}

// ──────────────────────────────────────────────────────────────────────────────
// Constructor
// ──────────────────────────────────────────────────────────────────────────────
ExtrinsicCalibrationNode::ExtrinsicCalibrationNode(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_calibration_by_hand", options)
{
  // ── Static / infrastructure parameters ──────────────────────────────────
  declare_parameter("image_topic",           "/image_raw");
  declare_parameter("image_transport",       "raw");
  declare_parameter("lidar_topic",           "/livox/lidar");
  declare_parameter("debug_topic",           "/extrinsic_debug_image");
  declare_parameter("camera_intrinsic_yaml", "");
  declare_parameter("save_on_enter", true);
  declare_parameter("save_general_yaml", "");
  declare_parameter("save_params_yaml", "");

  // ── Load camera intrinsics ───────────────────────────────────────────────
  const auto intrinsic_path = get_parameter("camera_intrinsic_yaml").as_string();
  loadCameraIntrinsics(intrinsic_path);

  // ── Core extrinsic parameters (overridden by YAML params file at launch) ──
  declare_parameter("x", 0.006253, fp_desc("Translation X (m)", -5.0, 5.0));
  declare_parameter("y", -0.018056, fp_desc("Translation Y (m)", -5.0, 5.0));
  declare_parameter("z", -0.114992, fp_desc("Translation Z (m)", -5.0, 5.0));
  declare_parameter("roll", 126.295327, fp_desc("Rotation Roll  (deg)", -180.0, 180.0));
  declare_parameter("pitch", -88.042738, fp_desc("Rotation Pitch (deg)", -180.0, 180.0));
  declare_parameter("yaw", -33.329511, fp_desc("Rotation Yaw   (deg)", -180.0, 180.0));
  declare_parameter("cy", camera_matrix_.at<double>(1, 2), fp_desc("Principal point cy (px)", 0.0, 2000.0));

  // ── ROI parameters (LiDAR frame, metres) ─────────────────────────────────
  declare_parameter("roi_x_min", -20.0, fp_desc("ROI min X (m)", -100.0,   0.0));
  declare_parameter("roi_x_max",  20.0, fp_desc("ROI max X (m)",    0.0, 100.0));
  declare_parameter("roi_y_min", -20.0, fp_desc("ROI min Y (m)", -100.0,   0.0));
  declare_parameter("roi_y_max",  20.0, fp_desc("ROI max Y (m)",    0.0, 100.0));
  declare_parameter("roi_z_min",  -2.0, fp_desc("ROI min Z (m)",  -10.0,   0.0));
  declare_parameter("roi_z_max",   5.0, fp_desc("ROI max Z (m)",    0.0,  50.0));

  // ── Processing parameters ─────────────────────────────────────────────────
  declare_parameter("voxel_size",  0.1, fp_desc("Voxel leaf size (m)",   0.0, 1.0));
  declare_parameter("decay_time",  0.0, fp_desc("Point decay time (s)",  0.0, 5.0));
  declare_parameter(
    "post_merge_voxel_size", 0.0,
    fp_desc("Post-merge voxel leaf size (m, 0=disabled)", 0.0, 1.0));
  declare_parameter(
    "max_projected_points", 40000,
    int_desc("Maximum points projected per frame (0=unlimited)", 0, 1000000));
  declare_parameter(
    "draw_point_radius", 1,
    int_desc("Projected point radius in pixels", 1, 10));
  declare_parameter("profile_timing", true);

  // ── Register dynamic parameter callback ──────────────────────────────────
  param_cb_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & p) {
      return onParamsChanged(p);
    });

  // Build initial T from declared defaults
  updateExtrinsic();

  // ── Subscriptions ─────────────────────────────────────────────────────────
  const auto image_topic = get_parameter("image_topic").as_string();
  const auto image_transport_name = get_parameter("image_transport").as_string();
  const auto lidar_topic = get_parameter("lidar_topic").as_string();
  const auto debug_topic = get_parameter("debug_topic").as_string();
  const auto qos = rclcpp::SensorDataQoS();

  image_sub_ = image_transport::create_subscription(
    this,
    image_topic,
    std::bind(&ExtrinsicCalibrationNode::imageCallback, this, std::placeholders::_1),
    image_transport_name,
    qos.get_rmw_qos_profile());
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic, qos,
    std::bind(&ExtrinsicCalibrationNode::lidarCallback, this, std::placeholders::_1));

  pub_image_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, 1);

  if (get_parameter("save_on_enter").as_bool()) {
    stdin_thread_ = std::thread(&ExtrinsicCalibrationNode::stdinSaveLoop, this);
    stdin_thread_.detach();
  }

  RCLCPP_INFO(get_logger(),
    "ExtrinsicCalibrationNode ready.\n"
    "  image  → %s\n"
    "  transport → %s\n"
    "  lidar  → %s\n"
    "  output → %s\n"
    "Press ENTER in this terminal to save current params to YAML.\n"
    "Use rqt (Plugins > Configuration > Dynamic Reconfigure) to tune parameters.",
    image_topic.c_str(), image_transport_name.c_str(), lidar_topic.c_str(), debug_topic.c_str());
}

// ──────────────────────────────────────────────────────────────────────────────
// Calibration loading
// ──────────────────────────────────────────────────────────────────────────────
void ExtrinsicCalibrationNode::loadCameraIntrinsics(const std::string & yaml_path)
{
  YAML::Node cfg = YAML::LoadFile(yaml_path);

  const auto cam_data  = cfg["camera_matrix"]["data"].as<std::vector<double>>();
  const auto dist_data = cfg["distortion_coefficients"]["data"].as<std::vector<double>>();

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 9; ++i) {
    camera_matrix_.at<double>(i / 3, i % 3) = cam_data[i];
  }

  dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_data.size()), CV_64F);
  for (int i = 0; i < static_cast<int>(dist_data.size()); ++i) {
    dist_coeffs_.at<double>(0, i) = dist_data[i];
  }

  RCLCPP_INFO(get_logger(), "Camera matrix loaded from: %s", yaml_path.c_str());
}

Eigen::Matrix4d ExtrinsicCalibrationNode::buildExtrinsicMatrix(
  double x, double y, double z,
  double roll_deg, double pitch_deg, double yaw_deg) const
{
  constexpr double deg2rad = M_PI / 180.0;
  const double r = roll_deg  * deg2rad;
  const double p = pitch_deg * deg2rad;
  const double yw = yaw_deg  * deg2rad;

  Eigen::Matrix3d Rx, Ry, Rz;
  Rx << 1,          0,           0,
        0,  std::cos(r), -std::sin(r),
        0,  std::sin(r),  std::cos(r);

  Ry <<  std::cos(p), 0, std::sin(p),
                    0, 1,           0,
        -std::sin(p), 0, std::cos(p);

  Rz << std::cos(yw), -std::sin(yw), 0,
        std::sin(yw),  std::cos(yw), 0,
                   0,             0, 1;

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Rz * Ry * Rx;
  T(0, 3) = x;
  T(1, 3) = y;
  T(2, 3) = z;
  return T;
}

void ExtrinsicCalibrationNode::updateExtrinsic()
{
  const double x     = get_parameter("x").as_double();
  const double y     = get_parameter("y").as_double();
  const double z     = get_parameter("z").as_double();
  const double roll  = get_parameter("roll").as_double();
  const double pitch = get_parameter("pitch").as_double();
  const double yaw   = get_parameter("yaw").as_double();

  std::lock_guard<std::mutex> lk(extrinsic_mutex_);
  T_lidar_to_cam_ = buildExtrinsicMatrix(x, y, z, roll, pitch, yaw);
}

// ──────────────────────────────────────────────────────────────────────────────
// Dynamic parameter callback
// ──────────────────────────────────────────────────────────────────────────────
rcl_interfaces::msg::SetParametersResult
ExtrinsicCalibrationNode::onParamsChanged(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::set<std::string> kExtrinsicNames =
    {"x", "y", "z", "roll", "pitch", "yaw"};

  // Helper: use new value if present in the update, else current node value
  auto get_val = [&](const std::string & name) -> double {
    for (const auto & p : params) {
      if (p.get_name() == name) { return p.as_double(); }
    }
    return get_parameter(name).as_double();
  };

  bool needs_rebuild = false;
  for (const auto & p : params) {
    if (kExtrinsicNames.count(p.get_name())) { needs_rebuild = true; break; }
  }

  if (needs_rebuild) {
    const auto T = buildExtrinsicMatrix(
      get_val("x"), get_val("y"), get_val("z"),
      get_val("roll"), get_val("pitch"), get_val("yaw"));
    {
      std::lock_guard<std::mutex> lk(extrinsic_mutex_);
      T_lidar_to_cam_ = T;
    }
    RCLCPP_INFO(get_logger(),
      "[param update] x=%.4f y=%.4f z=%.4f roll=%.3f pitch=%.3f yaw=%.3f",
      get_val("x"), get_val("y"), get_val("z"),
      get_val("roll"), get_val("pitch"), get_val("yaw"));
  }

  return result;
}

// ──────────────────────────────────────────────────────────────────────────────
// Filters
// ──────────────────────────────────────────────────────────────────────────────
pcl::PointCloud<pcl::PointXYZI>::Ptr
ExtrinsicCalibrationNode::applyRoiFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  const RoiBounds & roi) const
{
  auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  out->reserve(cloud->size());

  for (const auto & pt : *cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (pt.x >= roi.x_min && pt.x <= roi.x_max &&
        pt.y >= roi.y_min && pt.y <= roi.y_max &&
        pt.z >= roi.z_min && pt.z <= roi.z_max)
    {
      out->push_back(pt);
    }
  }
  return out;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
ExtrinsicCalibrationNode::applyVoxelFilter(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
  float leaf) const
{
  if (leaf <= 0.0f || cloud->empty()) { return cloud; }

  auto out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

// ──────────────────────────────────────────────────────────────────────────────
// LiDAR callback (update reusable filtered cloud buffer)
// ──────────────────────────────────────────────────────────────────────────────
void ExtrinsicCalibrationNode::lidarCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  const RoiBounds roi{
    get_parameter("roi_x_min").as_double(),
    get_parameter("roi_x_max").as_double(),
    get_parameter("roi_y_min").as_double(),
    get_parameter("roi_y_max").as_double(),
    get_parameter("roi_z_min").as_double(),
    get_parameter("roi_z_max").as_double()};
  const float voxel_size = static_cast<float>(get_parameter("voxel_size").as_double());
  const double decay = get_parameter("decay_time").as_double();

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*cloud_msg, *cloud);

  auto filtered = applyRoiFilter(cloud, roi);
  filtered = applyVoxelFilter(filtered, voxel_size);

  const double stamp = this->now().seconds();
  const double cutoff = stamp - std::max(decay, 0.0);

  std::lock_guard<std::mutex> lk(buffer_mutex_);
  if (!filtered->empty()) {
    point_buffer_.push_back({stamp, filtered});
  }
  while (!point_buffer_.empty() &&
         point_buffer_.front().timestamp < cutoff)
  {
    point_buffer_.pop_front();
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// Image callback (project latest LiDAR buffer onto each image frame)
// ──────────────────────────────────────────────────────────────────────────────
void ExtrinsicCalibrationNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg)
{
  using Clock = std::chrono::steady_clock;
  const auto t0 = Clock::now();
  auto ms_since = [](const Clock::time_point & from, const Clock::time_point & to) {
      return std::chrono::duration<double, std::milli>(to - from).count();
    };

  const double now = this->now().seconds();
  double fps = 0.0;
  if (last_stamp_ && (now - *last_stamp_) > 0.0) {
    fps = 1.0 / (now - *last_stamp_);
  }
  last_stamp_ = now;

  const RoiBounds roi{
    get_parameter("roi_x_min").as_double(),
    get_parameter("roi_x_max").as_double(),
    get_parameter("roi_y_min").as_double(),
    get_parameter("roi_y_max").as_double(),
    get_parameter("roi_z_min").as_double(),
    get_parameter("roi_z_max").as_double()};
  const float voxel_size = static_cast<float>(get_parameter("voxel_size").as_double());
  const double decay = get_parameter("decay_time").as_double();
  const double post_merge_voxel = get_parameter("post_merge_voxel_size").as_double();
  const int max_projected_points = get_parameter("max_projected_points").as_int();
  const int draw_point_radius = get_parameter("draw_point_radius").as_int();
  const double cy = get_parameter("cy").as_double();
  const bool profile_timing = get_parameter("profile_timing").as_bool();
  const auto t_params = Clock::now();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat & img = cv_ptr->image;
  const auto t_img = Clock::now();

  auto merged = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const double cutoff = now - std::max(decay, 0.0);
  std::size_t buffered_frames = 0;
  double oldest_age = 0.0;
  double newest_age = 0.0;
  {
    std::lock_guard<std::mutex> lk(buffer_mutex_);
    while (!point_buffer_.empty() &&
           point_buffer_.front().timestamp < cutoff)
    {
      point_buffer_.pop_front();
    }

    std::size_t total_points = 0;
    for (const auto & entry : point_buffer_) {
      total_points += entry.cloud->size();
    }
    buffered_frames = point_buffer_.size();
    if (!point_buffer_.empty()) {
      oldest_age = now - point_buffer_.front().timestamp;
      newest_age = now - point_buffer_.back().timestamp;
    }
    merged->reserve(total_points);
    for (const auto & entry : point_buffer_) {
      merged->insert(merged->end(), entry.cloud->begin(), entry.cloud->end());
    }
  }
  const auto t_buffer = Clock::now();

  if (post_merge_voxel > 0.0 && !merged->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    const float leaf = static_cast<float>(post_merge_voxel);
    auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    vg.setInputCloud(merged);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*downsampled);
    merged = downsampled;
  }
  const auto t_post = Clock::now();

  if (merged->empty()) {
    drawOverlay(img, 0, fps, decay, cy, voxel_size, roi);
    auto out = cv_ptr->toImageMsg();
    out->header = image_msg->header;
    pub_image_->publish(*out);
    return;
  }

  Eigen::Matrix4d T;
  {
    std::lock_guard<std::mutex> lk(extrinsic_mutex_);
    T = T_lidar_to_cam_;
  }
  cv::Mat cam_mat = camera_matrix_.clone();
  cam_mat.at<double>(1, 2) = cy;

  std::size_t sample_step = 1;
  if (max_projected_points > 0 &&
      static_cast<int>(merged->size()) > max_projected_points)
  {
    sample_step = static_cast<std::size_t>(std::ceil(
      static_cast<double>(merged->size()) / static_cast<double>(max_projected_points)));
  }

  const std::size_t expected = (merged->size() + sample_step - 1) / sample_step;
  std::vector<cv::Point3f> pts_cam;
  std::vector<float> intensities;
  pts_cam.reserve(expected);
  intensities.reserve(expected);

  const double t00 = T(0, 0), t01 = T(0, 1), t02 = T(0, 2), t03 = T(0, 3);
  const double t10 = T(1, 0), t11 = T(1, 1), t12 = T(1, 2), t13 = T(1, 3);
  const double t20 = T(2, 0), t21 = T(2, 1), t22 = T(2, 2), t23 = T(2, 3);

  for (std::size_t idx = 0; idx < merged->size(); idx += sample_step) {
    const auto & pt = (*merged)[idx];
    const double x = static_cast<double>(pt.x);
    const double y = static_cast<double>(pt.y);
    const double z = static_cast<double>(pt.z);
    const double cam_x = t00 * x + t01 * y + t02 * z + t03;
    const double cam_y = t10 * x + t11 * y + t12 * z + t13;
    const double cam_z = t20 * x + t21 * y + t22 * z + t23;
    if (cam_z <= 0.0) { continue; }
    pts_cam.emplace_back(
      static_cast<float>(cam_x),
      static_cast<float>(cam_y),
      static_cast<float>(cam_z));
    intensities.push_back(pt.intensity);
  }
  const auto t_transform = Clock::now();

  if (pts_cam.empty()) {
    drawOverlay(img, 0, fps, decay, cy, voxel_size, roi);
    auto out = cv_ptr->toImageMsg();
    out->header = image_msg->header;
    pub_image_->publish(*out);
    return;
  }

  std::vector<cv::Point2f> img_pts;
  const cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  const cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::projectPoints(pts_cam, rvec, tvec, cam_mat, dist_coeffs_, img_pts);
  const auto t_project = Clock::now();

  const int h = img.rows;
  const int w = img.cols;

  float i_min = *std::min_element(intensities.begin(), intensities.end());
  float i_max = *std::max_element(intensities.begin(), intensities.end());
  const float denom = (i_max - i_min > 1e-6f) ? (i_max - i_min) : 1.0f;

  int n_drawn = 0;
  for (std::size_t i = 0; i < img_pts.size(); ++i) {
    const float u_f = img_pts[i].x;
    const float v_f = img_pts[i].y;

    if (!std::isfinite(u_f) || !std::isfinite(v_f)) { continue; }
    if (std::abs(u_f) > 1e6f || std::abs(v_f) > 1e6f) { continue; }

    const int u = static_cast<int>(std::round(u_f));
    const int v = static_cast<int>(std::round(v_f));
    if (u < 0 || u >= w || v < 0 || v >= h) { continue; }

    const float nd = std::clamp((intensities[i] - i_min) / denom, 0.0f, 1.0f);
    const cv::Scalar color(
      static_cast<int>(255.0f * nd),
      static_cast<int>(255.0f * (1.0f - std::abs(nd - 0.5f) * 2.0f)),
      static_cast<int>(255.0f * (1.0f - nd)));

    if (draw_point_radius <= 1) {
      auto & px = img.at<cv::Vec3b>(v, u);
      px[0] = static_cast<unsigned char>(color[0]);
      px[1] = static_cast<unsigned char>(color[1]);
      px[2] = static_cast<unsigned char>(color[2]);
    } else {
      cv::circle(img, {u, v}, draw_point_radius, color, cv::FILLED);
    }
    ++n_drawn;
  }
  const auto t_draw = Clock::now();

  drawOverlay(img, n_drawn, fps, decay, cy, voxel_size, roi);
  const auto t_overlay = Clock::now();

  auto out_msg = cv_ptr->toImageMsg();
  out_msg->header = image_msg->header;
  pub_image_->publish(*out_msg);
  const auto t_publish = Clock::now();

  if (profile_timing) {
    const double total = ms_since(t0, t_publish);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[timing ms] total=%.2f params=%.2f img=%.2f "
      "buffer=%.2f post=%.2f transform=%.2f project=%.2f draw=%.2f "
      "overlay=%.2f publish=%.2f | decay=%.2f frames=%zu merged=%zu drawn=%d age(old/new)=%.3f/%.3f",
      total,
      ms_since(t0, t_params),
      ms_since(t_params, t_img),
      ms_since(t_img, t_buffer),
      ms_since(t_buffer, t_post),
      ms_since(t_post, t_transform),
      ms_since(t_transform, t_project),
      ms_since(t_project, t_draw),
      ms_since(t_draw, t_overlay),
      ms_since(t_overlay, t_publish),
      decay, buffered_frames, merged->size(), n_drawn, oldest_age, newest_age);
  }
}

bool ExtrinsicCalibrationNode::saveCurrentParamsToYaml()
{
  const auto general_yaml = get_parameter("save_general_yaml").as_string();
  const auto params_yaml = get_parameter("save_params_yaml").as_string();
  if (general_yaml.empty() || params_yaml.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "save_general_yaml/save_params_yaml is empty; cannot persist parameters.");
    return false;
  }

  auto fmt_double = [](double v) {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(6) << v;
      return oss.str();
    };

  std::ostringstream general_out;
  general_out
    << "extrinsic_calibration_by_hand:\n"
    << "  ros__parameters:\n"
    << "    image_topic: " << get_parameter("image_topic").as_string() << "\n"
    << "    lidar_topic: " << get_parameter("lidar_topic").as_string() << "\n"
    << "    debug_topic: " << get_parameter("debug_topic").as_string() << "\n"
    << "    roi_x_min: " << fmt_double(get_parameter("roi_x_min").as_double()) << "\n"
    << "    roi_x_max: " << fmt_double(get_parameter("roi_x_max").as_double()) << "\n"
    << "    roi_y_min: " << fmt_double(get_parameter("roi_y_min").as_double()) << "\n"
    << "    roi_y_max: " << fmt_double(get_parameter("roi_y_max").as_double()) << "\n"
    << "    roi_z_min: " << fmt_double(get_parameter("roi_z_min").as_double()) << "\n"
    << "    roi_z_max: " << fmt_double(get_parameter("roi_z_max").as_double()) << "\n"
    << "    voxel_size: " << fmt_double(get_parameter("voxel_size").as_double()) << "\n"
    << "    decay_time: " << fmt_double(get_parameter("decay_time").as_double()) << "\n"
    << "    post_merge_voxel_size: " << fmt_double(get_parameter("post_merge_voxel_size").as_double()) << "\n"
    << "    max_projected_points: " << get_parameter("max_projected_points").as_int() << "\n"
    << "    draw_point_radius: " << get_parameter("draw_point_radius").as_int() << "\n"
    << "    profile_timing: " << (get_parameter("profile_timing").as_bool() ? "true" : "false") << "\n";

  std::ostringstream params_out;
  params_out
    << "extrinsic_calibration_by_hand:\n"
    << "  ros__parameters:\n"
    << "    x: " << fmt_double(get_parameter("x").as_double()) << "\n"
    << "    y: " << fmt_double(get_parameter("y").as_double()) << "\n"
    << "    z: " << fmt_double(get_parameter("z").as_double()) << "\n"
    << "    roll: " << fmt_double(get_parameter("roll").as_double()) << "\n"
    << "    pitch: " << fmt_double(get_parameter("pitch").as_double()) << "\n"
    << "    yaw: " << fmt_double(get_parameter("yaw").as_double()) << "\n"
    << "    cy: " << fmt_double(get_parameter("cy").as_double()) << "\n";

  std::ofstream gfs(general_yaml, std::ios::out | std::ios::trunc);
  if (!gfs) {
    RCLCPP_ERROR(get_logger(), "Failed opening general yaml for write: %s", general_yaml.c_str());
    return false;
  }
  gfs << general_out.str();
  gfs.close();

  std::ofstream pfs(params_yaml, std::ios::out | std::ios::trunc);
  if (!pfs) {
    RCLCPP_ERROR(get_logger(), "Failed opening params yaml for write: %s", params_yaml.c_str());
    return false;
  }
  pfs << params_out.str();
  pfs.close();

  RCLCPP_INFO(
    get_logger(),
    "Saved current parameters to:\n  general: %s\n  params : %s",
    general_yaml.c_str(), params_yaml.c_str());
  return true;
}

void ExtrinsicCalibrationNode::stdinSaveLoop()
{
  std::ifstream tty("/dev/tty");
  std::istream * input = &std::cin;
  if (tty.is_open()) {
    input = &tty;
    RCLCPP_INFO(get_logger(), "ENTER hotkey listener attached to /dev/tty");
  } else {
    RCLCPP_WARN(get_logger(), "Could not open /dev/tty; falling back to stdin for save hotkey.");
  }

  std::string line;
  while (rclcpp::ok() && std::getline(*input, line)) {
    (void)line;
    saveCurrentParamsToYaml();
  }
}

// ──────────────────────────────────────────────────────────────────────────────
// HUD overlay
// ──────────────────────────────────────────────────────────────────────────────
void ExtrinsicCalibrationNode::drawOverlay(
  cv::Mat & img, int n_pts, double fps, double decay, double cy,
  double voxel_size, const RoiBounds & roi) const
{
  auto g = [&](const std::string & name) {
    return get_parameter(name).as_double();
  };

  std::ostringstream oss;
  const std::vector<std::string> lines = {
    [&]() {
      std::ostringstream s;
      s << std::fixed << std::setprecision(1)
        << "FPS: " << fps
        << "   pts: " << n_pts
        << "   decay: " << std::setprecision(2) << decay << "s";
      return s.str();
    }(),
    [&]() {
      std::ostringstream s;
      s << std::fixed << std::setprecision(4)
        << "x=" << g("x") << " m   y=" << g("y") << " m   z=" << g("z") << " m";
      return s.str();
    }(),
    [&]() {
      std::ostringstream s;
      s << std::fixed << std::setprecision(3)
        << "roll=" << g("roll") << " deg   pitch=" << g("pitch")
        << " deg   yaw=" << g("yaw") << " deg";
      return s.str();
    }(),
    [&]() {
      std::ostringstream s;
      s << std::fixed << std::setprecision(2) << "cy=" << cy << " px";
      return s.str();
    }(),
    [&]() {
      std::ostringstream s;
      s << std::fixed << std::setprecision(3)
        << "voxel=" << voxel_size << " m   "
        << "ROI x[" << std::setprecision(1) << roi.x_min
        << "," << roi.x_max << "]"
        << " y[" << roi.y_min << "," << roi.y_max << "]"
        << " z[" << roi.z_min << "," << roi.z_max << "]";
      return s.str();
    }(),
  };

  const auto font  = cv::FONT_HERSHEY_SIMPLEX;
  const double scale = 0.6;
  const int    thick = 1;
  const int    pad   = 6;
  int y_pos = 30;

  for (const auto & line : lines) {
    // Shadow for readability
    cv::putText(img, line, {pad + 1, y_pos + 1}, font, scale,
                {0, 0, 0}, thick + 1);
    cv::putText(img, line, {pad,     y_pos},     font, scale,
                {0, 255, 128}, thick);
    y_pos += 28;
  }
}

}  // namespace camera_lidar_calibration
