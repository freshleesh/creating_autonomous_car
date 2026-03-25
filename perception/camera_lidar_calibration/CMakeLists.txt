cmake_minimum_required(VERSION 3.14)
project(camera_lidar_calibration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── ROS dependencies ───────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)

# ── System dependencies ────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

# ── Target ────────────────────────────────────────────────────────────────────
add_executable(camera_lidar_calibration
  src/node.cpp
  src/main.cpp
)

target_include_directories(camera_lidar_calibration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(camera_lidar_calibration
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  image_transport
)

target_link_libraries(camera_lidar_calibration
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
)

target_compile_definitions(camera_lidar_calibration
  PRIVATE ${PCL_DEFINITIONS}
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS camera_lidar_calibration
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
