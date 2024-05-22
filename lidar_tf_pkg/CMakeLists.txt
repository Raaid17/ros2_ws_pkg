cmake_minimum_required(VERSION 3.8)
project(lidar_tf_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

# Add executable for rocking_motion_node
add_executable(rocking_motion_node src/rocking_motion_node.cpp)
ament_target_dependencies(rocking_motion_node
  rclcpp
  std_msgs
  dynamixel_sdk
)

# Add executable for lidar_3d_node
add_executable(lidar_3d_node src/lidar_3d_node.cpp)
ament_target_dependencies(lidar_3d_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  octomap
  octomap_msgs
)

install(TARGETS
  rocking_motion_node
  lidar_3d_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing section
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

