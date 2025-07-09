cmake_minimum_required(VERSION 3.8)
project(mariam_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# Create the existing executable
add_executable(experiment_dual_waypoint_publisher src/experiment_dual_waypoint_publisher.cpp)
ament_target_dependencies(experiment_dual_waypoint_publisher
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs 
)

# Create the new robot follower executable
add_executable(robot_follower src/robot_follower.cpp)
ament_target_dependencies(robot_follower
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs 
)

# Create the new dual_cmd_vel_distance executable
add_executable(dual_cmd_vel_distance src/dual_cmd_vel_distance.cpp)
ament_target_dependencies(dual_cmd_vel_distance
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs 
)

# Install the executables
install(TARGETS
  experiment_dual_waypoint_publisher
  robot_follower
  dual_cmd_vel_distance
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
install(
  DIRECTORY config maps launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()