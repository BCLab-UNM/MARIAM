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

include_directories(include)

# Create the existing executable
add_executable(dual_waypoint_publisher src/dual_waypoint_publisher.cpp)
ament_target_dependencies(dual_waypoint_publisher
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

# Create the dual_linear_constant_velocity_open_loop executable
add_executable(dual_linear_constant_velocity_open_loop src/dual_linear_constant_velocity_open_loop.cpp)
ament_target_dependencies(dual_linear_constant_velocity_open_loop
  rclcpp
  geometry_msgs
  nav_msgs
)

# Create the dual_linear_constant_velocity_closed_loop executable
add_executable(dual_linear_constant_velocity_closed_loop src/dual_linear_constant_velocity_closed_loop.cpp)
ament_target_dependencies(dual_linear_constant_velocity_closed_loop
  rclcpp
  geometry_msgs
  nav_msgs
)

# Create the dual_linear_sinusoidal_velocity_open_loop executable
add_executable(dual_linear_sinusoidal_velocity_open_loop src/dual_linear_sinusoidal_velocity_open_loop.cpp)
ament_target_dependencies(dual_linear_sinusoidal_velocity_open_loop
  rclcpp
  geometry_msgs
  nav_msgs
)

# Create the dual_linear_sinusoidal_acceleration_open_loop executable
add_executable(dual_linear_sinusoidal_acceleration_open_loop src/dual_linear_sinusoidal_acceleration_open_loop.cpp)
ament_target_dependencies(dual_linear_sinusoidal_acceleration_open_loop
  rclcpp
  geometry_msgs
  nav_msgs
)

# Create the dual_curved_constant_velocity executable
add_executable(dual_curved_constant_velocity src/dual_curved_constant_velocity.cpp)
ament_target_dependencies(dual_curved_constant_velocity
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_msgs 
)

# Create the vicom_dual_linear_constant_velocity_open_loop executable
add_executable(vicom_dual_linear_constant_velocity_open_loop src/vicom_dual_linear_constant_velocity_open_loop.cpp)
ament_target_dependencies(vicom_dual_linear_constant_velocity_open_loop
  rclcpp
  geometry_msgs
  nav_msgs
)

# Create the trajectory_follower executable
add_executable(trajectory_follower src/trajectory_follower.cpp)
ament_target_dependencies(trajectory_follower
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install the executables
install(TARGETS
  dual_waypoint_publisher
  robot_follower
  dual_linear_constant_velocity_open_loop
  dual_linear_constant_velocity_closed_loop
  dual_linear_sinusoidal_velocity_open_loop
  dual_linear_sinusoidal_acceleration_open_loop
  dual_curved_constant_velocity
  vicom_dual_linear_constant_velocity_open_loop
  trajectory_follower
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