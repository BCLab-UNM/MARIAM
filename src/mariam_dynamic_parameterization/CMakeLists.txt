cmake_minimum_required(VERSION 3.8)
project(mariam_dynamic_parameterization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable, specify dependencies for centralized_parameterizer
add_executable(centralized_parameterizer src/centralized_parameterizer.cpp)
ament_target_dependencies(centralized_parameterizer
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# Add executable, specify dependencies for distributed_parameterizer
add_executable(distributed_parameterizer src/distributed_parameterizer.cpp)
ament_target_dependencies(distributed_parameterizer
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

# Add executable, specify dependencies for agent_displacement_estimator
add_executable(agent_displacement_estimator src/agent_displacement_estimator.cpp)
ament_target_dependencies(agent_displacement_estimator
  rclcpp
  std_msgs
)

# Install executables
install(TARGETS
  centralized_parameterizer
  distributed_parameterizer
  agent_displacement_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()