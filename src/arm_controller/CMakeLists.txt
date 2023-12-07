cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(interbotix_moveit_interface REQUIRED)
# find_package(interbotix_moveit_interface_msgs REQUIRED)
# find_package(interbotix_xs_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# For executable pose_listener_moveit
add_executable(pose_listener_moveit src/pose_listener_moveit.cpp)
target_include_directories(pose_listener_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pose_listener_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pose_listener_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)
install(TARGETS pose_listener_moveit
  DESTINATION lib/${PROJECT_NAME}
) 

# For executable pos_control_moveit_interface
#add_executable(pos_control_moveit_interface src/pos_control_moveit_interface.cpp)
#target_include_directories(pos_control_moveit_interface PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(pos_control_moveit_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#ament_target_dependencies(
#  pos_control_moveit_interface
#  "moveit_ros_planning_interface"
#  "rclcpp"
#  "std_msgs"
#)
#install(TARGETS pos_control_moveit_interface
#  DESTINATION lib/${PROJECT_NAME}
#) 

# For executable experiment1
add_executable(experiment1 src/experiment1.cpp)
target_include_directories(experiment1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(experiment1 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  experiment1
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)
install(TARGETS experiment1
  DESTINATION lib/${PROJECT_NAME}
) 

# For executable box_pickup_listener_moveit
add_executable(box_pickup_listener_moveit src/box_pickup_listener_moveit.cpp)
target_include_directories(box_pickup_listener_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(box_pickup_listener_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  box_pickup_listener_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)
install(TARGETS box_pickup_listener_moveit
  DESTINATION lib/${PROJECT_NAME}
)


# Add launch file to share folder
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
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
