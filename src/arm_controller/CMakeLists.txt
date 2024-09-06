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
find_package(interbotix_xs_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(joy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

# For executable joy_moveit.cpp
add_executable(joy_moveit src/joy_moveit.cpp)
target_include_directories(joy_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joy_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)
install(TARGETS joy_moveit
  DESTINATION lib/${PROJECT_NAME}
)

# For executable joy_moveit_constrained.cpp
add_executable(joy_moveit_constrained src/joy_moveit_constrained.cpp)
target_include_directories(joy_moveit_constrained PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy_moveit_constrained PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joy_moveit_constrained
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
)
install(TARGETS joy_moveit_constrained
  DESTINATION lib/${PROJECT_NAME}
)

# For executable joy_input_handler.cpp
add_executable(joy_input_handler src/joy_input_handler.cpp)
target_include_directories(joy_input_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy_input_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joy_input_handler
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "interbotix_xs_msgs"
)
install(TARGETS joy_input_handler
  DESTINATION lib/${PROJECT_NAME}
)

# For executable admittance_controller.cpp
add_executable(admittance_controller src/admittance_controller.cpp)
target_include_directories(admittance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(admittance_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  admittance_controller
  "rclcpp"
  "std_msgs"
)
install(TARGETS admittance_controller
  DESTINATION lib/${PROJECT_NAME}
)

# For executable virtual_pose_publisher.cpp
add_executable(virtual_pose_publisher src/virtual_pose_publisher.cpp)
target_include_directories(virtual_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(virtual_pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  virtual_pose_publisher
  "rclcpp"
  "std_msgs"
)
install(TARGETS virtual_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# For executable ellipse_publisher.cpp
add_executable(ellipse_publisher src/ellipse_publisher.cpp)
target_include_directories(ellipse_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ellipse_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ellipse_publisher
  "rclcpp"
  "std_msgs"
)
install(TARGETS ellipse_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# For executable force_publisher.cpp
add_executable(force_publisher src/force_publisher.cpp)
target_include_directories(force_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(force_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  force_publisher
  "rclcpp"
  "std_msgs"
)
install(TARGETS force_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# For executable high_freq_pose_publisher.cpp
add_executable(high_freq_pose_publisher src/high_freq_pose_publisher.cpp)
target_include_directories(high_freq_pose_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(high_freq_pose_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  high_freq_pose_publisher
  "rclcpp"
  "std_msgs"
)
install(TARGETS high_freq_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Add launch file to share folder
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

##### adding script to lib folder #####
install(
  PROGRAMS
    scripts/xsarm_robot.py
    scripts/xsarm_ikpy_robot.py
  DESTINATION
    lib/${PROJECT_NAME}
)

##### adding custom messages #####
set(msg_files
  "msg/ConstrainedPose.msg"
  "msg/PathAndExecutionTiming.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(joy_moveit_constrained "${cpp_typesupport_target}")
target_link_libraries(joy_input_handler "${cpp_typesupport_target}")
target_link_libraries(admittance_controller "${cpp_typesupport_target}")
target_link_libraries(virtual_pose_publisher "${cpp_typesupport_target}")
target_link_libraries(ellipse_publisher "${cpp_typesupport_target}")
target_link_libraries(force_publisher "${cpp_typesupport_target}")
target_link_libraries(high_freq_pose_publisher "${cpp_typesupport_target}")

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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
