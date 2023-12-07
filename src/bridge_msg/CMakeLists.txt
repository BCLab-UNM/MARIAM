cmake_minimum_required(VERSION 3.5)
project(bridge_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidWheel.msg"
  "msg/PidState.msg"
  "msg/MoveRequest.msg"
  "msg/MoveResult.msg"
  DEPENDENCIES std_msgs
 )
 
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)
ament_package()

