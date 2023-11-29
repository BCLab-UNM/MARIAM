cmake_minimum_required(VERSION 3.5)
project(bridge)

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
find_package(bridge_msg REQUIRED)
include_directories(include)

add_executable(abridge src/abridge.cpp src/usbSerial.cpp src/pid.cpp)

set_property(TARGET abridge PROPERTY ENABLE_EXPORTS 1)
ament_target_dependencies(abridge
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
  bridge_msg
  )

#install(TARGETS bridge
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS 
  abridge
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)
#ament_export_dependencies(rosidl_default_runtime)
#ament_export_dependencies(std_msgs)
ament_export_dependencies(bridge)
ament_package()

