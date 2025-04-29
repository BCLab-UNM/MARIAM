cmake_minimum_required(VERSION 3.8)
project(mariam_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


#### Add libraries
add_library(force_sensor SHARED src/force_sensor.cc)
target_link_libraries(force_sensor ${GAZEBO_LIBRARIES})
ament_target_dependencies(
  force_sensor
  "rclcpp"
  "std_msgs"
)
install(
  TARGETS force_sensor
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders
install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

##### adding script to lib folder #####
install(
  PROGRAMS
    scripts/px100_controller_gazebo.py
  DESTINATION
    lib/${PROJECT_NAME}
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
