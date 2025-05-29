#!/bin/bash

# This script is used to set up and start an individual robot.
#
# The ros2 workspace and environment variables are sourced,
# because the SSH connections do not use a pseudo-terminal, 
# and thus do not source the .bashrc file.

if [ "$1" == "kill" ]; then
  echo "Killing all processes"
  # first, interrupt the processes gracefully
  pkill -2 -f "ros2 launch mariam_experiments"

  # kill any ghost processes
  pkill -9 -f "mariam_experiments"
  pkill -9 -f "/home/swarmie/MARIAM/*"
  pkill -9 -f "realsense2_camera"
  pkill -9 -f "robot_state_publisher"
  pkill -9 -f "micro_ros_agent"

else
  source /opt/ros/humble/setup.bash
  source /home/swarmie/MARIAM/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  source ~/microros_ws/install/setup.bash

  exec ros2 launch mariam_experiments start_robot.launch.py robot_name:=$1 use_admittance_control:=$2
fi




