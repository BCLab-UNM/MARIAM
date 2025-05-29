#!/bin/bash

# This script is used to set up and start an individual robot
# with the necessary ROS 2 environment and configurations. This
# is necessary because the software for the robot is managed by
# systemd, which makes starting and shutting down the robot
# easier.

# set up the environment for the systemd service
source /opt/ros/humble/setup.bash
source /home/swarmie/MARIAM/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/microros_ws/install/setup.bash

exec ros2 launch mariam_experiments start_robot.launch.py robot_name:=$1 use_admittance_control:=$2
