#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/swarmie/microros_ws/install/setup.bash
#DO NOT restart this if you do you will have to unplug the teensy
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

