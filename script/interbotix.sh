#!/usr/bin/env bash

echo "Removing CATKIN_IGNORE to make perception work"
rm src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/CATKIN_IGNORE \
   src/interbotix_ros_toolboxes/interbotix_perception_toolbox/CATKIN_IGNORE


cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    
    
#@TODO edit interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/px100.yaml update_rate to ~440hz
