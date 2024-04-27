#!/usr/bin/env bash
# can start up like 
#`ssh swarmie@monica.local  /home/swarmie/MARIAM/misc/start_robot.sh`

source /opt/ros/humble/setup.bash

if [ ! -d /home/swarmie/MARIAM/install/interbotix_ros_xsarms ]; then
  echo "Missing interbotix_ros_xsarms build."
  echo "Pulling and building everything"
  cd /home/swarmie/MARIAM && git pull
  cd /home/swarmie/MARIAM && colcon build --symlink-install
fi

# If you give any arguments it will pull and build the mariam_demos arm_controller packages
if [ $1 ]
then
    cd /home/swarmie/MARIAM && git pull
    cd /home/swarmie/MARIAM && colcon build --symlink-install --packages-select mariam_demos arm_controller
fi
#@NOTE might need to run `colcon clean workspace -y ` if people are not building with symlinks
# Also assuming that all the  interbotix packages are alredy built

# Source workspace
source /home/swarmie/MARIAM/install/setup.bash

# Get the current hostname
hostname=$(hostname)

# Launch the arm controller
ros2 launch interbotix_xsarm_joy xsarm_joy.launch.py robot_model:=px100 controller:=xbox360 robot_name:=$hostname

# Will add staticTF in post-processing
# source /home/swarmie/MARIAM/install/setup.bash
# ros2 launch mariam_demos staticTFs.launch.py &

#@TODO start onboard cameras here

# ROS Domains only needed for
# if [ $HOSTNAME == "monica" ]
# then
#     export ROS_DOMAIN_ID=1
# else
#     export ROS_DOMAIN_ID=2
# fi
# ros2 launch arm_controller xsarm_moveit_joy.launch.py robot_model:=px100 hardware_type:=actual controller:=xbox360 use_joy:=false