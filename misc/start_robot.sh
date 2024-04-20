cd /home/swarmie/MARIAM && git pull

source /opt/ros/humble/setup.bash 
colcon build --symlink-install --packages-select mariam_demos arm_controller
source /home/swarmie/microros_ws/install/setup.bash
source /home/swarmie/MARIAM/install/setup.bash

touch /tmp/robot_running_$BASHPID

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 &

ros2 launch mariam_demos staticTFs.launch.py &

if [ $HOSTNAME == "monica" ]
then
    export ROS_DOMAIN_ID=1
else
    export ROS_DOMAIN_ID=2
fi
ros2 launch arm_controller xsarm_moveit_joy.launch.py robot_model:=px100 hardware_type:=actual controller:=xbox360 use_joy:=false
