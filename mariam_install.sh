#!/usr/bin/env bash

echo -e "\n\n"
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}            Starting MARIAM installation!     ${NORM}${OFF}"
echo -e "${GRN}${BOLD}   This process may take around 15 Minutes!   ${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo -e "\n\n"

INSTALL_PATH=~/MARIAM

echo "Updating system..."
sudo apt-get update && sudo apt -y upgrade
sudo apt -y autoremove

echo "Installing packages..."
sudo apt install python3-serial ros-humble-realsense2-camera \
    ros-humble-dynamixel-sdk ros-humble-ros2-control ros-humble-ros2-control-test-assets \
    ros-humble-graph-msgs ros-humble-rviz-visual-tools ros-humble-hardware-interface \
    ros-humble-moveit ros-humble-tf-transformations ros-humble-joint-trajectory-controller \
    python3-rosdep python3-colcon-common-extensions python3-colcon-clean ros-humble-apriltag \
    ros-humble-moveit-visual-tools python3-pip
pip3 install transforms3d modern_robotics

echo "Copying UDEV rules..."
cd ~/MARIAM/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# echo "Remove COLON ignore..."
# cd ~/MARIAM/src
# rm                                                                                                  \
#     interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE      \
#     interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
#     interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE

echo "Building MARIAM workspace..."
cd ~/MARIAM
rosdep install --from-paths src --ignore-src -r -y
if colcon build; then
    echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    echo "source ~/MARIAM/install/setup.bash" >> ~/.bashrc
    source ~/MARIAM/install/setup.bash
else
    echo "Failed to build Interbotix Arm ROS Packages."
fi

# Set up Environment Variables
if [ -z "$ROS_IP" ]; then
    echo "Setting up Environment Variables..."
    echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
    echo "Environment variables already set!"
fi

echo -e "${GRN}NOTE: Remember to reboot the computer before using the robot!${OFF}"

