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
sudo apt install python3-serial \
    ros-humble-dynamixel-sdk ros-humble-ros2-control ros-humble-ros2-control-test-assets \
    ros-humble-graph-msgs ros-humble-rviz-visual-tools ros-humble-hardware-interface \
    ros-humble-moveit ros-humble-tf-transformations ros-humble-joint-trajectory-controller \
    python3-rosdep python3-colcon-common-extensions python3-colcon-clean ros-humble-apriltag \
    ros-humble-moveit-visual-tools python3-pip ros-humble-usb-cam ros-humble-domain-bridge ros-humble-apriltag-ros \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-rviz-plugins \
    ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-rqt-robot-steering \
    ros-humble-rtabmap ros-humble-rtabmap-ros ros-humble-rtabmap-rviz-plugins \
    ros-humble-imu-filter-madgwick sudo apt install \
    sudo apt install ros-humble-librealsense2* \

pip3 install transforms3d modern_robotics glob

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
if colcon build --symlink-install; then
    echo -e "${GRN}${BOLD}Interbotix Arm ROS Packages built successfully!${NORM}${OFF}"
    echo "source ~/MARIAM/install/setup.bash" >> ~/.bashrc
    source ~/MARIAM/install/setup.bash
else
    echo "Failed to build Interbotix Arm ROS Packages."
    echo "Did you source your ROS 2 installation? (source /opt/ros/humble/setup.bash)"
    exit 1
fi

# Set up Environment Variables
if [ -z "$ROS_IP" ]; then
    echo "Setting up Environment Variables..."
    echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
    echo "Environment variables already set!"
fi

echo -e "\n\n${GRN}Configuration Setup for Robot Usage${OFF}"

echo -e "\n${GRN}The following commands can be added to your ~/.bashrc to ensure your environment is properly set up when a terminal is created:${OFF}"

# List of commands to source
commands=(
    "source /opt/ros/humble/setup.bash"
    "source ~/MARIAM/install/setup.bash"
    "source /usr/share/gazebo/setup.sh"
)

# Iterate over each command and ask user if they want to add it
for cmd in "${commands[@]}"; do
    echo -e "\n${GRN}Would you like to add the following command to your ~/.bashrc?${OFF}"
    echo -e "${GRN}${cmd}${OFF}"
    read -p "Enter Y to add it, or N to skip: " user_input
    if [[ "$user_input" =~ ^[Yy]([Ee][Ss])?$ ]]; then
        # Check if already exists in ~/.bashrc to avoid duplicates
        if grep -Fxq "$cmd" ~/.bashrc; then
            echo -e "${GRN}Command already exists in ~/.bashrc. Skipping...${OFF}"
        else
            echo "$cmd" >> ~/.bashrc
            echo -e "${GRN}Command added to ~/.bashrc.${OFF}"
        fi
    else
        echo -e "${GRN}Skipping this command.${OFF}"
    fi
done

# Ask if the user would like to reboot
echo -e "\n${GRN}Would you like to reboot the computer now?${OFF}"
read -p "Enter Y to reboot, or N to cancel: " reboot_input
if [[ "$reboot_input" =~ ^[Yy]([Ee][Ss])?$ ]]; then
    echo -e "${GRN}Rebooting the computer...${OFF}"
    sudo reboot
else
    echo -e "${GRN}Reboot cancelled. Please reboot manually later if necessary.${OFF}"
fi