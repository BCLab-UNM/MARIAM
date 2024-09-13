# MARIAM (Multi-Agent Robust & Intelligent Autonomous Manipulation)

## Installation

Assuming a prior install of `ros-humble-desktop-full`.

### Easy-Peasy-Lemon-Squeezy Install (BAD)
TODO: Fix this, now that forked submodules are being used.

```bash
cd ~
git clone --recursive git@github.com:BCLab-UNM/MARIAM.git
sudo rosdep init
rosdep update
chmod +x ~/MARIAM/scripts/mariam_install.sh
./MARIAM/scripts/mariam_install.sh
```

### Manual Install
Clone repo:
```bash
cd ~
git clone --recursive git@github.com:BCLab-UNM/MARIAM.git
```

Install packages:
```bash
sudo apt-get update && sudo apt -y upgrade
sudo apt -y autoremove
sudo apt install python3-serial ros-humble-realsense2-camera ros-humble-dynamixel-sdk ros-humble-ros2-control ros-humble-ros2-control-test-assets ros-humble-graph-msgs ros-humble-rviz-visual-tools ros-humble-hardware-interface ros-humble-moveit ros-humble-tf-transformations ros-humble-joint-trajectory-controller python3-rosdep python3-colcon-common-extensions python3-colcon-clean ros-humble-apriltag ros-humble-moveit-visual-tools python3-pip ansible
sudo pip3 install transforms3d modern_robotics
```

Setup files:
```bash
cd ~/MARIAM/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
cd ~/MARIAM/src
rm                                                                                                  \
    interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/COLCON_IGNORE      \
    interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface_msgs/COLCON_IGNORE \
    interbotix_ros_toolboxes/interbotix_rpi_toolbox/COLCON_IGNORE
```

Build and source:
```bash
cd ~/MARIAM
colcon build
source ~/MARIAM/install/setup.bash
echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
```

## Submodule Updates
All submodules that we need to make changes to have been forked by the BCLab-UNM group. If you find any other submodules that you need to edit that have not been forked yet, feel free to do so. Pulling and pushing to submodules is somewhat nuanced, so please follow these instructions carefully...

### Pulling
Here are the steps for **pulling** changes that others have made to submodules:
```bash
cd ~/MARIAM
git submodule init
git submodule update --init --recursive
cd <path/to/submodule>
git fetch origin
git pull origin <branch>
```

### Editing Submodules & Pushing
Here are the steps for editing and **pushing** your changes to a submodules. First checkout the correct branch:
```bash
cd <path/to/submodule>
git checkout <branch>
```
Next, make your changes and push:
```bash
git add .
git commit -m "<Your commit message>"
git push origin <branch>
cd ~/MARIAM
git add <path/to/submodule>
git commit -m "Update submodule to the latest commit"
git push origin main
```

## Experimentation
All experiments are located in `doc/experiment_descriptions.md`.

## Robot Maintenance
Please check on swarmies at least once a semester and log battery states in https://docs.google.com/spreadsheets/d/1_3pMGXdnTmHs9H_7EAtQgZp_9tN-sKu-xCbLE7cxGMk/edit?gid=0#gid=0 for future reference.
