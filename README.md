# MARIAM (Multi-Agent Robust & Intelligent Autonomous Manipulation)

[Description]

## Installation

Assuming a prior install of `ros-humble-desktop-full`.

### Easy-Peasy-Lemon-Squeezy Install

```bash
cd ~
git clone --recursive git@github.com:BCLab-UNM/MARIAM.git
./MARIAM/mariam_install.sh
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

### Submodele Updates
Some edits have been made to the submodules owned by TrossenRobotics. Easily make those edits with this script.

```bash
cd ~
ansible MARIAM/misc/playbook_submodule_edits.yaml
```

To revert edits reset each submodule to its head.
```bash
cd ~MARIAM/src/interbotix_ros_toolboxes
git reset --hard HEAD
cd ~MARIAM/src/interbotix_ros_core
git reset --hard HEAD
cd ~MARIAM/src/interbotix_ros_manipulators
git reset --hard HEAD
```
## Experimentation

### Experiment 1 Setup
No arguements are needed the defaults are already set.
On laptop: `ros2 launch arm_controller laptop_demo1.launch.py`
On Monica: `ros2 launch arm_controller monica_demo1.launch.py`
On Ross: `ros2 launch arm_controller ross_demo1.launch.py`

Perform pickup of box.
Then on laptop launch  `<move base node>`