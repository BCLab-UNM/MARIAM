# MARIAM (Multi-Agent Robust & Intelligent Autonomous Manipulation)

## Installation

Assuming a prior install of `ros-humble-desktop-full`.

### Install
Please use the following setup script for install.

```bash
cd ~
git clone --recursive git@github.com:BCLab-UNM/MARIAM.git
sudo rosdep init
rosdep update
chmod +x ~/MARIAM/script/mariam_install.sh
./MARIAM/script/mariam_install.sh
```

In addition, to use gazebo please build the `humble` branch of the `gazebo_ros2_control` soruce code independently. 
```bash
cd ~
git clone https://github.com/ros-controls/gazebo_ros2_control.git
cd gazebo_ros2_control
git checkout humble
col build
source install/setup.bash
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
Please check on swarmies at least once a semester and log battery states in `doc/Swarmie_GiAnts, Dragonfly, Laptop & Desktop Inventory.xlsx` for future reference.

"As I recall they are 4S batteries, each cell's "nominal" voltage (lowest voltage that you should run it at) is 3.8v so 15.2volts, fully charged they should be around 4.2v so fully charged you should read 16.8 volts." - C. Frost
