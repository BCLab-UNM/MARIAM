# Arm Controller Package

## Overview
Nodes for controlling the joints and end effector of the arm

## Run
Launch file is a modified copy of the one made in the 'interbotix_xsarm_moveit' package to include custom nodes

```bash
ros2 launch arm_controller xsarm_moveit.launch.py robot_model:=px100 hardware_type:=fake
```

Hardware types include: 'actual' 'gz_classic' 'fake'