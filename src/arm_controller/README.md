# Arm Controller Package
Different modes of control for the end effector of the PX100 robotic arm.

## Table of commands for xsarm_joy.launch.py
Documentation: https://wiki.ros.org/joy#Microsoft_Xbox_360_Wired_Controller_for_Linux

Note: the buttons on the controller fall under two categories: buttons and axes. The following table documents which category each button falls under and the action it will perform when pressed.

**Buttons**
| Value | Button       | Action |
|-------|--------------|--------|
|   0   | A            |        |
|   1   | B            |        |
|   2   | X            |        |
|   3   | Y            |        |
|   4   | LB           |        |
|   5   | RB           |        |
|   6   | back         |        |
|   7   | start        |        |
|   8   | power        |        |
|   9   | LS (pressed) |        |
|  10   | RS (pressed) |        |

**Axes**
| Value | Button                 | Action |
|-------|------------------------|--------|
|   0   | LS (left/right)        |        |
|   1   | LS (up/down)           |        |
|   2   | LT                     |        |
|   3   | RS (left/right)        |        |
|   4   | RS (up/down)           |        |
|   5   | RT                     |        |
|   6   | Cross key (left/right) |        |
|   7   | Cross key (up/down)    |        |

## Launch Files

### moveit_joy_constrained.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_joy_constrained.launch.py
```

Or:
```bash
ros2 launch arm_controller moveit_joy_constrained.launch.py hardware_type:=actual
```

### moveit_joy.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_joy.launch.py \
  robot_model:=px100 \
  hardware_type:=fake
  use_joy:=true
```

### xsarm_moveit_listener.launch.py
Launch with:

```bash
ros2 launch arm_controller moveit_listener.launch.py \
  robot_model:=px100 \
  hardware_type:=fake
```

### Parameters
Each launch file will use some or all of these parameters, here are their options.
- hardware_type: actual, gz_classic, fake
- robot_model: px100
- controller:= xbox360
- use_rviz:= true, false
- use_joy:= true, false
