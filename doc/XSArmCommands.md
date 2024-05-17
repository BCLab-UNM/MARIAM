# PX100 Arm Commands & Tools

A record of all launch files, commands, and tools that are relevant to working on the px100.

## Launch Files
### Basic Controller
```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

### Joystick Controller
```bash
ros2 launch interbotix_xsarm_joy xsarm_joy.launch.py robot_model:=px100
```

### MoveIt Controller
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py robot_model:=px100 hardware_type:=actual
```


## Services, Actions, Publishing
### Unlock Arm
```bash
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```

### Lock Arm
```bash
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

### Reset Overloaded Servos
- string cmd_type: set to 'group' if commanding a joint group or 'single' if commanding a single joint
- string name: name of the group if commanding a joint group or joint if commanding a single joint
- bool enable: hether to torque the selected joints on after reboot
- bool smart_reboot: set to true to only reboot motors in a specified group that are in an error state
```bash
ros2 service call /px100/reboot_motors interbotix_xs_msgs/srv/Reboot "cmd_type: 'group'
name: 'all'
enable: true
smart_reboot: true"
```

### Send Joint Group Command
```bash
ros2 topic pub /ross_arm/commands/joint_group interbotix_xs_msgs/msg/JointGroupCommand "{name: 'all', cmd: [0,-0.9,0,-0.7,0]}"
```

```bash
```
