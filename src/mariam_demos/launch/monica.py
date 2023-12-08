#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
import os

def generate_launch_description():
    # Define configurations
    robot_model = LaunchConfiguration('robot_model', default='px100')
    robot_name = LaunchConfiguration('robot_name', default='monica')
    controller = LaunchConfiguration('controller', default='xbox360')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # arm_controller launch
    arm_controller_launch = Node(
        package='arm_controller',
        executable='xsarm_joy_nojoy.launch.py',
        name='arm_controller',
        arguments=[
            'robot_model:=' + robot_model,
            'robot_name:=' + robot_name,
            'controller:=' + controller,
            'use_rviz:=' + use_rviz
        ]
    )

    # Relay nodes
    joint_group_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_group_relay',
        arguments=[
            '/monica/commands/joint_group', '/ross/commands/joint_group',
            '--ros-args', '-r', '__node:=joint_group_relay'
        ]
    )

    joint_group2_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_group2_relay',
        arguments=[
            '/monica/commands/joint_single', '/ross/commands/joint_single',
            '--ros-args', '-r', '__node:=joint_group2_relay'
        ]
    )

    # Python script execution
    python_script_path = os.path.expanduser('~/MARIAM/src/demos/DelayJoyInvert.py')
    run_python_script = ExecuteProcess(
        cmd=['python3', python_script_path]
    )

    return LaunchDescription([
        arm_controller_launch,
        joint_group_relay,
        joint_group2_relay,
        run_python_script
    ])

