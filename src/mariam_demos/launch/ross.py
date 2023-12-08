#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Define configurations
    robot_model = LaunchConfiguration('robot_model', default='px100')
    robot_name = LaunchConfiguration('robot_name', default='ross')
    controller = LaunchConfiguration('controller', default='xbox360')

    # arm_controller launch
    arm_controller_launch = Node(
        package='arm_controller',
        executable='xsarm_joy_nojoy_noxsarmjoy.launch.py',
        name='arm_controller',
        arguments=[
            'robot_model:=' + robot_model,
            'robot_name:=' + robot_name,
            'controller:=' + controller
        ]
    )

    # Python script execution
    python_script_path = os.path.expanduser('~/MARIAM/src/demos/DelayJoy.py')
    run_python_script = ExecuteProcess(
        cmd=['python3', python_script_path]
    )

    return LaunchDescription([
        arm_controller_launch,
        run_python_script
    ])

