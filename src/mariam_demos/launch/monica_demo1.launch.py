#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription, 
    ExecuteProcess )
from launch.substitutions import (
    LaunchConfiguration, 
    Command )
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution )
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Define configurations
    robot_model = LaunchConfiguration('robot_model', default='px100')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_joy = LaunchConfiguration('use_joy', default='false')
    hardware_type = LaunchConfiguration('hardware_type', default='actual')
    
    arm_controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'xsarm_moveit_joy.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'use_rviz': use_rviz,
            'hardware_type': hardware_type,
            'use_joy': use_joy
        }.items(),
    )

    # Relay nodes
    joint_group_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_group_relay',
        arguments=[
            '/px100/commands/joint_group', '/ross/commands/joint_group'
        ]
    )

    joint_group2_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_single_relay',
        arguments=[
            '/px100/commands/joint_single', '/ross/commands/joint_single',
        ]
    )

    joint_group3_relay = Node(
        package='topic_tools',
        executable='relay',
        name='joint_trajectory_relay',
        arguments=[
            '/px100/commands/joint_trajectory', '/ross/commands/joint_trajectory',
        ]
    )

    # Python script execution
    python_script_path = os.path.expanduser('~/MARIAM/src/mariam_demos/src/DelayJoyInvert.py')
    run_python_script = ExecuteProcess(
        cmd=['python3', python_script_path]
    )

    return LaunchDescription([
        arm_controller_launch_include,
        joint_group_relay,
        joint_group2_relay,
        run_python_script
    ])

