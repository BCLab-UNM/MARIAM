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
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    hardware_type = LaunchConfiguration('hardware_type', default='actual')
    robot_name = LaunchConfiguration('robot_name', default='ross')

    arm_controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name' : robot_name,
            'use_rviz': use_rviz,
            'hardware_type': hardware_type,
        }.items(),
    )

    # Python script execution
    python_script_path = os.path.expanduser('~/MARIAM/src/demos/DelayJoy.py')
    run_python_script = ExecuteProcess(
        cmd=['python3', python_script_path]
    )

    return LaunchDescription([
        arm_controller_launch_include,
        run_python_script
    ])

