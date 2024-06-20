from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('arm_controller')
    launch_dir = os.path.join(pkg_dir, 'launch', 'moveit_joy.launch.py')

    # parameters
    robot_model = LaunchConfiguration('robot_model', default='px100')
    hardware_type = LaunchConfiguration('hardware_type', default='fake')
    use_joy = LaunchConfiguration('use_joy', default='true')

    moveit_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_dir),
        launch_arguments={
            'robot_model': robot_model,
            'hardware_type': hardware_type,
            'use_joy': use_joy
        }.items()
    )

    return LaunchDescription([
        moveit_joy_launch,
        Node(
            package='arm_controller',
            # namespace='joy_moveit_constrained',
            executable='joy_moveit_constrained',
            name='joy_moveit_constrained'
        )
    ])