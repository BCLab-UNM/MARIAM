from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


import os

"""
This file serves as a meta launch file for launching the appropriate things
to perform experiments.

It can be used to launch the experiment for the real robots or the simulator.

For the real robots, this launch file must be run on each robot individually.
"""


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_gazebo_launch_arg = LaunchConfiguration('use_gazebo')
    admittance_control_launch_arg = LaunchConfiguration('use_admittance_control')

    # -----------------------------------------------------
    # Stuff for the real robot
    # -----------------------------------------------------
    px100_controller_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                FindPackageShare('arm_controller'),
                'launch',
                'px100_controller.launch.py'
            )
        ),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_admittance_control': admittance_control_launch_arg,
            'use_fake_force': 'false',
            'use_sim': 'false'
        },
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_launch_arg,
                "' == 'false'"
            ])
        )
    )

    micro_ros_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                FindPackageShare('mariam_microros'),
                'launch',
                'micro_ros_agent_launch.py'
            )
        ),
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_launch_arg,
                "' == 'false'"
            ])
        )
    )

    mariam_description_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_rsp.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            # TODO: does this need to be set???
            # 'use_sim_time': 'false',
        },
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_launch_arg,
                "' == 'false'"
            ])
        )
    )
    
    realsense_imu_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('mariam_vision'),
                'launch',
                'realsense_imu.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        },
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_launch_arg,
                "' == 'false'"
            ])
        )
    )
    
    hardware_ekf_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('mariam_navigation'),
                'launch',
                'hardware_ekf.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        },
        condition=IfCondition(
            PythonExpression([
                "'", use_gazebo_launch_arg,
                "' == 'false'"
            ])
        )
    )

    # TODO: add additional nodes for the real robot here

    return [
        px100_controller_desc,
        micro_ros_desc,
        mariam_description_launch_desc,
        realsense_imu_launch_desc,
        hardware_ekf_launch_desc
    ]


def generate_launch_description():
    #--------------------------------------------
    # Declaring launch arguments
    # --------------------------------------------
    declared_launch_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            description='The name of the robot (ross or monica).',
        ),
        DeclareLaunchArgument(
            'use_admittance_control',
            default_value='true',
            choices=('true', 'false'),
            description='Whether to use admittance control'
        )
    ]

    return LaunchDescription(declared_launch_arguments + 
        [OpaqueFunction(function=launch_setup)])
