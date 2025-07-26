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
from launch_ros.actions import Node
import os

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition



def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    admittance_control_launch_arg = LaunchConfiguration(
        'use_admittance_control')
    use_sim_time = LaunchConfiguration('use_sim_time')


    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_robot_{robot_name_launch_arg.perform(context)}',
        arguments=[
            '-entity', robot_name_launch_arg.perform(context),
            # topic to read the robot description from
            '-topic', f'/{robot_name_launch_arg.perform(context)}/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            '-robot_namespace', robot_name_launch_arg.perform(context),
            # '--ros-args', '--log-level', 'DEBUG'
        ],
        output='screen',
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster'
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace=robot_name_launch_arg,
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller'
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    px100_controller_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'px100_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_admittance_control': admittance_control_launch_arg,
            'use_fake_force': 'false',
            'use_sim': use_sim_time,
            'use_rsp': 'false',
            'use_rviz': 'false',
        }.items()
    )

    mariam_description_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_rsp.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # realsense_imu_launch_desc = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mariam_vision'),
    #             'launch',
    #             'realsense_imu.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'namespace': robot_name_launch_arg,
    #     }.items()
    # )

    ekf_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'ekf.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg
        }.items()
    )

    slam_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2_bringup_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_navigation'),
                'launch',
                'nav2_custom_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return [
        spawn_node,
        px100_controller_desc,
        mariam_description_launch_desc,
        # ekf_launch_desc,
        # slam_launch_desc,
        # nav2_bringup_launch_desc,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        spawn_joint_state_broadcaster_node,
        spawn_arm_controller_node
    ]


def generate_launch_description():
    # --------------------------------------------
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
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    ]

    return LaunchDescription(declared_launch_arguments +
                             [OpaqueFunction(function=launch_setup)])
