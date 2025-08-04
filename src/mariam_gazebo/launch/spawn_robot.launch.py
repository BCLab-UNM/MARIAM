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
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    admittance_control_launch_arg = LaunchConfiguration('use_admittance_control')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz_markers_launch_arg = LaunchConfiguration('use_rviz_markers')
    spawn_location_launch_arg = LaunchConfiguration('spawn_location')

    spawn_location = spawn_location_launch_arg.perform(context).split()

    spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_robot_{robot_name_launch_arg.perform(context)}',
        arguments=[
            '-entity', robot_name_launch_arg.perform(context),
            # topic to read the robot description from
            '-topic', f'/{robot_name_launch_arg.perform(context)}/robot_description',
            '-x', spawn_location[0],
            '-y', spawn_location[1],
            '-z', spawn_location[2],
            '-R', spawn_location[3],
            '-P', spawn_location[4],
            '-Y', spawn_location[5],
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


    px100_controller_node = Node(
        package='mariam_gazebo',
        executable='px100_controller_gazebo.py',
        name='px100_controller',
        namespace=robot_name_launch_arg,
        output='screen',
        arguments=[
                # '--ros-args', '--log-level', 'DEBUG'
        ]
    )

    heading_publisher_cmd = Node(
        package='arm_controller',
        executable='heading_publisher',
        namespace=robot_name_launch_arg,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    admittance_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'admittance_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_fake_force': 'false',
            'use_rviz_markers': use_rviz_markers_launch_arg
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        )
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

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output='screen',
    )

    ekf_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'ekf.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            'config_file': os.path.join(
                get_package_share_directory('mariam_localization'),
                'config',
                'ekf_params_gazebo.yaml'
            ),
        }.items()
    )

    slam_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'rtab.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            'use_sim_time': use_sim_time
        }.items()
    )

    # slam_launch_desc = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mariam_localization'),
    #             'launch',
    #             'slam.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'namespace': robot_name_launch_arg,
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )

    # nav2_bringup_launch_desc = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mariam_navigation'),
    #             'launch',
    #             'nav2_custom_bringup.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'namespace': robot_name_launch_arg,
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # )

    return [
        spawn_node,
        px100_controller_node,
        # heading_publisher_cmd,
        admittance_control_description,
        joint_state_publisher_node,
        # ekf_launch_desc,
        slam_launch_desc,
        # nav2_bringup_launch_desc,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        mariam_description_launch_desc,

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
        ),
        DeclareLaunchArgument(
            'use_rviz_markers',
            default_value='false',
            choices=('true', 'false')
        ),
        DeclareLaunchArgument(
            'spawn_location',
            default_value='0.0 0.0 0.1 0.0 0.0 0.0',
            description='The spawn location of the robot in the Gazebo world, in x y z roll pitch yaw format.'
        )
    ]

    return LaunchDescription(declared_launch_arguments +
                             [OpaqueFunction(function=launch_setup)])
