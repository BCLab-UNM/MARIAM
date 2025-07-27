from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction
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


""" 
NOTE: In this launch file, we create two different instances of the
robot_state_publisher; one for ross and one for monica.

The reason is we want to create two separate instances of the
ros2_gazebo_control node, and this can only be done with two separate
instances of mariam_description.launch.py.

"""


def launch_setup(context, *args, **kwargs):
    ######################################
    # Launch configurations
    ######################################
    # RViz launch configurations
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')

    # navigation and localization
    # localization_launch_arg = LaunchConfiguration('localization')

    # use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # GZ launch configurations
    verbose_launch_arg = LaunchConfiguration('verbose')
    world_file_path_launch_arg = LaunchConfiguration('world_file_path')
    paused_launch_arg = LaunchConfiguration('paused')
    
    admittance_control_launch_arg = LaunchConfiguration('use_admittance_control')
    use_rviz_markers_launch_arg = LaunchConfiguration('use_rviz_markers')
    use_fake_force_launch_arg = LaunchConfiguration('use_fake_force')


    # Setting GZ environment variables. Prevents Gazebo from 
    # downloading unnecessary models, and adds all necessary model paths
    # to the appropriate environment variables.
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            '/usr/share/gazebo-11/models/',
            ':',
            str(Path(
                FindPackageShare('interbotix_common_sim').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare(
                    'interbotix_xsarm_descriptions').perform(context)
            ).parent.resolve()),
            ':',
            str(Path(
                FindPackageShare(
                    'mariam_description').perform(context)
            ).parent.resolve()),
        ]
    )

    # Set GAZEBO_MODEL_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_URI',
        value=['']
    )

    # Set GAZEBO_MODEL_DATABASE_URI to empty string to prevent Gazebo from downloading models
    gz_model_uri_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=['']
    )

    # launches both the gzserver and gzclient
    gazebo_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
        ]),
        launch_arguments={
            'verbose': verbose_launch_arg,
            'world': world_file_path_launch_arg,
            'pause': paused_launch_arg,
        }.items()
    )

    #######################################
    # SPAWNING ROSS
    #######################################
    spawn_ross_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_gazebo'),
                'launch',
                'spawn_robot.launch.py'
            ]),
        ]),
        launch_arguments={
            'robot_name': 'ross',
            'spawn_location': '0.0 0.0 0.1 0.0 0.0 0.0',
            'use_sim_time': use_sim_time
        }.items()
    )

    
    #######################################
    # SPAWNING MONICA
    #######################################
    spawn_monica_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_gazebo'),
                'launch',
                'spawn_robot.launch.py'
            ]),
        ]),
        launch_arguments={
            'robot_name': 'monica',
            'spawn_location': '1.5 0.0 0.1 0.0 0.0 3.14159',
            'use_sim_time': use_sim_time
        }.items()
    )


    return [
        gz_resource_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,
        spawn_ross_launch_include,
        TimerAction(
            period=5.0,
            actions=[spawn_monica_launch_include]
        )
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_file_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('mariam_gazebo'),
                'worlds',
                'shapes.world',
            ]),
            description="the file path to the Gazebo 'world' file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            choices=('true', 'false'),
            description='launches Gazebo with verbose console logging if `true`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'paused',
            default_value='false',
            choices=('true', 'false'),
            description='start Gazebo in a paused state.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock.'
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_admittance_control',
            default_value='true',
            choices=('true', 'false'),
            description=(
                "Launches nodes for admittance control."
            )
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz_markers',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "Displays an RViz marker for the position computed by the admittance controller."
            )
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
