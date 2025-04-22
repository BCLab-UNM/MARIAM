from pathlib import Path

from interbotix_xs_modules.xs_launch.xs_launch import determine_use_sim_time_param
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
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit



def launch_setup(context, *args, **kwargs):
    #### Launch configurations

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


    #### Setting GZ environment variables
    # Set gazebo resource path
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

    #### Nodes and launch descriptions
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

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_mariam',
        arguments=[
            '-entity', 'mariam',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            # '--ros-args', '--log-level', 'DEBUG'
        ],
        output='screen',
    )

    #### PX100 control nodes
    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster'
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller'
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    #### mariam description launch
    mariam_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_description.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': use_rviz_launch_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rqt_robot_steering_node = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
    )

    # Start Depth to LaserScan Node
    # use 'LIBGL_ALWAYS_SOFTWARE=1 rviz2' if crashes
    start_depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[
            {'scan_time': 0.033},
            {'range_min': 0.45},
            {'range_max': 100.0},
            {'scan_height': 1},
            {'output_frame': 'camera_link'},
        ],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ]
    )

    return [
        gz_resource_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,

        spawn_robot_node,
        rqt_robot_steering_node,
        start_depth_to_laserscan_node,

        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        mariam_description_launch_include
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
