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
    #### Launch configurations
    robot_name_launch_arg = LaunchConfiguration('robot_name')
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

    #######################################
    # SPAWNING ROSS
    #######################################

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_ross',
        arguments=[
            '-entity', 'ross',
            # topic to read the robot description from
            '-topic', '/ross/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',
            # '--ros-args', '--log-level', 'DEBUG'
        ],
        output='screen',
    )


    px100_controller_node = Node(
        package='mariam_gazebo',
        executable='px100_controller_gazebo.py',
        name='px100_controller',
        namespace='ross',
        output='screen',
        arguments=[
            # '--ros-args', '--log-level', 'DEBUG'
        ]
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='ross',
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
        namespace='ross',
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

    #### mariam description launch for ross
    mariam_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'ross',
            'use_rviz': use_rviz_launch_arg,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # TODO: Admittance control launch for ross
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
            'use_fake_force': use_fake_force_launch_arg,
            'use_rviz_markers': use_rviz_markers_launch_arg
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        )
    )

    # TODO: laser scan node for ross
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
        start_depth_to_laserscan_node,

        px100_controller_node,
        load_joint_state_broadcaster_event,
        load_arm_controller_event,
        mariam_description_launch_include,
        # admittance_control_description
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='mariam'
        )
    )
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
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_force',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "launches a node to publish fake force readings"
            )
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
