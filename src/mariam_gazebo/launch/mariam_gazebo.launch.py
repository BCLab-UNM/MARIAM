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


""" 
NOTE: In this launch file, we create two different instances of the
robot_state_publisher; one for ross and one for monica.

The reason is we want to create two separate instances of the
ros2_gazebo_control node, and this can only be done with two separate
instances of mariam_description.launch.py.

"""


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
    spawn_ross_node = Node(
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

    ross_px100_controller_node = Node(
        package='mariam_gazebo',
        executable='px100_controller_gazebo.py',
        name='px100_controller',
        namespace='ross',
        output='screen',
        arguments=[
            # '--ros-args', '--log-level', 'DEBUG'
        ]
    )

    ross_spawn_joint_state_broadcaster_node = Node(
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

    ross_spawn_arm_controller_node = Node(
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

    ross_load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_ross_node,
            on_exit=[ross_spawn_joint_state_broadcaster_node]
        )
    )

    ross_load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ross_spawn_joint_state_broadcaster_node,
            on_exit=[ross_spawn_arm_controller_node]
        )
    )

    # This launches the robot description for Ross
    ross_description_launch_include = IncludeLaunchDescription(
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

    ross_admittance_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'admittance_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'ross',
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

    # Start Depth to LaserScan Node
    # use 'LIBGL_ALWAYS_SOFTWARE=1 rviz2' if crashes
    ross_start_depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        namespace='ross',
        output='screen',
        parameters=[
            {'scan_time': 0.033},
            {'range_min': 0.45},
            {'range_max': 100.0},
            {'scan_height': 1},
            {'output_frame': 'camera_link'},
        ],
        remappings=[
            ('depth', '/ross/camera/depth/image_raw'),
            ('depth_camera_info', '/ross/camera/depth/camera_info'),
            ('scan', '/ross/scan'),
        ]
    )

    #######################################
    # SPAWNING MONICA
    #######################################
    spawn_monica_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_ross',
        arguments=[
            '-entity', 'monica',
            # topic to read the robot description from
            '-topic', '/monica/robot_description',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.1',
            # '--ros-args', '--log-level', 'DEBUG'
        ],
        output='screen',
    )

    monica_px100_controller_node = Node(
        package='mariam_gazebo',
        executable='px100_controller_gazebo.py',
        name='px100_controller',
        namespace='monica',
        output='screen',
        arguments=[
            # '--ros-args', '--log-level', 'DEBUG'
        ]
    )

    monica_spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='monica',
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

    monica_spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='monica',
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

    monica_load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_monica_node,
            on_exit=[monica_spawn_joint_state_broadcaster_node]
        )
    )

    monica_load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=monica_spawn_joint_state_broadcaster_node,
            on_exit=[monica_spawn_arm_controller_node]
        )
    )

    # This launches the robot description for Ross
    monica_description_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'monica',
            # TODO: for monica, just don't launch rviz.
            # In the future, maybe we should only launch RViz here 
            'use_rviz': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    monica_admittance_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'admittance_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': 'monica',
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

    # Start Depth to LaserScan Node
    # use 'LIBGL_ALWAYS_SOFTWARE=1 rviz2' if crashes
    monica_start_depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        namespace='monica',
        output='screen',
        parameters=[
            {'scan_time': 0.033},
            {'range_min': 0.45},
            {'range_max': 100.0},
            {'scan_height': 1},
            {'output_frame': 'camera_link'},
        ],
        remappings=[
            ('depth', '/monica/camera/depth/image_raw'),
            ('depth_camera_info', '/monica/camera/depth/camera_info'),
            ('scan', '/monica/scan'),
        ]
    )

    return [
        gz_resource_path_env_var,
        gz_model_uri_env_var,
        gazebo_launch_include,

        # nodes for ross
        spawn_ross_node,
        ross_start_depth_to_laserscan_node,

        ross_px100_controller_node,
        ross_load_joint_state_broadcaster_event,
        ross_load_arm_controller_event,
        ross_description_launch_include,
        ross_admittance_control_description,

        # nodes for monica
        spawn_monica_node,
        monica_start_depth_to_laserscan_node,

        monica_px100_controller_node,
        monica_load_joint_state_broadcaster_event,
        monica_load_arm_controller_event,
        monica_description_launch_include,
        monica_admittance_control_description
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
