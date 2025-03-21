from launch import LaunchDescription

from launch_ros.actions import Node
from launch.conditions import IfCondition

from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments
)

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration(
        'xs_driver_logging_level')

    tracker_launch_arg = LaunchConfiguration('use_tracker')

    xsarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_control'),
                'launch',
                'xsarm_control.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'robot_name': robot_name_launch_arg,
            'base_link_frame': base_link_frame_launch_arg,
            'use_rviz': use_rviz_launch_arg,
            'mode_configs': mode_configs_launch_arg,
            'use_sim': use_sim_launch_arg,
            'robot_description': robot_description_launch_arg,
            'xs_driver_logging_level': xs_driver_logging_level_launch_arg,
        }.items(),
        condition=IfCondition(launch_driver_launch_arg)
    )

    sine_wave_publisher_node = Node(
        name='sine_wave_publisher',
        package='arm_controller',
        executable='sine_wave_publisher',
        namespace=robot_name_launch_arg,
    )

    joint_delay_tracker_node = Node(
        name='joint_delay_tracker',
        package='arm_controller',
        executable='joint_delay_tracker',
        namespace=robot_name_launch_arg,
        condition=IfCondition(
            PythonExpression([
                "'", tracker_launch_arg,
                "' == 'true'"
            ])
        )
    )

    declared_arguments = [
        # Declaring launch arguments
        DeclareLaunchArgument(
            'robot_model', default_value='px100'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model')
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=('true', 'false'),
            description='launches RViz if set to `true`.',
        ),
        DeclareLaunchArgument(
            'mode_configs',
            default_value=PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=('true', 'false'),
            description=(
                '`true` if xsarm_control should be launched - set to `false` if you would like to '
                'run your own version of this file separately.'
            ),
        ),
        DeclareLaunchArgument(
            'xs_driver_logging_level',
            default_value='INFO',
            choices=('DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'),
            description='set the logging level of the X-Series Driver.'
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            choices=('true', 'false'),
            description=(
                "if `true`, the DYNAMIXEL simulator node is run; use RViz to visualize the robot's"
                ' motion; if `false`, the real DYNAMIXEL driver node is run.'
            ),
        ),
        DeclareLaunchArgument(
            'use_tracker',
            default_value='true'
        )
    ]

    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments())

    return LaunchDescription(declared_arguments + [
        # launch sine_wave_publisher after 2 seconds have passed
        TimerAction(
            period=2.0,
            actions=[
                sine_wave_publisher_node
            ]
        ),
        xsarm_control_launch,
        joint_delay_tracker_node
    ])
