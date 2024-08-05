from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    base_link_frame_launch_arg = LaunchConfiguration('base_link_frame')
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    mode_configs_launch_arg = LaunchConfiguration('mode_configs')
    threshold_launch_arg = LaunchConfiguration('threshold')
    controller_launch_arg = LaunchConfiguration('controller')
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    experiment_launch_arg = LaunchConfiguration('experiment')

    experiment_node = Node(
        package='arm_controller',
        executable='constraint_experiment',
        name='constraint_experiment',
        output='screen',
        condition=IfCondition(experiment_launch_arg)
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=robot_name_launch_arg,
        parameters=[{
            'dev': '/dev/input/js0',
        }],
        remappings=[
            ('joy', 'commands/joy_raw')
        ]
    )

    joy_input_handler_node = Node(
        name='joy_input_handler',
        package='arm_controller',
        executable='joy_input_handler',
        namespace=robot_name_launch_arg,
        parameters=[{
            'threshold': threshold_launch_arg,
            'controller': controller_launch_arg
        }],
    )

    xsarm_robot_node = Node(
        name='xsarm_robot_node',
        package='arm_controller',
        executable='xsarm_robot.py',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', robot_name_launch_arg.perform(context),
        ],
    )

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

    return [
        experiment_node,
        joy_node,
        joy_input_handler_node,
        xsarm_robot_node,
        xsarm_control_launch
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model', default_value='px100'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model')
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.75',
            description=(
                'value from 0 to 1 defining joystick sensitivity; a larger number means the '
                'joystick should be less sensitive.'
            ),
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
                FindPackageShare('interbotix_xsarm_joy'),
                'config',
                'modes.yaml',
            ]),
            description="the file path to the 'mode config' YAML file.",
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='xbox360',
            choices=('ps4', 'ps3', 'xbox360'),
            description='type of controller.',

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
            'experiment',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "If `true`, runs the constraint_experiment node."
            )
        )
    ]
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            # show_gripper_bar='false',
            # show_gripper_fingers='false',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
