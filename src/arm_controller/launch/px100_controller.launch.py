from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
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
    launch_driver_launch_arg = LaunchConfiguration('launch_driver')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    robot_description_launch_arg = LaunchConfiguration('robot_description')
    xs_driver_logging_level_launch_arg = LaunchConfiguration('xs_driver_logging_level')

    admittance_control_launch_arg = LaunchConfiguration('use_admittance_control')
    force_node_launch_arg = LaunchConfiguration('use_fake_force')

    #### Setting environment variable
    if robot_name_launch_arg.perform(context) == 'ross':
        domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", '42')
    elif robot_name_launch_arg.perform(context) == 'monica':
        domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", '43')
    else:
        # NOTE: 0 is the default domain ID for nodes
        domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", '0')

    #### Nodes and launch descriptions
    px100_controller_node = Node(
        name='px100_controller_node',
        package='arm_controller',
        executable='px100_controller.py',
        namespace=robot_name_launch_arg,
        output='screen', # change to 'screen' for messages
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', robot_name_launch_arg.perform(context),
        ]
    )

    ## Control node launch description
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

    
    ## Admittance Controller launch description
    admittance_control_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('arm_controller'),
                        'launch',
                        'admittance_controller.launch.py'
                    ])
                ]),
                launch_arguments={
                    'robot_name': robot_name_launch_arg,
                    'use_fake_force': force_node_launch_arg,
                    'use_rviz_markers': use_rviz_launch_arg
                }.items(),
                condition=IfCondition(
                    PythonExpression([
                        "'", admittance_control_launch_arg,
                        "' == 'true'"
                    ])
                )
            )

    return [
        domain_id,
        px100_controller_node,
        xsarm_control_launch,
        admittance_control_launch
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

        # NOTE: if use_sim == false, use_rviz == false
        DeclareLaunchArgument(
            'use_rviz',
            default_value=LaunchConfiguration('use_sim'),
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
            'use_admittance_control',
            default_value='true',
            choices=('true', 'false'),
            description="Launches an admittance controller node when true."
        ),
        DeclareLaunchArgument(
            'use_fake_force',
            default_value='false',
            choices=('true', 'false'),
            description="Launches a force publisher if use_admittance_control is also true."
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
