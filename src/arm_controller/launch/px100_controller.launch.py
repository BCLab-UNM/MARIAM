from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import (
    Node,
    PushRosNamespace
)
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

    ikpy_launch_arg = LaunchConfiguration('use_ikpy')
    admittance_control_launch_arg = LaunchConfiguration('use_admittance_control')

    xsarm_robot_node = Node(
        name='xsarm_robot_node',
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
        ],
        condition=IfCondition(
            PythonExpression([
                "'",ikpy_launch_arg,
                "' == 'false'"
            ])
        )
    )

    xsarm_ikpy_robot_node = Node(
        name='xsarm_ikpy_robot_node',
        package='arm_controller',
        executable='xsarm_ikpy_robot.py',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_model': robot_model_launch_arg,
        }],
        arguments=[
            '--robot_model', robot_model_launch_arg.perform(context),
            '--robot_name', robot_name_launch_arg.perform(context),
        ],
        condition=IfCondition(
            PythonExpression([
                "'",ikpy_launch_arg,
                "' == 'true'"
            ])
        )
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


    admittance_controller_node = Node(
        name='admittance_controller_node',
        package='arm_controller',
        executable='admittance_controller',
        namespace=robot_name_launch_arg,
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        )
    )

    admittance_rviz_markers_node = Node(
        name='admittance_rviz_markers_node',
        package='arm_controller',
        executable='admittance_rviz_markers',
        namespace=robot_name_launch_arg,
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        ),
    )

    virtual_pose_node = Node(
        package='arm_controller',
        executable='virtual_pose_publisher',
        name='virtual_pose_publisher_node',
        namespace=robot_name_launch_arg,
        parameters=[{
            'delay': 0.0,
            'frequency': 0.002,
            'x_pos': 0.0,
            'y_pos': 0.25,
            'z_pos': 0.098
        }],
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        )
    )

    force_node = Node(
        package='arm_controller',
        executable='force_publisher',
        name='force_publisher_node',
        namespace=robot_name_launch_arg,
        parameters=[{
            'delay': 0.0,
            'frequency': 2.0,
            'max_ticks': 2
        }],
        condition=IfCondition(
            PythonExpression([
                "'", admittance_control_launch_arg,
                "' == 'true'"
            ])
        )
    )

    return [
        xsarm_robot_node,
        xsarm_ikpy_robot_node,
        xsarm_control_launch,
        admittance_controller_node,
        admittance_rviz_markers_node,
        virtual_pose_node,
        force_node
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
            'use_ikpy',
            default_value='false',
            choices=('true', 'false'),
            description=(
                "If true, launches a node that uses IKPy as the IK solver."
            )
        ),
        DeclareLaunchArgument(
            'use_admittance_control',
            default_value='false',
            choices=('true', 'false'),
            description="Launches an admittance controller node when true."
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
