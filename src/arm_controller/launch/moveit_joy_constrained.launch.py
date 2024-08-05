import os

from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
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
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    ##### Parameters #####
    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    hardware_type_launch_arg = LaunchConfiguration('hardware_type')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])
    robot_description = {'robot_description': robot_description_launch_arg}
    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model_launch_arg.perform(context),
                config_path=config_path
            ),
    }

    interbotix_pkg_path = get_package_share_directory(
        'interbotix_xsarm_moveit'
    )
    interbotix_launch_path = os.path.join(
        interbotix_pkg_path,
        'launch',
        'xsarm_moveit.launch.py'
    )
    interbotix_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            interbotix_launch_path,
        ),
        launch_arguments={
            'robot_model': robot_model_launch_arg,
            'hardware_type': hardware_type_launch_arg
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    joy_input_handler_node = Node(
        package='arm_controller',
        executable='joy_input_handler',
        name='joy_input_handler',
        parameters=[
            # robot_description,
        ],
        output='screen',
    )

    joy_moveit_constrained_node = Node(
        package='arm_controller',
        executable='joy_moveit_constrained',
        name='joy_moveit_constrained',
        parameters=[
            # these are needed for the moveit group interface
            robot_description,
            robot_description_semantic,
        ],
        output='screen',
    )

    return [
        interbotix_launch_description,
        joy_node,
        joy_input_handler_node,
        joy_moveit_constrained_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model', default_value='px100'
        ),
        DeclareLaunchArgument(
            'hardware_type', default_value='fake'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model')
        )
    ]
    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual'
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
