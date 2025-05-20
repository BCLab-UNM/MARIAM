from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    #### Creating launch configurations
    # path to the model
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    model_launch_arg = LaunchConfiguration('model')

    # RViz launch configurations
    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_config_launch_arg = LaunchConfiguration('rviz_config_file')
    
    # navigation and localization
    # localization_launch_arg = LaunchConfiguration('localization')
    
    # use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_pub_launch_arg = LaunchConfiguration('use_joint_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')


    #### Nodes and Launch Descriptions
    # NOTE: the robot state publisher cannot be launched under a namespace.
    # It will never publish the robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro', ' ', model_launch_arg, ' ',
                    'robot_name:=', robot_name_launch_arg
                ]),
                value_type=str),
            'use_sim_time': use_sim_time,
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output='screen',
    )

    joint_state_publisher_node = Node(
        condition=IfCondition(use_joint_pub_launch_arg),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=robot_name_launch_arg,
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        output='screen',
    )

    rviz2_node = Node(
        condition=IfCondition(use_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_launch_arg,
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node
    ]



def generate_launch_description():
    mariam_share_pkg = FindPackageShare('mariam_description')

    #### Declaring launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                mariam_share_pkg,
                'xacro_models',
                'mariam.urdf.xacro'
            ])
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
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                mariam_share_pkg,
                'rviz',
                'mariam_config.rviz',
            ]),
            description='file path to the config file RViz should load.',
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
            'use_joint_pub',
            default_value='false',
            choices=('true', 'false'),
            description='launches the joint_state_publisher node.',
        )
    )

    return LaunchDescription(
        declared_arguments 
        + [OpaqueFunction(function=launch_setup)])
