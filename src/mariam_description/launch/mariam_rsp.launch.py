import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    # Locate the package share directory.
    pkg_share = FindPackageShare(package='mariam_description').find('mariam_description')
    
    # Define the robot model and rviz config file paths.
    robot_name_in_urdf = 'mariam'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mariam_config.rviz')
    xacro_model_path = os.path.join(pkg_share, 'xacro_models/mariam.urdf.xacro')
  
    # Launch configuration variables.
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    namespace = LaunchConfiguration('namespace')
  
    # Declare the launch arguments.
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=xacro_model_path, 
        description='Absolute path to robot urdf file'
    )
  
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace for the robot'
    )
  
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher'
    )
  
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='False',
        description='Whether to start RVIZ'
    )
  
    # Robot state publisher node.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': Command(['xacro ', model])
        }]
    )
  
    # Optional RViz node.
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
  
    # Create the launch description and add actions.
    ld = LaunchDescription()
  
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
  
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
  
    return ld
