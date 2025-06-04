import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

import xacro

# Launch instructions:
# ros2 launch mariam_description mariam_rsp.launch.py namespace:=robot1

def generate_launch_description():
    # Locate the package share directory.
    pkg_share = FindPackageShare(package='mariam_description').find('mariam_description')
    
    # Define the robot model and rviz config file paths.
    xacro_model_path = os.path.join(pkg_share, 'xacro_models/mariam_agent.urdf.xacro')
  
    # Launch configuration variables.
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
  
    # Declare the launch arguments.
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=xacro_model_path, 
        description='Absolute path to robot urdf file'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Namespace for the robot'
    )
  
  
    # Robot state publisher node.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            'robot_description': Command(['xacro ', model]),
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static','tf_static'),
            ('/joint_states', 'joint_states'),
        ]
    )
  
    # Create the launch description and add actions.
    ld = LaunchDescription()
  
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
