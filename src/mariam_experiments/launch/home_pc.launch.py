from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get the path to the domain_bridge config file
    domain_bridge_config = PathJoinSubstitution([
        FindPackageShare('mariam_experiments'),
        'config',
        'domain_bridge.yaml'
    ])
    
    return LaunchDescription([
        # Domain bridge node
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            arguments=[domain_bridge_config],
            output='screen'
        ),
        
        # VICOM pose publisher node
        Node(
            package='mariam_localization',
            executable='vicom_pose_publisher',
            output='screen'
        ),
        
        # Motion capture tracking launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('motion_capture_tracking'),
                '/launch/launch.py'
            ])
        ),
    ])