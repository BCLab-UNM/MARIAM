from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Dynamically locate the path to the config file
    config_path = os.path.join(
        get_package_share_directory('mariam_navigation'),
        'config',
        'ekf_testing.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path],
        )
    ])
