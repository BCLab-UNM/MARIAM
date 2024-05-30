import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Correctly find the package directory
    mariam_demos_share = get_package_share_directory('mariam_demos')

    # Path to the launch file you want to include
    ceiling_camera_launch_file = os.path.join(
        mariam_demos_share,
        'launch',
        'start_ceiling_v4l_camera.launch.py'
    )
        # Get the path to the YAML file using the package name
    domain_config_file_path = os.path.join(
        mariam_demos_share,
        'resource',
        'experiment_1_bridge.yml'
    )

    # Create a LaunchDescription object
    ld = LaunchDescription()

    # Include the ceiling camera launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ceiling_camera_launch_file)
        )
    )

    # Additional nodes...
    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        )
    )

    # Define the domain bridge node
    ld.add_action(
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='domain_bridge',
            output='screen',
            arguments=[domain_config_file_path],
        )
    )

    ld.add_action(
        Node(
            package='mariam_demos',
            executable='JoyToCmdVel',
            name='JoyToCmdVel'
        )
    )

    return ld
