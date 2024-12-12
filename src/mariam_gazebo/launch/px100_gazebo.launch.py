import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the URDF file
    urdf_file = os.path.join(
        get_package_share_directory('interbotix_xsarm_descriptions'),
        'urdf/px100.urdf.xacro'
    )

    # Ensure the URDF file exists
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")

    # Define the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file,
            'use_sim_time': "true",
        }],
        namespace="px100",
        output={'both': 'log'},
    )

    # Define the spawn_entity node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'px100_modified',  # Name of the entity
            '-file', urdf_file,        # Path to the URDF file
            '-x', '0', '-y', '0', '-z', '1'  # Initial position (optional)
        ],
        output='screen'
    )

    # Launch description with Gazebo and spawn_entity node
    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
