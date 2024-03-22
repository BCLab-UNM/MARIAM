import launch
from launch_ros.actions import Node

def generate_launch_description():
    # Get the hostname to use as the prefix
    hostname = open("/etc/hostname", "r").read().strip()

    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_mast_link',
            arguments=[
                '-0.2032', '0.0', '0.4826', '0', '0', '0', '1',
                f'{hostname}/base_link', f'{hostname}/mast_link'  
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_arm_link',
            arguments=[
                '0.1778', '0.0', '0.0762', '0', '0', '0', '1',
                f'{hostname}/base_link', f'{hostname}/arm_link' 
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_link',
            arguments=[
                '-0.1397', '0.0', '0.4445', '0', '0', '0', '1',
                f'{hostname}/base_link', f'{hostname}/camera_link'  
            ],
        ),
    ])

