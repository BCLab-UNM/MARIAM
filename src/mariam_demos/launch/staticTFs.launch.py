#!/usr/bin/env python3
import launch
from launch_ros.actions import Node

def generate_launch_description():
    # Get the hostname to use as the prefix
    hostname = open("/etc/hostname", "r").read().strip()

    return launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mast_link_base_link',
            arguments=[
                '-0.2032', '0.0', '-0.4826', '0', '0', '1', '0',
                f'{hostname}/mast_link', f'{hostname}/base_link'  
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='top_link_base_link',
            arguments=[
                '0.0', '0.0', '-0.09', '0', '0', '1', '0',
                f'{hostname}/top_link', f'{hostname}/base_link'  
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_top_link',
            arguments=[
                '0.0', '0.0', '0.09', '0', '0', '-1', '0',
                f'{hostname}/base_link', f'{hostname}/top_link'  
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

