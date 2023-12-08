#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # First joy_node with remapping
    joy_node_drive = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_drive',
        remappings=[('joy', 'joy_drive')],
        output='screen'
    )

    # Second joy_node with remapping
    joy_node_monica = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_monica',
        remappings=[('joy', '/monica/commands/joy_raw')],
        output='screen'
    )

    return LaunchDescription([
        joy_node_drive,
        joy_node_monica
    ])

