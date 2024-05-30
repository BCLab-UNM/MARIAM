#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    # Get the package share directory
    package_share_directory = get_package_share_directory('apriltag_mariam')
    
    # Path to the parameter files
    apriltag_path = os.path.join(package_share_directory, 'resource', 'apriltag.yaml')
    camera_info_url = f"file://{os.path.join(package_share_directory, 'resource', 'ceiling_camera_y.yaml')}"

    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='ceiling_camera',
            parameters=[{
                'video_device': '/dev/video_ceiling',
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'image_size': [800,448],
                'camera_info_url': camera_info_url,
                'camera_frame_id': 'ceiling_camera'
            }],
            name='ceiling_camera',
            output='screen', 
            respawn=True,
            respawn_delay=1
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            namespace='ceiling_camera', 
            parameters=[apriltag_path],
            remappings=[
                ('image_rect', 'image_raw'),
                ('camera_info', 'camera_info')
            ],
            name='ceiling_camera_apriltag_node',
            output='screen'
        )
    ])

