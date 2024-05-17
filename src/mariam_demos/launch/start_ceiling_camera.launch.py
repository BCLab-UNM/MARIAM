#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            namespace='ceiling_camera',
            parameters=[{
                'camera_name': 'ceiling_camera',
                'device': '/dev/video_ceiling',
                'frame_id':  "ceiling_camera",
                'video_device': '/dev/video0',
                'image_width': 800,
                'image_height': 448,
                'frame_rate': 30,
                'pixel_format': 'yuyv',
                'io_method': 'mmap',
                'camera_info_url': 'package://mariam_demos/resource/ceiling_camera_y.yaml',
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
            #namespace='ceiling_camera', 
            parameters=['/home/calvinjs/MARIAM/src/mariam_demos/resource/apriltag.yaml'],
            remappings=[
                ('image_rect', '/ceiling_camera/image_raw'),
                ('camera_info', '/ceiling_camera/camera_info')
            ],
            name='ceiling_camera_apriltag_node',
            output='screen'
        )
    ])

