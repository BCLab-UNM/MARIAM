#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define the path to the apriltag config file
    apriltag_config = os.path.join(get_package_share_directory('mariam_demos'), 'resource', 'apriltag.yaml')
    
    # Define the usb_cam node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
                'video_device': '/dev/video_ceiling',
                #'image_width': 1920,
                #'image_height': 1080,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                #'camera_name': 'ceiling_cam',
                #'frame_id': 'ceiling_cam',
                'camera_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': [357.357484, 0.000000, 174.600869, 0.000000, 357.899268, 131.766830, 0.000000, 0.000000, 1.000000]
                },
                'distortion_model': 'plumb_bob',
                'distortion_coefficients': {
                    'rows': 1,
                    'cols': 5,
                    'data': [0.059693, 0.055818, 0.002143, 0.010728, 0.000000]
                },
                'rectification_matrix': {
                    'rows': 3,
                    'cols': 3,
                    'data': [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
                },
                'projection_matrix': {
                    'rows': 3,
                    'cols': 4,
                    'data': [363.801727, 0.000000, 176.118069, 0.000000, 0.000000, 366.281525, 131.365056, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
                }
            }],
        remappings=[
            ('/image_raw', '/image_rect'),
            ('/camera_info', '/camera_info')
        ]
    )

    # Define the first apriltag_ros node
    apriltag_node_1 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltagRealSence',
        remappings=[
            ('camera_info', '/camera/color/camera_info'),
            ('image_rect', '/camera/color/image_raw')
        ],
        parameters=[apriltag_config]
    )

    # Define the second apriltag_ros node
    apriltag_node_2 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        parameters=[apriltag_config]
    )

    # Define the joy node
    joy_node = Node(
        package='joy',
        executable='joy_node'
    )

    # Define the path to the realsense2_camera launch file
    realsense_launch_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    # Include the realsense2_camera launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )


    return LaunchDescription([
        realsense_launch,
        usb_cam_node,
        apriltag_node_1,
        apriltag_node_2,
        joy_node
    ])

