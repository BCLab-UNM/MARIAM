from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

# Launch instructions:
# ros2 launch mariam_slam realsense_imu.launch.py tf_prefix:=robot1 camera_namespace:=robot1


def generate_launch_description():
    # Launch arguments.
    camera_namespace = LaunchConfiguration('camera_namespace')
    tf_prefix = LaunchConfiguration('tf_prefix')
    enable_gyro = LaunchConfiguration('enable_gyro')
    enable_accel = LaunchConfiguration('enable_accel')

    # Determine the path to the rs_launch.py file.
    realsense2_package_share_directory = get_package_share_directory('realsense2_camera')
    rs_launch_file = os.path.join(realsense2_package_share_directory, 'launch', 'rs_launch.py')

    # Include rs_launch.py and pass the parameters.
    rs_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_file),
        launch_arguments={
            'camera_namespace': camera_namespace,
            'tf_prefix': tf_prefix,
            'enable_gyro': enable_gyro,
            'enable_accel': enable_accel,
        }.items()
    )

    return LaunchDescription([
        # Declare launch arguments.
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='Namespace for the camera'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Namespace for /tf and /tf_static'
        ),
        DeclareLaunchArgument(
            'enable_gyro',
            default_value='true',
            description='Enable gyro stream (passed as enable_gyro to rs_launch.py)'
        ),
        DeclareLaunchArgument(
            'enable_accel',
            default_value='true',
            description='Enable accel stream'
        ),
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='1',
            description='[0-None, 1-copy, 2-linear_interpolation]'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='false',
            description='enable depth stream'
        ),
        DeclareLaunchArgument(
            'enable_color',
            default_value='false',
            description='enable color stream'
        ),
        rs_launch_include
    ])
