from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

# Launch instructions:
# ros2 launch mariam_vision realsense_imu.launch.py tf_prefix:=robot1 camera_namespace:=robot1


def generate_launch_description():
    # Launch arguments.
    namespace = LaunchConfiguration('namespace')
    enable_gyro = LaunchConfiguration('enable_gyro')
    enable_accel = LaunchConfiguration('enable_accel')
    unite_imu_method = LaunchConfiguration('unite_imu_method')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_color = LaunchConfiguration('enable_color')

    # Determine the path to the rs_launch.py file.
    realsense2_package_share_directory = get_package_share_directory('mariam_vision')
    rs_launch_file = os.path.join(realsense2_package_share_directory, 'launch', 'rs_launch_modified.py')

    # Include rs_launch.py and pass the parameters.
    rs_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_file),
        launch_arguments={
            'camera_namespace': namespace,
            'tf_prefix': namespace,
            'enable_gyro': enable_gyro,
            'enable_accel': enable_accel,
            'unite_imu_method': unite_imu_method,
            'enable_depth': enable_depth,
            'enable_color': enable_color,
        }.items()
    )

    return LaunchDescription([
        # Declare launch arguments.
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the camera and tf frames'
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
