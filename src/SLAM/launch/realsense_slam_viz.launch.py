# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True}]
    
    remappings=[
            ('imu', '/imu/data'),
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/realigned_depth_to_color/image_raw'),
            ('odom','odom_slam')]

    return LaunchDescription([
        # Nodes to launch    
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='comp_viz', output='screen',
            parameters=parameters),
#            remappings=remappings)
    ])
