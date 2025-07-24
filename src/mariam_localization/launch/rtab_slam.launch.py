import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='', 
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('mariam_localization'), 
            'config', 
            'rtab_slam_params.yaml'),
        description='Full path to the RTAB SLAM parameters file')

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
        # ('imu', 'imu/data/null'),
        ('imu', 'imu/data'),
        ('rgb/image', 'camera/color/image_raw'),
        ('rgb/camera_info', 'camera/color/camera_info'),
        ('depth/image', 'camera/aligned_depth_to_color/image_raw'),
        ('odom', 'odometry/filtered'),
        # ('odom', 'wheel/odom'),

    ]

    ### Nodes ###

    # rtab_odom_node = Node(
    #     namespace=namespace,
    #     package='rtabmap_odom',
    #     executable='rgbd_odometry',
    #     output='screen',
    #     parameters=parameters,
    #     remappings=remappings
    # )

    rtab_slam_node = Node(
        namespace=namespace,
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[slam_params_file],
        remappings=remappings,
        arguments=['-d']
    )



    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add nodes
    # ld.add_action(rtab_odom_node)
    ld.add_action(rtab_slam_node)

    return ld