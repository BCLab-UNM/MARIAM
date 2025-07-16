import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='', 
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('mariam_localization'), 
            'config', 
            'rtab_slam_params.yaml'),
        description='Full path to the slam_toolbox parameters file')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    parameters=[{
        'frame_id':'base_footprint',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync':False,
        'wait_imu_to_init': True
    }]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/map', 'map'),
        ('/map_metadata', 'map_metadata'),
        ('imu', 'imu/data'),
        ('rgb/image', 'camera/color/image_raw/image_topics'),
        ('rgb/camera_info', 'camera/color/camera_info'),
        ('depth/image', 'camera/aligned_depth_to_color/image_raw/image_topics')
    ]

    ### Nodes ###

    rtab_odom_node = Node(
        namespace=namespace,
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=parameters,
        remappings=remappings
    )

    rtab_slam_node = Node(
        namespace=namespace,
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=parameters,
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
    ld.add_action(rtab_odom_node)
    ld.add_action(rtab_slam_node)

    return ld