from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os


def launch_setup(context, *args, **kwargs):
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    admittance_control_launch_arg = LaunchConfiguration(
        'use_admittance_control')

    # Get the robot name and set ROS_DOMAIN_ID based on namespace
    robot_name = robot_name_launch_arg.perform(context)
    if robot_name.lower() == 'monica':
        os.environ['ROS_DOMAIN_ID'] = '1'
    elif robot_name.lower() == 'ross':
        os.environ['ROS_DOMAIN_ID'] = '2'
    else:
        # Optional: set a default domain or raise an error
        print(
            f"Warning: Unknown robot name '{robot_name}', using default domain")
        os.environ['ROS_DOMAIN_ID'] = '0'

    px100_controller_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('arm_controller'),
                'launch',
                'px100_controller.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name_launch_arg,
            'use_admittance_control': admittance_control_launch_arg,
            'use_fake_force': 'false',
            'use_sim': 'false',
            'use_rsp': 'false',
        }.items()
    )

    micro_ros_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_microros'),
                'launch',
                'micro_ros_agent_launch.py'
            ])
        ])
    )

    mariam_description_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_description'),
                'launch',
                'mariam_rsp.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
            # TODO: does this need to be set???
            # 'use_sim_time': 'false',
        }.items()
    )

    realsense_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_vision'),
                'launch',
                'rs_launch_modified.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_namespace': robot_name_launch_arg,
            'tf_prefix': robot_name_launch_arg,
            'unite_imu_method': '2', # use linear interpolation
            # enable these to use the IMU
            'enable_gyro': 'true',
            'enable_accel': 'true',
            
            # depth settings
            'enable_depth': 'true',
            'depth_module.depth_profile': '424,240,30',
            'depth_module.enable_auto_exposure': 'false',
            'align_depth.enable': 'true',

            # color settings
            'enable_color': 'true',
            'rgb_camera.color_profile': '424,240,30',
            'rgb_camera.enable_auto_exposure': 'false',

            'enable_sync': 'true',

            'initial_reset': 'true',
        }.items()
    )

    imu_filter_madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        namespace=robot_name_launch_arg,
        output='screen',
        parameters=[{
            'stateless': False,
            'use_mag': False,
            'publish_tf': False,
            'reverse_tf': False,
            'fixed_frame': "base_footprint",
            'constant_dt': 0.0,
            'publish_debug_topics': False,
            # NOTE: this paprameter sets the orientation 
            # of the axes for the robot
            'world_frame': 'enu',
            'gain': 0.03,
            'zeta': 0.001,
            'orientation_stddev': 0.05
        }],
        remappings=[
            ('imu/data_raw', 'camera/imu'),
            ('/tf', 'tf')
        ],
    )
            

    ekf_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'ekf.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        }.items()
    )

    slam_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'slam.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        }.items()
    )

    rtab_slam_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_localization'),
                'launch',
                'rtab_slam.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        }.items()
    )
    
    nav2_bringup_launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mariam_navigation'),
                'launch',
                'nav2_custom_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': robot_name_launch_arg,
        }.items()
    )

    robot_follower_node = Node(
        package='mariam_navigation',
        executable='robot_follower',
        name='robot_follower',
        namespace=robot_name_launch_arg,
        output='screen',
        parameters=[{
            'robot_name': robot_name_launch_arg,
        }]
    )

    return [
        # px100_controller_desc,
        micro_ros_desc,
        mariam_description_launch_desc,
        realsense_launch_desc,
        ekf_launch_desc,
        rtab_slam_launch_desc,

        # unused nodes / launch descriptions
        imu_filter_madgwick_node,
        # slam_launch_desc,
        # nav2_bringup_launch_desc
        # robot_follower_node
    ]


def generate_launch_description():
    # --------------------------------------------
    # Declaring launch arguments
    # --------------------------------------------
    declared_launch_arguments = [
        DeclareLaunchArgument(
            'robot_name',
            description='The name of the robot (ross or monica).',
        ),
        DeclareLaunchArgument(
            'use_admittance_control',
            default_value='true',
            choices=('true', 'false'),
            description='Whether to use admittance control'
        )
    ]

    return LaunchDescription(declared_launch_arguments +
                             [OpaqueFunction(function=launch_setup)])
