import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  mariam_description_pkg_share = FindPackageShare(package='mariam_description').find('mariam_description')
  mariam_navigation_pkg_share = FindPackageShare(package='mariam_navigation').find('mariam_navigation')
  mariam_gazebo_pkg_share = FindPackageShare(package='mariam_gazebo').find('mariam_gazebo')

  xacro_model_path = os.path.join(mariam_description_pkg_share, 'xacro_models/mariam.urdf.xacro')
  default_sdf_model_path = os.path.join(mariam_description_pkg_share, 'gazebo_models/mariam_description/mariam.sdf')
  robot_name = 'mariam'

  robot_localization_file_path = os.path.join(mariam_navigation_pkg_share, 'config/ekf.yaml') 
  default_rviz_config_path = os.path.join(mariam_description_pkg_share, 'rviz/mariam_config.rviz')
  
  world_file_name = 'shapes.world'
  world_path = os.path.join(mariam_gazebo_pkg_share, 'worlds', world_file_name)
  
  # Launch configuration variables specific to simulation
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  world = LaunchConfiguration('world')

  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=xacro_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    launch_arguments={'world': world, 'pause': 'true'}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
  
  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': Command(['xacro ', xacro_model_path])}])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
  
  # Spawn the robot into Gazebo
  spawn_robot_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', robot_name, '-file', default_sdf_model_path, '-x', '0', '-y', '0', '-z', '0.1'],
    output='screen'
  )

  # Spawn the robot into Gazebo
  rqt_robot_steering_cmd = Node(
    package='rqt_robot_steering',
    executable='rqt_robot_steering',
  )

  # Start Depth to LaserScan Node
  # use 'LIBGL_ALWAYS_SOFTWARE=1 rviz2' if crashes
  start_depth_to_laserscan_cmd = Node(
    package='depthimage_to_laserscan',
    executable='depthimage_to_laserscan_node',
    name='depthimage_to_laserscan',
    output='screen',
    parameters=[
        {'scan_time': 0.033},
        {'range_min': 0.45},
        {'range_max': 100.0},
        {'scan_height': 1},
        {'output_frame': 'camera_link'},
    ],
    remappings=[
        ('depth', '/camera/depth/image_raw'),
        ('depth_camera_info', '/camera/depth/camera_info'),
        ('scan', '/scan'),
    ]
  )
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(spawn_robot_cmd)
  ld.add_action(rqt_robot_steering_cmd)
  # ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_depth_to_laserscan_cmd)

  return ld