from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace for rviz2'
    )
    
    namespace = LaunchConfiguration('namespace')
    
    config_file_path = PathJoinSubstitution([
        FindPackageShare('mariam_description'),
        'rviz',
        PythonExpression(["'", namespace, "' + '_config.rviz'"])
    ])
    
    # RViz2 node with comprehensive remappings and config file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['-d', config_file_path],
        remappings=[
            # Transform topics
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            # Interactive markers
            ('/clicked_point', 'clicked_point'),
            ('/goal_pose', 'goal_pose'),
            ('/initialpose', 'initialpose'),
            # Diagnostics
            ('/diagnostics', 'diagnostics'),
            # Visualization topics
            ('/map', 'map'),
            ('/local_costmap/costmap', 'local_costmap/costmap'),
            ('/global_costmap/costmap', 'global_costmap/costmap'),
            ('/particle_cloud', 'particle_cloud'),
            ('/scan', 'scan'),
            ('/camera/image_raw', 'camera/image_raw'),
            ('/camera/camera_info', 'camera/camera_info'),
            ('/pointcloud', 'pointcloud'),
            ('/pointcloud2', 'pointcloud2'),
            # Navigation topics
            ('/move_base_simple/goal', 'move_base_simple/goal'),
            ('/plan', 'plan'),
            ('/global_plan', 'global_plan'),
            ('/local_plan', 'local_plan'),
            ('/cmd_vel', 'cmd_vel'),
            ('/odom', 'odom'),
            # Robot state topics
            ('/joint_states', 'joint_states'),
            ('/robot_description', 'robot_description'),
            # Markers and visualization
            ('/visualization_marker', 'visualization_marker'),
            ('/visualization_marker_array', 'visualization_marker_array'),
            ('/path', 'path'),
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        rviz_node
    ])