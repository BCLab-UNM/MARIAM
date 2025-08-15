from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mariam_navigation',
            executable='trajectory_follower',
            namespace='ross',
            parameters=[{
                'x_0': -1.70,
                'y_0': -0.26,
                'x_f': 1.51,
                'y_f': -0.26,
                'trajectory_duration': 25.0,
                'control_frequency': 50.0,
                'max_linear_vel': 0.15,
                'max_angular_vel': 0.4,
                'parabola_coeff': 0.75,
                'num_waypoints': 5
            }],
            output='screen'
        ),
        Node(
            package='mariam_navigation',
            executable='trajectory_follower',
            namespace='monica',
            parameters=[{
                'x_0': -0.6,
                'y_0': -0.36,
                'x_f': 2.55,
                'y_f': -0.36,
                'trajectory_duration': 25.0,
                'control_frequency': 50.0,
                'max_linear_vel': 0.15,
                'max_angular_vel': 0.4,
                'parabola_coeff': 0.75,
                'num_waypoints': 5
            }],
            output='screen'
        )
    ])
