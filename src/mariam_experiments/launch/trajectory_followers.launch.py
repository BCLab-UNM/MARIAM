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
            namespace='monica',
            parameters=[{
                'x_0': -0.45,
                'y_0': -0.36,
                'x_f': 2.12,
                'y_f': 1.95,
                'trajectory_duration': 30.0,
                'control_frequency': 50.0,
                'max_linear_vel': 0.2,
                'max_angular_vel': 0.4,
                'quadratic_coeff': -1.0,
            }],
            output='screen'
        ),
        Node(
            package='mariam_navigation',
            executable='trajectory_follower',
            namespace='ross',
            parameters=[{
                'x_0': -1.49,
                'y_0': -0.26,
                'x_f': 1.61,
                'y_f': 1.29,
                'trajectory_duration': 30.0,
                'control_frequency': 50.0,
                'max_linear_vel': 0.2,
                'max_angular_vel': 0.4,
                'quadratic_coeff': -1.0,
            }],
            output='screen'
        )
    ])
