from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Dynamically locate the path to the config file
    config_path = os.path.join(
        get_package_share_directory('mariam_navigation'),
        'config',
        'ekf_hardware.yaml'
    )
    print(f"Looking for config file at: {config_path}")

    # Declare the launch argument for the namespace.
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the ekf node'
    )

    # Use the value of the namespace launch argument.
    namespace = LaunchConfiguration('namespace')

    # Define the EKF node, applying the namespace.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[config_path],
        remappings=[
            # TF remappings
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            # Input topic remappings
            ('/wheel/odom', 'wheel/odom'),
            ('/camera/imu', 'camera/imu')
        ]
    )

    return LaunchDescription([
        namespace_arg,
        ekf_node
    ])
