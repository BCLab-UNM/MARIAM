from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# Launch with:
# ros2 launch mariam_localization hardware_ekf.launch.py namespace:=monica

def generate_launch_description():
    # Declare the launch argument for the namespace.
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the ekf node'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('mariam_localization'),
            'config',
            'ekf_params.yaml'
        ),
        description='Path to the EKF configuration file'
    )

    # Use the value of the namespace launch argument.
    namespace = LaunchConfiguration('namespace')

    config_file = LaunchConfiguration('config_file')


    # Define the EKF node, applying the namespace.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[config_file],
        remappings=[
            # TF remappings
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/joint_states', 'joint_states'),
            # Input topic remappings
            ('/wheel/odom', 'wheel/odom'),
            ('/camera/imu', 'camera/imu')
        ]
    )

    return LaunchDescription([
        namespace_arg,
        ekf_node
    ])
