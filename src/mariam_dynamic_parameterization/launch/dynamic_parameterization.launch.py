import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch argument for namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )
    
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('mariam_dynamic_parameterization'),
        'config/parameterizer.yaml'
    )
    
    return LaunchDescription([
        namespace_arg,
        # Launch the distributed parameterizer node
        Node(
            package='mariam_dynamic_parameterization',
            executable='distributed_parameterizer',
            name='distributed_parameterizer',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
        # Launch the agent displacement estimator node
        Node(
            package='mariam_dynamic_parameterization',
            executable='vicon_agent_displacement_estimator',
            name='agent_displacement_estimator',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),
    ])