from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument for namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )
    
    return LaunchDescription([
        namespace_arg,
        
        # Launch the distributed parameterizer node
        Node(
            package='mariam_dynamic_parameterization',  # Replace with your actual package name
            executable='distributed_parameterizer',
            name='distributed_parameterizer_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                # Add any parameters here if needed
            ],
            remappings=[
                # Add any topic remappings here if needed
                # ('old_topic', 'new_topic'),
            ]
        ),
        
        # Launch the agent displacement estimator node
        Node(
            package='mariam_dynamic_parameterization',  # Replace with your actual package name
            executable='agent_displacement_estimator',
            name='agent_displacement_estimator_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                # Add any parameters here if needed
            ],
            remappings=[
                # Add any topic remappings here if needed
                # ('old_topic', 'new_topic'),
            ]
        ),
    ])