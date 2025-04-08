from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    namespace_arg = LaunchConfiguration('namespace')
    
    # this node publishes raw data from the IMU
    imu_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output='screen',
        parameters=[{'use_mag': False,
                     'world_frame': 'enu',
                     'publish_tf': False}],
        # remappings=[('imu/data_raw',
                    #  '/{}/camera/imu'.format(namespace_arg.perform(context)))],
        namespace=namespace_arg.perform(context)
    )

    return [imu_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description="Put's the imu node under a namespace"
        )
    ] + [OpaqueFunction(function=launch_setup)])
