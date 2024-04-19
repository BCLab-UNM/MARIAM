from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def slam_setup(context):
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': False,
        'wait_imu_to_init': True
    }]

    remappings = [
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/realigned_depth_to_color/image_raw'),
        ('odom','odom_slam')
    ]

    return [
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            output='screen',
            parameters=parameters,
            remappings=remappings
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']
        )
    ]

def realsense_setup(context):
    configurable_parameters = [
        {'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
        {'name': 'camera_namespace', 'default': 'camera', 'description': 'namespace for camera'},
        {'name': 'log_level', 'default': 'info', 'description': 'log level for the camera node'},
        {'name': 'output', 'default': 'screen', 'description': 'output for the camera node'},
        # Add more configurable parameters if needed
    ]

    def declare_configurable_parameters(parameters):
        return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

    def set_configurable_parameters(parameters):
        return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

    def launch_setup(context, params):
        _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
        params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)
        return [
            launch_ros.actions.Node(
                package='realsense2_camera',
                namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
                name=LaunchConfiguration('camera_name' + param_name_suffix),
                executable='realsense2_camera_node',
                parameters=[params, params_from_file],
                output=LaunchConfiguration('output' + param_name_suffix),
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
                emulate_tty=True,
            )
        ]

    return declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs={'params': set_configurable_parameters(configurable_parameters)})
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # RealSense camera setup
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)}),
        # SLAM setup
        OpaqueFunction(function=slam_setup)
    ])
