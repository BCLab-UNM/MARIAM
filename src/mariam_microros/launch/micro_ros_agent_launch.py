from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Micro ROS currently does not support setting node or namespace names
# on the client side, this has to be done in the .inu code.

# Therefore this launch file will not currenlty do what its intended to
# until an update is made to the micro_ros_agent package

def generate_launch_description():
    # Declare a launch argument for robot_name
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',  # Default value if none is provided
        description='Name of the robot to set the namespace and node name'
    )

    # Use LaunchConfiguration to get the value of robot_name
    robot_name = LaunchConfiguration('robot_name')

    # Define the Node with namespace and node name based on robot_name
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        namespace=robot_name,
        name=['micro_ros_arduino_node_on_', robot_name],  # Dynamic node name
        arguments=['serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        micro_ros_agent_node
    ])
