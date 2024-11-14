import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the path to the Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('mariam_description'),
        'models/mariam.urdf.xacro'
    )

    # Ensure the Xacro file exists
    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    # Convert Xacro to URDF (done at launch time using the `xacro` command)
    urdf_content = Command(['xacro ', xacro_file])

    # Define the Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            # 'verbose': verbose_launch_arg,
            # 'world': world_filepath_launch_arg,
            'pause': 'true',
            # 'record': recording_launch_arg,
            # 'gdb': debug_launch_arg,
            # 'valgrind': debug_launch_arg,
            # 'gui': use_gazebo_gui_launch_arg,
        }.items(),
    )

    # Define the spawn_entity node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_description',  # Name of the entity
            '-topic', 'px100/robot_description',  # Provide the description via a topic
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen'
    )

    # Publish the URDF to the /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='px100',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    spawn_joint_state_broadcaster_node = Node(
        name='joint_state_broadcaster_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='px100',
        arguments=[
            '-c',
            'controller_manager',
            'joint_state_broadcaster',
        ],
        parameters=[{
            'use_sim_time': True,
        }],
    )

    spawn_arm_controller_node = Node(
        name='arm_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='px100',
        arguments=[
            '-c',
            'controller_manager',
            'arm_controller',
        ],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    spawn_gripper_controller_node = Node(
        name='gripper_controller_spawner',
        package='controller_manager',
        executable='spawner',
        namespace='px100',
        arguments=[
            '-c',
            'controller_manager',
            'gripper_controller',
        ],
        parameters=[{
            'use_sim_time': True,
        }]
    )

    load_joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[spawn_joint_state_broadcaster_node]
        )
    )

    load_arm_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_arm_controller_node]
        )
    )

    load_gripper_controller_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster_node,
            on_exit=[spawn_gripper_controller_node]
        )
    )

    # Launch description with Gazebo, URDF publisher, and spawn_entity node
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_joint_state_broadcaster_node,
        spawn_arm_controller_node,
        spawn_gripper_controller_node,
        spawn_entity_node,
        load_arm_controller_event,
        load_gripper_controller_event,
        load_joint_state_broadcaster_event,
    ])
