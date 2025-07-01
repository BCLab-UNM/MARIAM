from launch import LaunchDescription

from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    
    # determines if the force node should be created
    force_node_launch_arg = LaunchConfiguration('use_fake_force')
    admittance_rviz_markers_launch_arg = LaunchConfiguration('use_rviz_markers')
    use_dynamic_parameterization_launch_arg = LaunchConfiguration('use_dynamic_parameterization')

    config_file = os.path.join(
        get_package_share_directory('arm_controller'),
        'config/admittance_control.yaml'
    )

    admittance_controller_node = Node(
       name='admittance_controller_node',
       package='arm_controller',
       executable='admittance_controller',
       namespace=robot_name_launch_arg,
       parameters=[config_file]
    )

    admittance_rviz_markers_node = Node(
       name='admittance_rviz_markers_node',
       package='arm_controller',
       executable='admittance_rviz_markers',
       namespace=robot_name_launch_arg,
       condition=IfCondition(
           PythonExpression([
               "'", admittance_rviz_markers_launch_arg,
               "' == 'true'"
           ])
       )
    )

    # Admittance control nodes
    virtual_pose_node = Node(
       package='arm_controller',
       executable='virtual_pose_publisher',
       name='virtual_pose_publisher_node',
       namespace=robot_name_launch_arg,
       parameters=[config_file]
    )

    force_node = Node(
       package='arm_controller',
       executable='force_publisher',
       name='force_publisher_node',
       namespace=robot_name_launch_arg,
       parameters=[config_file],
       condition=IfCondition(
           PythonExpression([
               "'", force_node_launch_arg,
               "' == 'true'"
           ])
       )
    )

    dynamic_parameterization_node = Node(
        package='mariam_dynamic_parameterization',
        executable='centralized_parameterizer',
        name='centralized_parameterizer_node',
        namespace=robot_name_launch_arg,
        parameters=[{
            'robot_namespace': [robot_name_launch_arg.perform(context)]
        }],
        condition=IfCondition(
            PythonExpression([
                "'", use_dynamic_parameterization_launch_arg,
                "' == 'true'"
            ])
        )
    )

    return LaunchDescription([
        # Declaring launch arguments
        DeclareLaunchArgument(
            'robot_name',
            default_value='px100'
        ),
        DeclareLaunchArgument(
            'use_fake_force',
            default_value='true',
            choices=('true', 'false'),
            description=("If true, launches a node to publish fake force measurements")
        ),
        DeclareLaunchArgument(
            'use_rviz_markers',
            default_value='true',
            choices=('true', 'false')
        ),
        DeclareLaunchArgument(
            'use_dynamic_parameterization',
            default_value='true',
            choices=('true', 'false'),
            description=("If true, launches the centralized parameterizer node")
        ),

        admittance_controller_node,
        virtual_pose_node,
        force_node,
        admittance_rviz_markers_node,
        dynamic_parameterization_node
    ])


   