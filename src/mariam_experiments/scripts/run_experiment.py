#!/usr/bin/env python3

from fabric import SerialGroup
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geographic_msgs.msg import Pose

import time

class ExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        self.ross_arm_publisher = self.create_publisher(
            Float64,
            '/ross/px100_virtual_pose_updater',
            10
        )
        
        self.monica_arm_publisher = self.create_publisher(
            Float64,
            '/monica/px100_virtual_pose_updater',
            10
        )

        self.get_logger().info('Starting up the robots...')
        self.start_robots()
        
    # -----------------------------------------------------
    # methods for each stage of the experiment
    # -----------------------------------------------------
    def drive_up(self):
        pass

    def lift_object(self):
        height = 0.067
        for i in range(100):
            # increase the height by a millimeter
            height += 0.001
            self.ross_arm_publisher.publish(Float64(data=height))
            self.monica_arm_publisher.publish(Float64(data=height))
            time.sleep(45e-3)  # sleep for 45 milliseconds

    def drive_away(self):
        pass

    def start_robots(self):
        serial_group = SerialGroup(
            'ross.local', 'monica.local',
            user='swarmie',
            # searches for SSH keys
            connect_kwargs={'look_for_keys': True}
        ).run('cd MARIAM')

        for (conn, _), robot_name in zip(serial_group.items(), ['ross', 'monica']):
            self.get_logger().info(f'Starting robot: {robot_name}')
            conn.run(f'ros2 launch mariam_experiments experiment.launch.py robot_name:={robot_name} use_gazebo:=false use_admittance_control:=true')
            self.get_logger().info(f'Robot {robot_name} started successfully.')


def main(args=None):
    rclpy.init(args=args)
    experiment_node = ExperimentNode()

    try:
        # run the experiment stages
        experiment_node.drive_up()
        experiment_node.lift_object()
        experiment_node.drive_away()
    
    except KeyboardInterrupt:
        pass
    
    finally:
        experiment_node.get_logger().info('Shutting down the experiment node...')
        experiment_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
