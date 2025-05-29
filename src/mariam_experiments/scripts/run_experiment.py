#!/usr/bin/env python3

from fabric import SerialGroup
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
# TODO: should probably change it to the stamped versions
from geometry_msgs.msg import Pose, Twist

import time

class ExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        # -------------------------------------------------
        # creating publishers
        # -------------------------------------------------
        self.ross_arm_publisher = self.create_publisher(
            Float64,
            '/ross/px100_virtual_pose_updater',
            10
        )
        self.ross_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/ross/cmd_vel',
            10
        )
        
        self.monica_arm_publisher = self.create_publisher(
            Float64,
            '/monica/px100_virtual_pose_updater',
            10
        )
        self.monica_cmd_vel_publisher = self.create_publisher(
            Twist,
            '/monica/cmd_vel',
            10
        )


        # -------------------------------------------------
        # starting up the robots
        # -------------------------------------------------
        self.get_logger().info('Starting up the robots...')
        self.start_robots()
        
    # -----------------------------------------------------
    # methods for each stage of the experiment
    # -----------------------------------------------------
    def drive_robots(self, speed=0.1, distance=0.25, same_direction=False):
        """
        This method will command the robots to drive up to the object
        at a fixed speed.

        :param speed: the speed (meters per second) at which the robots will drive up
        :param distance: the distance (meters) the robots will drive up
        :param same_direction: if True, both robots will drive in the same direction
                                 (monica will drive backwards)
        """
        sleep_period = distance / speed  # time to drive up
        
        ross_twist = Twist()
        ross_twist.linear.x = speed

        monica_twist = Twist()
        monica_twist.linear.x = (-speed if same_direction else speed)

        self.ross_cmd_vel_publisher.publish(ross_twist)
        # sleep for 0.5 seconds
        time.sleep(sleep_period)

        ross_twist = Twist()
        ross_twist.linear.x = 0

        monica_twist = Twist()
        monica_twist.linear.x = 0

        self.ross_cmd_vel_publisher.publish(ross_twist)
        self.monica_cmd_vel_publisher.publish(monica_twist)


    def lift_object(self):
        height = 0.067
        for _ in range(100):
            # increase the height by a millimeter
            height += 0.001
            self.ross_arm_publisher.publish(Float64(data=height))
            self.monica_arm_publisher.publish(Float64(data=height))
            time.sleep(45e-3)  # sleep for 45 milliseconds


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
        # drive up to the object
        experiment_node.drive_robots(
            speed=0.1,  # meters per second
            distance=0.25  # meters
        )
        experiment_node.lift_object()
        # drive away from the object
        experiment_node.drive_robots(
            speed=0.1,  # meters per second
            distance=1.0  # meters
        )
    
    except KeyboardInterrupt:
        # TODO: handle the keyboard interrupt gracefully.
        # this should properly stop each robot using the
        # connections created by the SerialGroup
        pass
    
    finally:
        experiment_node.get_logger().info('Shutting down the experiment node...')
        experiment_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
