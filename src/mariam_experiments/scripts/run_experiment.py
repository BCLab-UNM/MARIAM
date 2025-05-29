#!/usr/bin/env python3

from fabric import SerialGroup, Connection
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
# TODO: should probably change it to the stamped versions
from geometry_msgs.msg import Pose, Twist

import time
from threading import Thread

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
        # start up monica
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
        """
        :param hostname: the hostname of the robot to connect to
        :param robot_name: the name of the robot to start
        """
        # establish an SSH connection using an SSH key
        self.monica_conn = Connection(
            f'swarmie@monica.local',
            connect_kwargs={'look_for_keys': True}
        )

        # start up the robot
        result = self.monica_conn.run(f'./MARIAM/script/startup_robot.sh monica true &', hide=True)

        if result.ok:
            self.get_logger().info(f'Successfully started monica')
        else:
            self.get_logger().error(f'Failed to start monica: {result.stderr}')

    def shutdown_robots(self, hostname):
        """
        This method will shutdown the robots by closing the connections.
        """
        self.get_logger().info('Shutting down the robots...')

        # shutdown the robot
        result = self.monica_conn.run('pkill -2 -f "mariam_experiments"', hide=True)

        if result.ok:
            self.get_logger().info(f'Successfully shut down monica')
        else:
            self.get_logger().error(f'Failed to shut down monica: {result.stderr}')


def main(args=None):
    rclpy.init(args=args)
    experiment_node = ExperimentNode()

    try:        
        while rclpy.ok():
            input('Press Enter to start an experiment ')
            # drive up to the object
            # experiment_node.drive_robots(
            #     speed=0.1,  # meters per second
            #     distance=0.25  # meters
            # )
            # experiment_node.lift_object()
            # # drive away from the object
            # experiment_node.drive_robots(
            #     speed=0.1,  # meters per second
            #     distance=1.0  # meters
            # )
    
    except KeyboardInterrupt:
        experiment_node.get_logger().info('Shutting down the experiment node...')
        experiment_node.shutdown_robots(hostname='monica.local')
        experiment_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
