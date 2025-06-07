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
        #
        # We are running start_robot on separate threads
        # to start them up (nearly) in parallel.
        # -------------------------------------------------
        self.get_logger().info('Starting up the robots...')

        self.monica_conn = Connection(
            f'swarmie@monica.local',
            connect_kwargs={'look_for_keys': True}
        )
        
        self.ross_conn = Connection(
            f'swarmie@ross.local',
            connect_kwargs={'look_for_keys': True}
        )

        monica_thread = Thread(
            target=self.start_robots,
            args=(self.monica_conn, 'monica')
        )
        ross_thread = Thread(
            target=self.start_robots,
            args=(self.ross_conn, 'ross')
        )
        monica_thread.start()
        ross_thread.start()


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
        # the amount of time it should take to get from point A to point B
        sleep_period = distance / speed
        
        ross_twist = Twist()
        ross_twist.linear.x = speed
        ross_twist.linear.y = 0.0
        ross_twist.linear.z = 0.0
        ross_twist.angular.x = 0.0
        ross_twist.angular.y = 0.0
        ross_twist.angular.z = 0.0


        monica_twist = Twist()
        monica_twist.linear.x = (-speed if same_direction else speed)
        monica_twist.linear.y = 0.0
        monica_twist.linear.z = 0.0
        monica_twist.angular.x = 0.0
        monica_twist.angular.y = 0.0
        monica_twist.angular.z = 0.0


        self.ross_cmd_vel_publisher.publish(ross_twist)
        self.monica_cmd_vel_publisher.publish(monica_twist)
        time.sleep(sleep_period)

        ross_twist = Twist()
        ross_twist.linear.x = 0.0
        ross_twist.linear.y = 0.0
        ross_twist.linear.z = 0.0
        ross_twist.angular.x = 0.0
        ross_twist.angular.y = 0.0
        ross_twist.angular.z = 0.0

        monica_twist = Twist()
        monica_twist.linear.x = 0.0
        monica_twist.linear.y = 0.0
        monica_twist.linear.z = 0.0
        monica_twist.angular.x = 0.0
        monica_twist.angular.y = 0.0
        monica_twist.angular.z = 0.0


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


    def reset_arm_positions(self):
        self.get_logger().info('Resetting arm positions...')

        # reset the arm positions to 0.067 meters
        self.ross_arm_publisher.publish(Float64(data=0.067))
        self.monica_arm_publisher.publish(Float64(data=0.067))

        # wait for the arms to reset
        time.sleep(3)


    def start_robots(self, conn, robot_name):
        with conn.cd('MARIAM'):
            # this command will start up the robot and save the output to a log file
            # this log file is mostly used for debugging purposes
            cmd = f'./script/startup_robot.sh {robot_name} true > ./log/experiment.log &'
            
            # start up the robot
            result = conn.run(
                cmd,
                hide=True,
                # pty=True,
                
                # this is similar to pty, but does not return
                # a promise. It's useful for shell backgrounding
                # which is currently how we run the software on each robot
                # remotely
                disown=True
                # make the command run asynchronously
                # this prevents the command from hanging
                # asynchronous=True
            )


    def shutdown_robots(self):
        """
        This method will shutdown the robots by closing the connections.
        """
        self.get_logger().info('Shutting down the robots...')
        shutdown_robot_cmd = 'pkill -2 -f "mariam_experiments"'
        reset_arduino_cmd = 'stty -F /dev/ttyACM0 hupcl && echo "reset" > /dev/ttyACM0 && sleep 2'

        # shutdown the robots
        self.monica_conn.run(shutdown_robot_cmd)
        self.ross_conn.run(shutdown_robot_cmd)

        # Reset Arduino by toggling DTR
        self.monica_conn.run(reset_arduino_cmd)
        self.ross_conn.run(reset_arduino_cmd)



def main(args=None):
    rclpy.init(args=args)
    experiment_node = ExperimentNode()
    
    # period of time between each stage of the experiment
    time_interval = 3.0 # seconds
    robot_speed = 0.1  # meters per second
    # when the robots drive up to the object
    drive_up_distance = 0.25  # meters
    # when the robots drive away from the object
    drive_away_distance = 1.0  # meters

    try:        
        while rclpy.ok():
            input("Press Enter to start an experiment\n")

            # drive up to the object
            experiment_node.drive_robots(
                speed=robot_speed,
                distance=drive_up_distance,
                same_direction=False
            )

            time.sleep(time_interval)
            
            # lift the object
            experiment_node.lift_object()
            
            time.sleep(time_interval)
            
            # drive 1 meter away from the object
            experiment_node.drive_robots(
                speed=robot_speed,
                distance=drive_away_distance,
                same_direction=True
            )
    
    except KeyboardInterrupt:
        experiment_node.shutdown_robots()
        experiment_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
