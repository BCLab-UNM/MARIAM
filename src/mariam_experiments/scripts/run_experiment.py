#!/usr/bin/env python3

from fabric import Connection
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

import time
from threading import Thread

import math

class ExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        # -------------------------------------------------
        # creating publishers
        # -------------------------------------------------
        nav2_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)


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
        self.ross_nav2_pose_publisher = self.create_publisher(
            PoseStamped,
            '/ross/goal_pose',
            nav2_pose_qos
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
        self.monica_nav2_pose_publisher = self.create_publisher(
            PoseStamped,
            '/monica/goal_pose',
            nav2_pose_qos
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

    def drive_robot_nav2(self, robot_name: str, pose: PoseStamped):
        """
        This method will command the robot to drive to the specified pose
        using Nav2.

        :param robot_name: the name of the robot ('monica' or 'ross')
        :param pose: the pose for the robot to drive to
        """
        pose.header.stamp = self.get_clock().now().to_msg()
        if robot_name == 'monica':
            self.get_logger().info('Publishing desired pose for Monica')
            self.monica_nav2_pose_publisher.publish(pose)
        elif robot_name == 'ross':
            self.get_logger().info('Publishing desired pose for Ross')
            self.ross_nav2_pose_publisher.publish(pose)
        else:
            self.get_logger().error(f'Unknown robot name: {robot_name}')


    def drive_robots_nav2(self, monica_pose: PoseStamped, ross_pose: PoseStamped):
        """
        This method will command the robots to drive to the specified poses
        using Nav2.

        :param monica_pose: the pose for Monica to drive to
        :param ross_pose: the pose for Ross to drive to
        """
        self.get_logger().info('Publishing desired poses...')
        monica_pose.header.stamp = self.get_clock().now().to_msg()
        ross_pose.header.stamp = self.get_clock().now().to_msg()

        # publish the poses to the respective topics
        self.get_logger().debug(f'Ross Pose: {ross_pose}')
        self.get_logger().debug(f'Monica Pose: {monica_pose}')

        self.ross_nav2_pose_publisher.publish(ross_pose)
        self.monica_nav2_pose_publisher.publish(monica_pose)

    def start_trajectory_follower(self):
        monica_cmd = 'ROS_DOMAIN_ID=1 ros2 run mariam_navigation trajectory_follower --ros-args -r __ns:=/monica -p x_0:=-0.45 -p y_0:=-0.36 -p x_f:=2.12 -p y_f:=1.95 -p trajectory_duration:=30.0 -p control_frequency:=50.0 -p max_linear_vel:=0.2 -p max_angular_vel:=0.4 -p quadratic_coeff:=1.0'

        ross_cmd = 'ROS_DOMAIN_ID=2 ros2 run mariam_navigation trajectory_follower --ros-args -r __ns:=/ross -p x_0:=-1.49 -p y_0:=-0.26 -p x_f:=1.61 -p y_f:=1.29 -p trajectory_duration:=30.0 -p control_frequency:=50.0 -p max_linear_vel:=0.2 -p max_angular_vel:=0.4 -p quadratic_coeff:=1.0'

        self.monica_conn.run(monica_cmd, hide=True, disown=True)
        self.ross_conn.run(ross_cmd, hide=True, disown=True)

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

        # shutdown the robots
        self.monica_conn.run(shutdown_robot_cmd)
        self.ross_conn.run(shutdown_robot_cmd)


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

    # variables for Nav2
    # NOTE: we will only set the timestamp when we want to publish the pose
    monica_pose = PoseStamped()
    monica_pose.header.frame_id = 'map'
    monica_pose.pose.position.x = 0.0
    monica_pose.pose.position.y = 0.0
    monica_pose.pose.position.z = 0.0
    monica_pose.pose.orientation.w = math.cos(math.pi / 2)
    monica_pose.pose.orientation.x = 0.0
    monica_pose.pose.orientation.y = 0.0
    monica_pose.pose.orientation.z = math.sin(math.pi / 2)
    
    monica_pose2 = PoseStamped()
    monica_pose2.header.frame_id = 'map'
    monica_pose2.pose.position.x = 0.0
    monica_pose2.pose.position.y = 0.0
    monica_pose2.pose.position.z = 0.0
    monica_pose2.pose.orientation.w = math.cos(0)
    monica_pose2.pose.orientation.x = 0.0
    monica_pose2.pose.orientation.y = 0.0
    monica_pose2.pose.orientation.z = math.sin(0)
    
    monica_pose3 = PoseStamped()
    monica_pose3.header.frame_id = 'map'
    monica_pose3.pose.position.x = -0.5
    monica_pose3.pose.position.y = 0.0
    monica_pose3.pose.position.z = 0.0
    monica_pose3.pose.orientation.w = math.cos(0)
    monica_pose3.pose.orientation.x = 0.0
    monica_pose3.pose.orientation.y = 0.0
    monica_pose3.pose.orientation.z = math.sin(0)

    ross_pose = PoseStamped()
    ross_pose.header.frame_id = 'map'
    ross_pose.pose.position.x = 0.5
    ross_pose.pose.position.y = 0.5
    ross_pose.pose.position.z = 0.0
    ross_pose.pose.orientation.w = math.cos(0)
    ross_pose.pose.orientation.x = 0.0
    ross_pose.pose.orientation.y = 0.0
    ross_pose.pose.orientation.z = math.sin(0)

    try:        
        while rclpy.ok():
            input("Press CTRL+C to kill the robots\n")            
    
    except KeyboardInterrupt:
        experiment_node.shutdown_robots()
        experiment_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
