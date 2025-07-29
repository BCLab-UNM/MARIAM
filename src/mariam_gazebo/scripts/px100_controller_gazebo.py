#!/usr/bin/env python3

import argparse
import sys
import copy
from threading import Lock
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import numpy as np
from scipy.spatial.transform import Rotation as R
from modern_robotics import IKinSpace


class ArmController(Node):
    """
    This class is a position controller for the Interbotix PX100.
    """
    # used to lock the desired_pose field to avoid race conditions
    lock = Lock()

    # the pose we want the robot to be in
    # Let the default be where start_lift() will place the robot
    desired_pose = Pose(
        position=Point(x=0.0, y=0.230, z=0.067),
        orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
    )

    # position of the base link for each arm in their respective odom frames
    monica_base_in_odom_trans_matrix = np.array(
        [[0,  1, 0, 0.160],
         [-1,  0, 0, 0.0],
         [0,  0, 1, 0.095],
         [0,  0, 0, 1]]
    )
    ross_base_in_odom_trans_matrix = np.array(
        [[0,  1, 0, 0.160],
         [-1,  0, 0, 0.0],
         [0,  0, 1, 0.095],
         [0,  0, 0, 1]]
    )

    # transformation matrix between the odom frame and the other odom frame
    odom_frame_trans_matrix = np.array([
        [-1, 0, 0, 1.657],
        [0, -1, 0, 0.0],
        [0,  0, 1, 0.0],
        [0,  0, 0, 1.0]
    ])

    last_trans_matrix = np.eye(4)

    Slist = np.array([
        [0.0, 0.0, 1.0, 0.0,     0.0, 0.0],
        [0.0, 1.0, 0.0, -0.0931, 0.0, 0.0],
        [0.0, 1.0, 0.0, -0.1931, 0.0, 0.035],
        [0.0, 1.0, 0.0, -0.1931, 0.0, 0.135]
    ]).T

    M = np.array([
        [1.0, 0.0, 0.0, 0.248575],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.1931],
        [0.0, 0.0, 0.0, 1.0]
    ])


    last_joint_cmds = np.array([0.0, 0.0, 0.0, 0.0])

    joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']


    def __init__(self):
        super().__init__('px100_controller')
        # the rate at which the while loop in 'start_robot()' will run
        # (in Hz)
        loop_rate = 5
        self.rate = self.create_rate(loop_rate)
        self.robot_name = self.get_namespace().strip('/')

        # this is the amount of time it will take the robot's
        # joints to move from position A to position B
        self.duration = Duration()
        self.duration.sec = 0
        self.duration.nanosec = 100_000_000

        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            'arm_controller/joint_trajectory',
            10
        )

        self.create_subscription(
            Pose,
            'px100_target_pose',
            self.update_desired_pose_cb,
            10
        )

        self.create_subscription(
            Pose,
            '/monica/px100_base_link_pose',
            self.update_monica_base_link_pose_cb,
            10
        )

        self.create_subscription(
            Pose,
            '/ross/px100_base_link_pose',
            self.update_ross_base_link_pose_cb,
            10
        )

        # creating a timmer to periodically move the end
        # effector
        self.create_timer(
            timer_period_sec=0.002,
            callback=self.move_end_effector
        )


    def move_end_effector(self) -> None:
        """
        This function is called periodically to command the arm
        to move to a desired position.
        """

        with self.lock:
            desired_pose = self.desired_pose

        # create a rotation matrix from the desired pose's quaternion
        desired_rotation_matrix = np.eye(4)
        desired_rotation_matrix[:3, :3] = R.from_quat([
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix()

        # create a homogeneous transformation matrix from the
        # desired pose and the rotation matrix
        desired_trans_matrix = np.eye(4)
        desired_trans_matrix[:3, :3] = desired_rotation_matrix[:3, :3]
        desired_trans_matrix[0, 3] = desired_pose.position.x
        desired_trans_matrix[1, 3] = desired_pose.position.y
        desired_trans_matrix[2, 3] = desired_pose.position.z

        desired_trans_matrix = self.adjust_heading(desired_trans_matrix)

        # compute the desired joint angles
        joint_cmds, success = IKinSpace(
            Slist=self.Slist,
            M=self.M,
            T=desired_trans_matrix,
            thetalist0=self.last_joint_cmds,
            eomg=0.001,
            ev=0.001
        )

        if success:
            trajectory_point = JointTrajectoryPoint()
            # cast the numpy array into a list
            trajectory_point.positions = list(joint_cmds)
            trajectory_point.time_from_start = self.duration

            trajectory_msg = JointTrajectory()

            trajectory_msg.joint_names = self.joint_names
            trajectory_msg.points = [trajectory_point]
            
            self.get_logger().debug(f'Publishing joint commands: {joint_cmds}')
            self.joint_trajectory_pub.publish(trajectory_msg)

            self.last_joint_cmds = joint_cmds
        else:
            self.log_info('IK solver failed to find a solution')

    def adjust_heading(self, T_sd):
        """
        Update the waist angle to account for the other robot's heading.

        @param T_sd: The desired transformation matrix for the end effector.
        This will be adjusted directly in this function.
        """
        # compute the desired position of the end effector
        # within a virtual frame that removes the yaw angle
        yaw_angle = np.arctan2(T_sd[1, 3], T_sd[0, 3])

        T_sy = np.eye(4)

        T_sy[:3, :3] = np.array(
            R.from_euler('z', yaw_angle, degrees=False).as_matrix())

        T_yd = np.linalg.inv(T_sy) @ T_sd

        if self.robot_name == 'monica':
            # compute the pose of ross' base in monica's odom frame
            T_ross_base_in_monica_odom = self.odom_frame_trans_matrix \
                @ self.ross_base_in_odom_trans_matrix

            # compute the pose of ross' base relative to monica's base link
            T_ross_base_in_monica_base = np.linalg.inv(
                self.monica_base_in_odom_trans_matrix) \
                @ T_ross_base_in_monica_odom

            # extract the angle from the transformation matrix
            theta = np.arctan2(T_ross_base_in_monica_base[1, 3],
                                T_ross_base_in_monica_base[0, 3])

        elif self.robot_name == 'ross':
            # compute the pose of monica's base in ross' odom frame
            T_monica_base_in_ross_odom = self.odom_frame_trans_matrix \
                @ self.monica_base_in_odom_trans_matrix
                
            # compute the pose of monica's base relative to ross' base link
            T_monica_base_in_ross_base = np.linalg.inv(self.ross_base_in_odom_trans_matrix) \
                @ T_monica_base_in_ross_odom

            # extract the angle from the transformation matrix
            theta = np.arctan2(T_monica_base_in_ross_base[1, 3],
                                T_monica_base_in_ross_base[0, 3])

        else:
            return T_sd
        
        # adjust the desired transformation matrix
        T_sy[:3, :3] = np.array(
            R.from_euler('z', theta, degrees=False).as_matrix())

        return T_sy @ T_yd


    def update_desired_pose_cb(self, msg: Pose):
        """
        This function will update 'self.desired_pose' each time a new
        pose is published to the topic 'px100_target_pose'.
        """
        # if the pose is the same, just return
        if (self.desired_pose.position == msg.position
                and self.desired_pose.orientation == msg.orientation):
            return

        # use a lock to update self.desired_pose
        with self.lock:
            self.desired_pose = copy.deepcopy(msg)


    def update_monica_base_link_pose_cb(self, msg: Pose):
        self.monica_base_in_odom_trans_matrix[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        self.monica_base_in_odom_trans_matrix[0, 3] = msg.position.x
        self.monica_base_in_odom_trans_matrix[1, 3] = msg.position.y
        self.monica_base_in_odom_trans_matrix[2, 3] = msg.position.z

    def update_ross_base_link_pose_cb(self, msg: Pose):
        self.ross_base_in_odom_trans_matrix[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        self.ross_base_in_odom_trans_matrix[0, 3] = msg.position.x
        self.ross_base_in_odom_trans_matrix[1, 3] = msg.position.y
        self.ross_base_in_odom_trans_matrix[2, 3] = msg.position.z

    def log_info(self, msg):
        self.get_logger().info(f'{msg}')

    def log_debug(self, msg):
        self.get_logger().debug(f'{msg}')

    

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(num_threads=3)
    arm_controller = ArmController()

    executor.add_node(arm_controller)
    executor.spin()

    arm_controller.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
