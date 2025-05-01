#!/usr/bin/env python3

import argparse
import sys
import copy
from threading import Lock
import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
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

    duration = Duration()

    def __init__(self, pargs, args=None):
        super().__init__('px100_controller')
        # the rate at which the while loop in 'start_robot()' will run
        # (in seconds)
        self.loop_rate = 2

        self.duration.sec = 0
        self.duration.nanosec = 500_000_000

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

        self.log_info(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        self.log_info('Starting robot...')

        try:
            while rclpy.ok():
                self.move_end_effector()
                time.sleep(self.loop_rate)

        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')


    def move_end_effector(self) -> None:
        """
        """
        cartesian_pos_tolerance = 1e-3

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

        # computing the cartesian position error
        position_error = np.abs(
            desired_trans_matrix[:3, 3] - self.last_trans_matrix[:3, 3])

        if (np.any(position_error > cartesian_pos_tolerance)):
            joint_cmds, success = IKinSpace(
                Slist=self.Slist,
                M=self.M,
                T=desired_trans_matrix,
                thetalist0=self.last_joint_cmds,
                eomg=0.001,
                ev=0.001
            )

            self.get_logger().debug(f'Joint commands & success: {type(joint_cmds)}, {success}')

            if success:
                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = [j for j in joint_cmds]
                trajectory_point.time_from_start = self.duration

                trajectory_msg = JointTrajectory()

                trajectory_msg.joint_names = self.joint_names
                trajectory_msg.points = [trajectory_point]
                
                self.get_logger().debug(f'Joint commands: {joint_cmds}')
                self.get_logger().debug(f'Position error: {position_error}')
                self.joint_trajectory_pub.publish(trajectory_msg)

                # self.last_trans_matrix = desired_trans_matrix
                # self.last_joint_cmds = joint_cmds
            else:
                self.log_info('Failed to solve for target pose')

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

    def log_info(self, msg):
        self.get_logger().info(f'{msg}')


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    rclpy.init(args=args)

    arm_controller = ArmController(ros_args, args=args)
    arm_controller.start_robot()

    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
