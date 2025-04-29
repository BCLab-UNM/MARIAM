#!/usr/bin/env python3

import argparse
import sys

import copy
from threading import Lock

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


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

    joint_cmds = [0.0, 0.0, 0.0, 0.0]

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

    joint_names = ['waist', 'shoulder', 'elbow', 'wrist_angle']

    duration = Duration()


    def __init__(self, pargs, args=None):
        super().__init__('px100_controller')
        # the rate at which the while loop in 'start_robot()' will run
        loop_rate = 500
        # sets the rate at which the control loop will run
        self.rate = self.create_rate(loop_rate)

        self.duration.nanosec = 2_000_000_000

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
        self.joint_trajectory_pub.publish(
            JointTrajectory(
                joint_names=self.joint_names,
                points=[JointTrajectoryPoint(
                    positions=[1.5, 0.52447963, 0.67205733, -1.17653696],
                    # in nanoseconds
                    time_from_start=self.duration
                )]
            )
        )

        while rclpy.ok():
            self.move_end_effector()
            self.rate.sleep()


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

        trans_matrix = self.arm.get_ee_pose_command()

        # computing the cartesian position error
        position_error = np.abs(
            desired_trans_matrix[:3, 3] - trans_matrix[:3, 3])

        if (np.any(position_error) > cartesian_pos_tolerance):
            joint_cmds, success = IKinSpace(
                Slist=self.Slist,
                M=self.M,
                T=desired_trans_matrix,
                thetalist0=self.joint_cmds,
            )

            if success:
                trajectory_point = JointTrajectoryPoint(
                    positions=joint_cmds,
                    time_from_start=self.duration
                )
                
                trajectory_msg = JointTrajectory()
                trajectory_msg.joint_names = self.joint_names
                trajectory_msg.points = trajectory_point
                trajectory_msg.header.stamp = self.get_clock().now().to_msg()

                self.joint_group_pub.publish(trajectory_msg)
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
    # rclpy.spin(arm_controller)
    arm_controller.start_robot()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
