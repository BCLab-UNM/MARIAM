#!/usr/bin/env python3

import argparse
import sys
import time
import copy
from threading import Lock

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Float64

class ArmController(InterbotixManipulatorXS):
    """
    This class is a position controller for the Interbotix PX100.
    """
    # the rate at which the while loop in 'start_robot()' will run
    current_loop_rate = 500
    # the amount of time to spend moving to the desired position
    moving_time = 0.2
    # the amount of time to spend accelerating/decelerating
    accel_time = 0.1
    # used to lock the desired_pose field to avoid race conditions
    lock = Lock()
    # the pose we want the robot to be in
    desired_pose = Pose()

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            moving_time=self.moving_time,
            accel_time=self.accel_time,
            args=args,
        )
        # sets the rate at which the control loop will run
        self.rate = self.core.get_node().create_rate(self.current_loop_rate)

        self.core.get_node().create_subscription(
            Pose,
            'px100_target_pose',
            self.update_desired_pose_cb,
            10
        )

        self.exec_time_pub = self.core.get_node().create_publisher(
            Float64,
            'IK_solver_execution_time',
            10
        )

        self.core.get_node().loginfo(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        self.arm.start_lift(blocking=True)

        try:
            robot_startup()
            while rclpy.ok():
                self.move_end_effector()
                self.rate.sleep()

        except KeyboardInterrupt:
            robot_shutdown()

    def move_end_effector(self) -> None:
        """
        """
        waist_angle_tolerance = 1e-2
        cartesian_pos_tolerance = 8e-3

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

        # get the desired roll, pitch, yaw angles from the rotation matrix
        desired_rpy = ang.rotation_matrix_to_euler_angles(
            desired_rotation_matrix)

        # create a homogeneous transformation matrix from the
        # desired pose and the rotation matrix
        desired_trans_matrix = np.eye(4)
        desired_trans_matrix[:3, :3] = desired_rotation_matrix[:3, :3]
        desired_trans_matrix[0, 3] = desired_pose.position.x
        desired_trans_matrix[1, 3] = desired_pose.position.y
        desired_trans_matrix[2, 3] = desired_pose.position.z

        trans_matrix = self.arm.get_ee_pose_command()

        ######### adjust waist angle #########
        waist_position = self.arm.get_single_joint_command('waist')
        if abs(desired_rpy[2] - waist_position) > waist_angle_tolerance:
            self.arm.set_single_joint_position(
                joint_name='waist',
                position=desired_rpy[2],
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                blocking=False
            )

        # computing the cartesian position error
        position_error = np.abs(desired_trans_matrix[:3, 3] - trans_matrix[:3, 3])

        if (np.any(position_error) > cartesian_pos_tolerance):
            start = self.core.get_node().get_clock().now()
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=desired_trans_matrix,
                # custom_guess=self.arm.get_joint_commands(),
                execute=True,
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                blocking=False
            )
            end = self.core.get_node().get_clock().now()

            if success:
                exec_time_msg = Float64()
                exec_time_msg.data = (end-start).nanoseconds / 1e9
                self.exec_time_pub.publish(exec_time_msg)


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
        self.core.get_node().get_logger().info(f'{msg}')


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = ArmController(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
