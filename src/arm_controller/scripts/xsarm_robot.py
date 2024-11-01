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


class XSArmRobot(InterbotixManipulatorXS):
    current_loop_rate = 40
    waist_step        = 0.05
    translate_step    = 0.001
    ee_pitch_step     = 0.004
    moving_time       = 0.2
    accel_time        = 0.001
    lock = Lock()

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            moving_time=self.moving_time,
            accel_time=self.accel_time,
            args=args,
        )
        self.desired_pose = Pose()
        self.rate = self.core.get_node().create_rate(self.current_loop_rate)
        
        self.waist_index = self.arm.group_info.joint_names.index('waist')
        self.waist_ll = self.arm.group_info.joint_lower_limits[self.waist_index]
        self.waist_ul = self.arm.group_info.joint_upper_limits[self.waist_index]
        # The transformation matrix between the space frame and the virtual frame
        self.T_sy = np.identity(4)
        # The transformation matrix between the virtual frame and the body frame
        self.T_yb = np.identity(4)
        self.update_T_yb()
        self.core.get_node().create_subscription(
            Pose,
            'px100_target_pose',
            self.update_desired_pose_cb,
            10
        )
        time.sleep(0.5)
        self.core.get_node().loginfo(f'Robot name: {pargs.robot_name}')

    def start_robot(self) -> None:
        self.arm.go_to_home_pose(
            moving_time=1.5,
            accel_time=0.75
        )
        self.update_T_yb()
        try:
            robot_startup()
            while rclpy.ok():
                self.control_loop()
                self.rate.sleep()
        except KeyboardInterrupt:
            robot_shutdown()

    def update_T_yb(self) -> None:
        """
        Calculate the pose of the end-effector w.r.t. the virtual frame.
        """
        # Get the latest command
        T_sb = self.arm.get_ee_pose_command()
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        # update yaw angle
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])
        # Updates the end-effectors position relative to base frame
        self.T_yb = ang.trans_inv(self.T_sy) @ T_sb


    def control_loop(self) -> None:
        """
        Moves the end-effector towards the desired pose.

        msg: the desired pose.
        """
        with self.lock:
            goto_pose = copy.deepcopy(self.desired_pose)
        # start_time = self.core.get_node().get_clock().now()
        waist_tol   = 8e-2
        ee_tol      = 8e-3
        set_ee_pose = False
        
        # convert pose into the desired transformation matrix
        desired_rotation = np.eye(4)
        desired_rotation[:3, :3] = R.from_quat([ 
            goto_pose.orientation.x,
            goto_pose.orientation.y,
            goto_pose.orientation.z,
            goto_pose.orientation.w,
        ]).as_matrix()

        desired_rpy = ang.rotation_matrix_to_euler_angles(desired_rotation)
    
        desired_matrix = np.eye(4)
        desired_matrix[:3, :3] = desired_rotation[:3, :3]
        desired_matrix[0, 3] = goto_pose.position.x
        desired_matrix[1, 3] = goto_pose.position.y
        desired_matrix[2, 3] = goto_pose.position.z
        # create a copy of the current end-effector pose
        T_yb = np.array(self.T_yb)
        # compute the transformation matrix for the desired end-effector pose
        T_yd = np.linalg.inv(desired_rotation) @ desired_matrix
        # get the current rpy angle
        rpy = ang.rotation_matrix_to_euler_angles(T_yb)
        waist_position = self.arm.get_single_joint_command('waist')

        # error calculations
        waist_error = desired_rpy[2] - waist_position
        x_pos_error = T_yd[0, 3] - T_yb[0, 3]
        z_pos_error = T_yd[2, 3] - T_yb[2, 3]
        ee_pitch_error = desired_rpy[1] - rpy[1]

        if abs(waist_error) > waist_tol:
            waist_position += self.sign(waist_error) * self.waist_step
            success_waist = self.arm.set_single_joint_position(
                joint_name='waist',
                position=waist_position,
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                blocking=False
            )
            if (not success_waist and waist_position != self.waist_ul):
                self.arm.set_single_joint_position(
                    joint_name='waist',
                    position=self.waist_ul,
                    moving_time=self.moving_time,
                    accel_time=self.accel_time,
                    blocking=False
                )
            self.update_T_yb()
        
        
        if abs(x_pos_error) > ee_tol:
            set_ee_pose = True
            T_yb[0, 3] += self.sign(x_pos_error) * self.translate_step

        if abs(z_pos_error) > ee_tol:
            set_ee_pose = True
            T_yb[2, 3] += self.sign(z_pos_error) * self.translate_step

        if abs(ee_pitch_error) > ee_tol:
            set_ee_pose = True
            rpy[1] += self.sign(ee_pitch_error) * self.ee_pitch_step
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
        
        if set_ee_pose:
            T_sd = self.T_sy @ T_yb
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=T_sd,
                custom_guess=self.arm.get_joint_commands(),    
                execute=True,
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                blocking=False
            )
            if success:
                self.T_yb = np.array(T_yb)
            # else:
                # self.log_info('Failed to move to goal pose.')
                
        # end_time = self.core.get_node().get_clock().now()
        # self.core.get_node().loginfo(f'Time in control loop callback:\n{(end_time-start_time).nanoseconds / 1e9}')


    def update_desired_pose_cb(self, msg: Pose):
        with self.lock:
            self.desired_pose = copy.deepcopy(msg)

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0

    def log_info(self, msg):
        self.core.get_node().get_logger().info(f'{msg}')

def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = XSArmRobot(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
