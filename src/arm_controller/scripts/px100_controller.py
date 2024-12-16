#!/usr/bin/env python3

import argparse
import sys
import time
import copy
from threading import Lock, get_ident

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
from math import sqrt, pi

##### imports for the profiler ######
# import cProfile
# import pstats
# from io import StringIO


class XSArmRobot(InterbotixManipulatorXS):
    current_loop_rate = 250
    moving_time       = 0.2
    accel_time        = 0.1
    lock = Lock()
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

        ###### Enabling the profiler ######
        # profiler = cProfile.Profile()
        # profiler.enable()

        try:
            robot_startup()
            while rclpy.ok():
                # self.log_info(f'Thread ID (while loop): {get_ident()}')
                self.control_loop()
                self.rate.sleep()
        except KeyboardInterrupt:
            ###### Disabling the profiler ######
            # profiler.disable()
            ###### save the results from the profiler ######
            # s = StringIO()
            # ps = pstats.Stats(profiler, stream=s).sort_stats('cumulative')
            # ps.dump_stats('px100_controller.py-2.profile.stats')
            robot_shutdown()


    def update_T_yb(self) -> None:
        """
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
        # create a copy of the desired pose using a lock to avoid race conditions
        # start_time = self.core.get_node().get_clock().now()
        # self.log_info(f'Thread ID (control loop): {get_ident()}')
        with self.lock:
            desired_pose = copy.deepcopy(self.desired_pose)
        waist_step = 0.008
        translate_step = 0.001
        ee_pitch_step = 0.004
        waist_tol   = 8e-3
        ee_tol      = 8e-3
        set_ee_pose = False # determines if the IK solver will be called
        
        # this line creates a rotation matrix form the desired pose's quaterion
        desired_rotation = np.eye(4)
        desired_rotation[:3, :3] = R.from_quat([ 
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix()

        # get the desired roll, pitch, yaw angles from the rotation matrix
        desired_rpy = ang.rotation_matrix_to_euler_angles(desired_rotation)
    
        # create a transformation matrix from the desired pose
        desired_matrix = np.eye(4)
        desired_matrix[:3, :3] = desired_rotation[:3, :3]
        desired_matrix[0, 3] = desired_pose.position.x
        desired_matrix[1, 3] = desired_pose.position.y
        desired_matrix[2, 3] = desired_pose.position.z
        
        # create a copy of the current end-effector pose wrt virtual frame
        T_yb = np.array(self.T_yb)
        # compute the transformation matrix for the desired end-effector pose
        # wrt the virtual frame
        T_yd = np.linalg.inv(desired_rotation) @ desired_matrix
        # get the current rpy angle
        rpy = ang.rotation_matrix_to_euler_angles(T_yb)
        # get the current angle of the waist
        waist_position = self.arm.get_single_joint_command('waist')

        # error calculations
        waist_error = desired_rpy[2] - waist_position
        x_pos_error = T_yd[0, 3] - T_yb[0, 3]
        z_pos_error = T_yd[2, 3] - T_yb[2, 3]
        ee_pitch_error = desired_rpy[1] - rpy[1]

        # if the waist error is above some tolerance, move the waist by
        # a single step (self.wasit_step)
        if abs(waist_error) > waist_tol:
            waist_position += self.sign(waist_error) * waist_step
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
        
        
        """
        For each of the three if statements, if the error is greater than the
        tolerance, then increment the relative position and make 'set_ee_pose'
        true.
        """
        if abs(x_pos_error) > ee_tol:
            set_ee_pose = True
            T_yb[0, 3] += self.sign(x_pos_error) * translate_step

        if abs(z_pos_error) > ee_tol:
            set_ee_pose = True
            T_yb[2, 3] += self.sign(z_pos_error) * translate_step

        if abs(ee_pitch_error) > ee_tol:
            set_ee_pose = True
            rpy[1] += self.sign(ee_pitch_error) * ee_pitch_step
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
        
        # if 'set_ee_pose' is true, then the IK solver will be called
        # and the result of the IK solver is published to '/robot_name/commands/joint_group'

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


    def update_desired_pose_cb(self, msg: Pose):
        # self.log_info(f'Thread ID (desired pose cb): {get_ident()}')
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
