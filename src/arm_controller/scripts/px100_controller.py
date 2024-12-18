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


class ArmController(InterbotixManipulatorXS):
    """
    This class is a position controller for the Interbotix PX100.
    """
    current_loop_rate = 500
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
        # sets the rate at which the control loop will run
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
                self.move_end_effector()
                self.rate.sleep()
        except KeyboardInterrupt:
            self.arm.go_to_sleep_pose()
            time.sleep(2.5)
            ###### Disabling the profiler ######
            # profiler.disable()
            ###### save the results from the profiler ######
            # s = StringIO()
            # ps = pstats.Stats(profiler, stream=s).sort_stats('cumulative')
            # ps.dump_stats('px100_controller.py-2.profile.stats')
            robot_shutdown()


    def update_T_yb(self) -> None:
        """
        Helper function that calculates the pose of
        the end effector w.t.t the virtual frame (T_y)
        """
        # Get the latest command
        T_sb = self.arm.get_ee_pose_command()
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        # update yaw angle
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])
        # Updates the end-effectors position relative to base frame
        self.T_yb = ang.trans_inv(self.T_sy) @ T_sb


    def move_end_effector(self) -> None:
        """
        This function moves the end effector towards the desired position
        by reducing the difference between current position and the desired position.
        
        On each iteration, if the difference between the current position and the
        desired position of the end effector is above some tolerance, then position of the
        end effector will be adjusted by some constant value.
        """
        # start_time = self.core.get_node().get_clock().now()
        # self.log_info(f'Thread ID (control loop): {get_ident()}') 

        # create a local copy of the desired pose using a lock to avoid race conditions
        with self.lock:
            desired_pose = self.desired_pose

        
        # creates a rotation matrix from the desired pose's quaterion
        desired_rotation_mat = np.eye(4)
        desired_rotation_mat[:3, :3] = R.from_quat([ 
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix()

        # get the desired roll, pitch, yaw angles from the rotation matrix
        desired_rpy = ang.rotation_matrix_to_euler_angles(desired_rotation_mat)
    
        # create a transformation matrix from the desired pose and the rotation matrix
        desired_trans_mat = np.eye(4)
        desired_trans_mat[:3, :3] = desired_rotation_mat[:3, :3]
        desired_trans_mat[0, 3] = desired_pose.position.x
        desired_trans_mat[1, 3] = desired_pose.position.y
        desired_trans_mat[2, 3] = desired_pose.position.z
        
        # create a copy of the current end-effector position w.r.t. the virtual frame
        T_yb = np.array(self.T_yb)
        # compute the transformation matrix for the desired end-effector position
        # w.r.t. the virtual frame
        T_yd = np.linalg.inv(desired_rotation_mat) @ desired_trans_mat
        # get the current rpy angles
        rpy = ang.rotation_matrix_to_euler_angles(T_yb)

        ######### adjust waist angle #########
        waist_position = self.arm.get_single_joint_command('waist')
        self.adjust_waist_angle(desired_rpy[2], waist_position)

        ######### Compute a new end effector position w.r.t. virtual frame #########
        self.move_end_effector_wrt_virtual_frame(T_yd, T_yb, desired_rpy, rpy)
        


    def adjust_waist_angle(self, desired_waist_position, waist_position):
        """
        This function rotates the waist by a constant angle if the difference
        between the current position and the desired position is large enough.
        """
        # allowed difference between the ee position and the desired position
        waist_angle_tolerance = 8e-3
        # the value to move the waist by
        waist_angle_step = np.pi / 512  # in radians
        
        # compute the difference between the desired angle and the current angle
        error = desired_waist_position - waist_position

        if abs(error) > waist_angle_tolerance:
            waist_position += self.sign(error) * waist_angle_step
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


    def move_end_effector_wrt_virtual_frame(self, T_yd, T_yb, desired_rpy, rpy):
        """
        This function will move the end effector towards a desired position
        w.r.t. the virtual frame.

        The function calculates the error (difference) between the x and y
        position of current position and the desired position. If the difference
        is large enough, a new intermediate distance between the two is calculated
        and the end effector is told to move to this new position.

        The same process is applied to the pitch error.
        """
        translate_step = 0.001  # in meters
        ee_pitch_step = np.pi / 1024  # in radians
        ee_tol = np.pi / 512
        move_end_effector = False

        # position error calculations
        x_pos_error = T_yd[0, 3] - T_yb[0, 3]
        z_pos_error = T_yd[2, 3] - T_yb[2, 3]
        # end effector pitch error
        ee_pitch_error = desired_rpy[1] - rpy[1]

        if abs(x_pos_error) > ee_tol:
            move_end_effector = True
            T_yb[0, 3] += self.sign(x_pos_error) * translate_step

        if abs(z_pos_error) > ee_tol:
            move_end_effector = True
            T_yb[2, 3] += self.sign(z_pos_error) * translate_step

        if abs(ee_pitch_error) > ee_tol:
            move_end_effector = True
            pitch += self.sign(ee_pitch_error) * ee_pitch_step
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)

        # move the end effector if the error is greater than the tolerance
        if move_end_effector:
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
        
        # if the pose isn't different from the current one, just return
        if self.desired_pose.position == msg.position and self.desired_pose.orientation == msg.orientation:
            return

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

    bot = ArmController(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
