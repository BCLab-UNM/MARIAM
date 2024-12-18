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
from scipy.spatial.transform import Rotation as R

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
        self.T_space_to_virtual = np.identity(4)
        
        # The transformation matrix between the virtual frame and the body frame
        self.T_virtual_to_body = np.identity(4)
        
        self.update_T_virtual_to_body()
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
        self.update_T_virtual_to_body()

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


    def update_T_virtual_to_body(self) -> None:
        """
        Helper function that calculates the pose of
        the end effector w.t.t the virtual frame.
        """
        # Get the latest command
        T_space_to_body = self.arm.get_ee_pose_command()
        rpy = ang.rotation_matrix_to_euler_angles(T_space_to_body[:3, :3])
        # update yaw angle
        self.T_space_to_virtual[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])
        # Updates the end-effectors position relative to base frame
        T_space_to_virtual_inv = ang.trans_inv(self.T_space_to_virtual)
        self.T_virtual_to_body = T_space_to_virtual_inv @ T_space_to_body


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

        
        # creates a rotation matrix from the desired pose's quaternion
        desired_rotation_matrix = np.eye(4)
        desired_rotation_matrix[:3, :3] = R.from_quat([ 
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix()

        # get the desired roll, pitch, yaw angles from the rotation matrix
        desired_rpy = ang.rotation_matrix_to_euler_angles(desired_rotation_matrix)
    
        # create a transformation matrix from the desired pose and the rotation matrix
        desired_trans_matrix = np.eye(4)
        desired_trans_matrix[:3, :3] = desired_rotation_matrix[:3, :3]
        desired_trans_matrix[0, 3] = desired_pose.position.x
        desired_trans_matrix[1, 3] = desired_pose.position.y
        desired_trans_matrix[2, 3] = desired_pose.position.z
        
        # create a copy of the current end-effector position w.r.t. the virtual frame
        T_virtual_to_body = np.array(self.T_virtual_to_body)
        # compute the transformation matrix for the desired end-effector position
        # w.r.t. the virtual frame
        desired_T_virtual_to_body = np.linalg.inv(desired_rotation_matrix) @ desired_trans_matrix
        # get the current rpy angles
        rpy = ang.rotation_matrix_to_euler_angles(T_virtual_to_body)

        ######### adjust waist angle #########
        waist_position = self.arm.get_single_joint_command('waist')
        self.rotate_end_effector(desired_rpy[2], waist_position)

        ######### Compute a new end effector position w.r.t. virtual frame #########
        self.move_end_effector_wrt_virtual_frame(
            desired_T_virtual_to_body, 
            T_virtual_to_body,
            desired_rpy,
            rpy
        )
        


    def rotate_end_effector(self, desired_waist_position, waist_position):
        """
        This function rotates the waist by a constant value if the difference
        between the current waist position and the desired waist position is large enough.
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
            self.update_T_virtual_to_body()


    def move_end_effector_wrt_virtual_frame(self, desired_T_virtual_to_body,
                                                  T_virtual_to_body, desired_rpy, rpy):
        """
        This function will move the end effector towards a desired position
        w.r.t. the virtual frame.

        The function calculates the error (difference) between the x and y
        position of current position and the desired position. If the difference
        is large enough, a new intermediate distance between the two is calculated
        and the end effector is told to move to this new position.

        The same process is applied to the pitch angle.
        """
        translate_step = 0.001  # in meters
        ee_pitch_step = np.pi / 1024  # in radians
        position_tolerance = 8e-3
        pitch_tolerance = np.pi / 512
        move_end_effector = False

        # position error calculations
        x_pos_error = desired_T_virtual_to_body[0, 3] - T_virtual_to_body[0, 3]
        z_pos_error = desired_T_virtual_to_body[2, 3] - T_virtual_to_body[2, 3]
        # end effector pitch error
        ee_pitch_error = desired_rpy[1] - rpy[1]

        if abs(x_pos_error) > position_tolerance:
            move_end_effector = True
            T_virtual_to_body[0, 3] += self.sign(x_pos_error) * translate_step

        if abs(z_pos_error) > position_tolerance:
            move_end_effector = True
            T_virtual_to_body[2, 3] += self.sign(z_pos_error) * translate_step

        if abs(ee_pitch_error) > pitch_tolerance:
            move_end_effector = True
            rpy[1] += self.sign(ee_pitch_error) * ee_pitch_step
            T_virtual_to_body[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)

        # move the end effector if the error is greater than the tolerance
        if move_end_effector:
            T_sd = self.T_space_to_virtual @ T_virtual_to_body
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=T_sd,
                custom_guess=self.arm.get_joint_commands(),
                execute=True,
                moving_time=self.moving_time,
                accel_time=self.accel_time,
                blocking=False
            )
            if success:
                self.T_virtual_to_body = np.array(T_virtual_to_body)


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
