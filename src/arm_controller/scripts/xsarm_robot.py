#!/usr/bin/env python3

# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import copy
import sys
from threading import Lock
import time

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import ArmJoy
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R


class XSArmRobot(InterbotixManipulatorXS):
    """
    Processes incoming ArmJoy messages and outputs robot commands.

    The XSArmRobot class is responsible for reading in ArmJoy messages and sending
    joint and gripper commands to the xs_sdk node; while the `waist` joint can be
    directly controlled via the PS3/PS4 joystick, other buttons allow position-ik to be
    performed using all the arm joints.
    """

    waist_step = 0.06
    rotate_step = 0.04
    translate_step = 0.01
    current_loop_rate = 25
    current_torque_status = True
    loop_rates = {'coarse': 25, 'fine': 25}
    joy_msg = ArmJoy()
    joy_mutex = Lock()

    def __init__(self, pargs, args=None):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=pargs.robot_model,
            robot_name=pargs.robot_name,
            moving_time=0.2,
            accel_time=0.1,
            args=args,
        )
        self.rate = self.core.get_node().create_rate(self.current_loop_rate)
        self.num_joints = self.arm.group_info.num_joints
        self.waist_index = self.arm.group_info.joint_names.index('waist')
        self.waist_ll = self.arm.group_info.joint_lower_limits[self.waist_index]
        self.waist_ul = self.arm.group_info.joint_upper_limits[self.waist_index]
        # The transformation matrix between the space frame and the virtual frame
        self.T_sy = np.identity(4)
        # The transformation matrix between the virtual frame and the body frame
        self.T_yb = np.identity(4)
        self.update_T_yb()
        self.core.get_node().create_subscription(
            ArmJoy,
            'commands/joy_processed',
            self.joy_control_cb,
            10,
        )
        self.core.get_node().create_subscription(
            Pose,
            '/high_freq_target_pose',
            self.control_loop_cb,
            10
        )
        time.sleep(0.5)
        self.core.get_node().loginfo('Ready to receive processed joystick commands.')

    def start_robot(self) -> None:
        self.arm.go_to_home_pose(
            moving_time=1.5,
            accel_time=0.75
        )
        self.update_T_yb()
        try:
            robot_startup()
            while rclpy.ok():
                self.controller()
                self.rate.sleep()
        except KeyboardInterrupt:
            robot_shutdown()

    def update_speed(self, loop_rate: float) -> None:
        """
        Update the frequency at which the main control loop runs.

        :param loop_rate: desired loop frequency [Hz]
        """
        self.current_loop_rate = loop_rate
        self.rate = self.core.get_node().create_rate(self.current_loop_rate)
        self.core.get_node().loginfo(
            f'Current loop rate is {self.current_loop_rate} Hz.')

    def update_T_yb(self) -> None:
        """
        Calculate the pose of the end-effector w.r.t. T_y.
        Specifically updates T_sy to be the yaw angle of the end-effector's
        current position, then updates T_yb to reflect the end-effector's
        position relative to the base frame.
        """
        # Get the latest command
        T_sb = self.arm.get_ee_pose_command()
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        # update yaw angle
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])
        # Updates the end-effectors position relative to base frame
        self.T_yb = ang.trans_inv(self.T_sy) @ T_sb

    def joy_control_cb(self, msg: ArmJoy) -> None:
        """
        Process ArmJoy messages from ROS Subscription callback.

        :param msg: ArmJoy ROS message
        """
        with self.joy_mutex:
            self.joy_msg = copy.deepcopy(msg)

        # Check the speed_cmd
        if (msg.speed_cmd == ArmJoy.SPEED_INC and self.current_loop_rate < 40):
            self.update_speed(loop_rate=self.current_loop_rate + 1)
        elif (msg.speed_cmd == ArmJoy.SPEED_DEC and self.current_loop_rate > 10):
            self.update_speed(loop_rate=self.current_loop_rate - 1)

        # Check the speed_toggle_cmd
        if (msg.speed_toggle_cmd == ArmJoy.SPEED_COARSE):
            self.loop_rates['fine'] = self.current_loop_rate
            self.core.get_node().loginfo('Switched to Coarse Control')
            self.update_speed(loop_rate=self.loop_rates['coarse'])
        elif (msg.speed_toggle_cmd == ArmJoy.SPEED_FINE):
            self.loop_rates['coarse'] = self.current_loop_rate
            self.core.get_node().loginfo('Switched to Fine Control')
            self.update_speed(loop_rate=self.loop_rates['fine'])

        # Check the torque_cmd
        if (msg.torque_cmd == ArmJoy.TORQUE_ON):
            self.core.robot_torque_enable(
                cmd_type='group', name='arm', enable=True)
            self.arm.capture_joint_positions()
            self.update_T_yb()
            self.current_torque_status = True
        elif (msg.torque_cmd == ArmJoy.TORQUE_OFF):
            self.core.robot_torque_enable(
                cmd_type='group', name='arm', enable=False)
            self.current_torque_status = False

    def controller(self) -> None:
        """Run main arm manipulation control loop."""
        if not self.current_torque_status:
            return

        with self.joy_mutex:
            msg = copy.deepcopy(self.joy_msg)

        # Check the pose_cmd
        if (msg.pose_cmd != 0):
            if (msg.pose_cmd == ArmJoy.HOME_POSE):
                self.arm.go_to_home_pose(moving_time=1.5, accel_time=0.75)
            elif (msg.pose_cmd == ArmJoy.SLEEP_POSE):
                self.arm.go_to_sleep_pose(moving_time=1.5, accel_time=0.75)
            elif (msg.pose_cmd == ArmJoy.START_LIFT):
                self.arm.start_lift(moving_time=1.5, accel_time=0.75)
            elif (msg.pose_cmd == ArmJoy.LIFT):
                self.arm.lift(moving_time=1.5, accel_time=0.75)
            self.update_T_yb()
            self.arm.set_trajectory_time(moving_time=0.2, accel_time=0.1)

        # Check the waist_cmd
        if (msg.waist_cmd != 0):
            waist_position = self.arm.get_single_joint_command('waist')
            if (msg.waist_cmd == ArmJoy.WAIST_CCW):
                success = self.arm.set_single_joint_position(
                    joint_name='waist',
                    position=waist_position + self.waist_step,
                    moving_time=0.2,
                    accel_time=0.1,
                    blocking=False)
                if (not success and waist_position != self.waist_ul):
                    self.arm.set_single_joint_position(
                        joint_name='waist',
                        position=self.waist_ul,
                        moving_time=0.2,
                        accel_time=0.1,
                        blocking=False)
            elif (msg.waist_cmd == ArmJoy.WAIST_CW):
                success = self.arm.set_single_joint_position(
                    joint_name='waist',
                    position=waist_position - self.waist_step,
                    moving_time=0.2,
                    accel_time=0.1,
                    blocking=False)
                if (not success and waist_position != self.waist_ll):
                    self.arm.set_single_joint_position(
                        joint_name='waist',
                        position=self.waist_ll,
                        moving_time=0.2,
                        accel_time=0.1,
                        blocking=False)
            self.update_T_yb()

        position_changed = msg.ee_x_cmd + msg.ee_z_cmd
        orientation_changed = msg.ee_roll_cmd + msg.ee_pitch_cmd

        if (position_changed + orientation_changed == 0):
            return

        # Copy the most recent T_yb transform into a temporary variable
        T_yb = np.array(self.T_yb)

        if (position_changed):
            # check ee_x_cmd
            if (msg.ee_x_cmd == ArmJoy.EE_X_INC):
                T_yb[0, 3] += self.translate_step
            elif (msg.ee_x_cmd == ArmJoy.EE_X_DEC):
                T_yb[0, 3] -= self.translate_step

            # check ee_z_cmd
            if (msg.ee_z_cmd == ArmJoy.EE_Z_INC):
                T_yb[2, 3] += self.translate_step
            elif (msg.ee_z_cmd == ArmJoy.EE_Z_DEC):
                T_yb[2, 3] -= self.translate_step

        # check end-effector orientation related commands
        if (orientation_changed != 0):
            rpy = ang.rotation_matrix_to_euler_angles(T_yb[:3, :3])

            # check ee_pitch_cmd
            if (msg.ee_pitch_cmd == ArmJoy.EE_PITCH_DOWN):
                rpy[1] += self.rotate_step
            elif (msg.ee_pitch_cmd == ArmJoy.EE_PITCH_UP):
                rpy[1] -= self.rotate_step
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)

        # Get desired transformation matrix of the end-effector w.r.t. the base frame
        T_sd = self.T_sy @ T_yb
        _, success = self.arm.set_ee_pose_matrix(
            T_sd=T_sd,
            custom_guess=self.arm.get_joint_commands(),
            execute=True,
            moving_time=0.2,
            accel_time=0.1,
            blocking=False)
        if success:
            self.T_yb = np.array(T_yb)

    def control_loop_cb(self, msg: Pose) -> None:
        """
        Slightly moves the end-effector towards the desired pose.

        msg: the desired pose.
        """
        start_time = self.core.get_node().get_clock().now()
        waist_step = 0.005
        translate_step = 0.001
        ee_pitch_step = 0.002
        tol = 8e-3
        ee_tol = 8e-3
        moving_time = 0.8
        accel_time = 0.15
        set_ee_pose = False
        
        # convert pose into desired transformation matrices
        desired_rotation = np.eye(4)
        desired_rotation[:3, :3] = R.from_quat([ 
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()

        desired_rpy = ang.rotation_matrix_to_euler_angles(desired_rotation)
    
        desired_matrix = np.eye(4)
        desired_matrix[:3, :3] = desired_rotation[:3, :3]
        desired_matrix[0, 3] = msg.position.x
        desired_matrix[1, 3] = msg.position.y
        desired_matrix[2, 3] = msg.position.z
        
        # update waist joint angle
        waist_position = self.arm.get_single_joint_command('waist')
        waist_error = desired_rpy[2] - waist_position
        if abs(waist_error) > tol:
            waist_position += self.sign(waist_error) * waist_step
            success_waist = self.arm.set_single_joint_position(
                joint_name='waist',
                position=waist_position,
                moving_time=moving_time,
                accel_time=accel_time,
                blocking=False)
            if (not success_waist and waist_position != self.waist_ul):
                self.arm.set_single_joint_position(
                    joint_name='waist',
                    position=self.waist_ul,
                    moving_time=moving_time,
                    accel_time=accel_time,
                    blocking=False)
            self.update_T_yb()

        # update the position of end-effector w.r.t. base frame
        T_yb = np.array(self.T_yb)
        T_yd = np.linalg.inv(desired_rotation) @ desired_matrix
        x_pos_error = T_yd[0, 3] - T_yb[0, 3]
        z_pos_error = T_yd[2, 3] - T_yb[2, 3]

        if abs(x_pos_error) > tol:
            set_ee_pose = True
            T_yb[0, 3] += self.sign(x_pos_error) * translate_step

        if abs(z_pos_error) > tol:
            set_ee_pose = True
            T_yb[2, 3] += self.sign(z_pos_error) * translate_step

        # update the pitch angle of end-effector w.r.t. base frame
        rpy = ang.rotation_matrix_to_euler_angles(T_yb)
        ee_pitch_error = desired_rpy[1] - rpy[1]
        if abs(ee_pitch_error) > ee_tol:
            set_ee_pose = True
            rpy[1] += self.sign(ee_pitch_error) * ee_pitch_step
            T_yb[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
        
        if set_ee_pose:
            T_sd = self.T_sy @ T_yb
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=T_sd,
                custom_guess=self.arm.get_joint_commands(),
                execute=True,
                moving_time=moving_time,
                accel_time=accel_time,
                blocking=False)
            if success:
                self.T_yb = np.array(T_yb)
            else:
                self.log_info('Failed to move to goal pose.')
        end_time = self.core.get_node().get_clock().now()
        # self.log_info(
        #     f'Time: {(end_time-start_time).nanoseconds / 1e9}'
        # )

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
