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
from arm_controller.msg import ConstrainedPose


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
        # Measures the rotation of the ee w.r.t. the base frame's z-axis (yaw angle)
        self.T_sy = np.identity(4)
        # Measures the orientation and position of the ee w.r.t. the base frame
        # (not including the orientation around the z-axis)
        self.T_yb = np.identity(4)
        self.update_T_yb()
        self.core.get_node().create_subscription(
            ArmJoy,
            'commands/joy_processed',
            self.joy_control_cb,
            10,
        )
        self.core.get_node().create_subscription(
            ConstrainedPose,
            '/joy_target_pose',
            self.go_to,
            1
        )
        time.sleep(0.5)
        self.core.get_node().loginfo('Ready to receive processed joystick commands.')

    def start_robot(self) -> None:
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
            # elif (msg.pose_cmd == 80):
                # Used for testing IK solver
                # T_sd = np.array([
                #     [0, -1, 0, 0],
                #     [1, 0, 0, 0.233],
                #     [0, 0, 1, 0.098],
                #     [0, 0, 0, 1]
                # ])
                # _, success = self.arm.set_ee_pose_matrix(
                #     T_sd=T_sd,
                #     custom_guess=[1.7, 0.5, 0.6, -1.17],
                #     execute=True,
                #     moving_time=0.2,
                #     accel_time=0.1,
                #     blocking=False)
                # if success:
                #     self.T_yb = np.array(T_yb)
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
        # self.core.get_node().get_logger().info(f'T_sy:\n{self.T_sy}')
        # self.core.get_node().get_logger().info(f'T_yb:\n{T_yb}')
        # self.core.get_node().get_logger().info(f'T_sd:\n{T_sd}')
        _, success = self.arm.set_ee_pose_matrix(
            T_sd=T_sd,
            custom_guess=self.arm.get_joint_commands(),
            execute=True,
            moving_time=0.2,
            accel_time=0.1,
            blocking=False)
        if success:
            self.T_yb = np.array(T_yb)

    def go_to(self, msg: ConstrainedPose) -> None:
        """
        This method uses iteration to move the arm into a desired pose.
        """
        self.core.get_node().get_logger().info(f'Msg: {msg}')
        pose: Pose = msg.pose

        rotation_extract = R.from_quat([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]).as_matrix()
        desired_rpy = ang.rotation_matrix_to_euler_angles(rotation_extract)
        desired_rotation = np.eye(4)
        desired_rotation[:3, :3] = rotation_extract
    
        desired_matrix = np.eye(4)
        desired_matrix[:3, :3] = desired_rotation[:3, :3]
        desired_matrix[0, 3] = pose.position.x
        desired_matrix[1, 3] = pose.position.y
        desired_matrix[2, 3] = pose.position.z
        self.core.get_node().get_logger().info('Moving to goal pose...')
        success = True
        T_yb = np.array(self.T_yb)
        T_sd = np.eye(4)
        tol = 0.05

        while not np.allclose(a=T_sd, b=desired_matrix, atol=tol) and success:
            self.rotate_waist(desired_rpy[2])
            T_yb = self.translation(
                T_yb,
                # pass coordinates relative to the base frame without yaw angle
                np.linalg.inv(desired_rotation) @ desired_matrix
            )
            T_yb = self.rotate_ee(T_yb, desired_rpy)
            
            T_sd = self.T_sy @ T_yb
            # self.core.get_node().get_logger().info(f'T_d\n{desired_matrix}')
            # self.core.get_node().get_logger().info(f'T_sd\n{T_sd}')
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=T_sd,
                custom_guess=self.arm.get_joint_commands(),
                execute=True,
                moving_time=1.5,
                accel_time=0.75,
                blocking=False)
            if success:
                self.T_yb = np.array(T_yb)
            else:
                self.core.get_node().get_logger().info('Failed to move to goal pose.')


    def rotate_waist(self, desired):
        waist_position: float = self.arm.get_single_joint_command('waist')
        error = desired - waist_position
        if error > 0:
            waist_position += self.waist_step
        elif error < 0:
            waist_position -= self.waist_step

        success = self.arm.set_single_joint_position(
            joint_name='waist',
            position=waist_position,
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
        
        self.update_T_yb()

    def translation(self, T, T_d) -> np.ndarray:
        error1 = T_d[0, 3]- T[0, 3]
        step1 = error1**2
        error2 = T_d[2, 3] - T[2, 3]
        step2 = error2**2
        if error1 > 0:
            T[0, 3] += self.translate_step
        elif error1 < 0:
            T[0, 3] -= self.translate_step
        if error2 > 0:
            T[2, 3] += self.translate_step
        elif error2 < 0:
            T[2, 3] -= self.translate_step
        return T

    def rotate_ee(self, T, desired_rpy) -> np.ndarray:
        rpy = ang.rotation_matrix_to_euler_angles(T)
        error = desired_rpy[1] - rpy[1]
        if error > 0:
            rpy[1] += self.rotate_step
        elif error < 0:
            rpy[1] -= self.rotate_step
        T[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
        return T

    def move_forward(self):
        """Moves the arm a small distance forward"""
        T_yb = np.array(self.T_yb)
        translate_step = 1e-2
        T_yb[0, 3] += translate_step
        # Get desired transformation matrix of the end-effector w.r.t. the world frame
        T_sd = np.dot(self.T_sy, T_yb)
        _, success = self.arm.set_ee_pose_matrix(
            T_sd=T_sd,
            custom_guess=self.arm.get_joint_commands(),
            execute=True,
            moving_time=1.5,
            accel_time=0.75,
            blocking=True)
        if success:
            self.T_yb = np.array(T_yb)

    def lift(self):
        """
        This method will be used to get the robots to lift
        an object.
        """
        T_yb = np.array(self.T_yb)
        max_z_pos = 0
        while T_yb[2, 3] != max_z_pos:
            translate_step = 1e-2
            T_yb[2, 3] += translate_step
            # Get desired transformation matrix of the end-effector w.r.t. the world frame
            T_sd = np.dot(self.T_sy, T_yb)
            _, success = self.arm.set_ee_pose_matrix(
                T_sd=T_sd,
                custom_guess=self.arm.get_joint_commands(),
                execute=True,
                moving_time=1.5,
                accel_time=0.75,
                blocking=True)
            if success:
                self.T_yb = np.array(T_yb)
        

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
