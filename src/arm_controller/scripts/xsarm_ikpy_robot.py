#!/usr/bin/env python3

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
import ikpy.chain
import os


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

        urdf_file_path = os.path.join(os.getcwd(), 'px100.urdf')
        if os.path.isfile(urdf_file_path):
            self.arm_chain = ikpy.chain.Chain.from_urdf_file(
                urdf_file='/home/xavier/projects/MARIAM/px100.urdf',
                base_elements=['px100/base_link'],
                active_links_mask=[False, True,
                                   True, True, True, False, False],
                symbolic=True
            )
        self.log_info(f'Arm chain: {self.arm_chain}')
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
        tol = 0.001
        position = [
            msg.position.x,
            msg.position.y,
            msg.position.z,
        ]

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
        
        start_time = self.core.get_node().get_clock().now()
        joint_angles = self.arm_chain.inverse_kinematics(
            target_position=position,
            target_orientation=desired_rpy,
            orientation_mode="Z"
        )
        joint_angles = joint_angles[1:5]
        self.arm.set_joint_positions(
            joint_positions=joint_angles,
            moving_time=1.7,
            accel_time=0.15,
            blocking=False
        )
        end_time = self.core.get_node().get_clock().now()
        self.core.get_node().get_logger().info(
            f'Time: {(end_time-start_time).nanoseconds / 1e9}'
        )

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0
        
    def log_info(self, msg):
        self.core.get_node().get_logger().info(msg)


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
