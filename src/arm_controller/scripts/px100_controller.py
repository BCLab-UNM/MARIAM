#!/usr/bin/env python3

import argparse
import sys
import time
import copy
from threading import Lock

from interbotix_common_modules.common_robot.robot import (
    robot_shutdown, robot_startup
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.srv import RegisterValues
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Pose, Point, Quaternion
from interbotix_xs_msgs.msg import JointGroupCommand
from scipy.spatial.transform import Rotation as R


class ArmController(InterbotixManipulatorXS):
    """
    This class is a position controller for the Interbotix PX100.
    """
    # the rate at which the while loop in 'start_robot()' will run
    loop_rate = 500
    # the amount of time to spend moving to the desired position
    moving_time = 0.0
    # the amount of time to spend accelerating/decelerating
    accel_time = 0.0
    # used to lock the desired_pose field to avoid race conditions
    lock = Lock()
    # the pose we want the robot to be in
    # Let the default be where start_lift() will place the robot
    desired_pose = Pose(
        position=Point(x=0.0, y=0.230, z=0.067),
        orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
    )

    # NOTE: it's hard to determine these poses before hand
    # so we will not adjust the heading until they are updated
    # by the callbacks
    # pose of monica's base relative to world frame
    T_world_to_mm = np.eye(4)
    # pose of ross' base relative to world frame
    T_world_to_rm = np.eye(4)
    # transform from vicon to manipulator frame
    # this is the same for both robots
    T_v_to_m = np.array([ [ 0, 1, 0,  0.23],
                          [-1, 0, 0, -0.07],
                          [ 0, 0, 1, -0.06],
                          [ 0, 0, 0,  1]])

    current_theta = np.pi/2 # Default to no heading adjustment

    # offset to apply so the end effector is pitched up by theta degrees
    pitch_offset = np.array(R.from_euler('y', -15, degrees=True).as_matrix())

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
        self.rate = self.core.get_node().create_rate(self.loop_rate)
        # set the robot name
        self.robot_name = pargs.robot_name
        self.track_other_robot = (
            True if pargs.track_other_robot == 'true' else False)
        self.updated_ross_pose = False
        self.updated_monica_pose = False

        self.joint_group_pub = self.core.get_node().create_publisher(
            JointGroupCommand,
            'commands/joint_group',
            10
        )

        self.core.get_node().create_subscription(
            Pose,
            'px100_target_pose',
            self.update_desired_pose_cb,
            10
        )

        if self.track_other_robot:
            # subscribe to the base link pose publisher for both robots
            # This could help make calculations a bit easier
            self.core.get_node().create_subscription(
                Pose,
                '/world_monica_pose',
                self.update_monica_vicon_pose_cb,
                10
            )

            self.core.get_node().create_subscription(
                Pose,
                '/world_ross_pose',
                self.update_ross_vicon_pose_cb,
                10
            )

        self.core.get_node().loginfo(f'Robot name: {self.robot_name}')

    def start_robot(self) -> None:
        self.arm.start_lift(
            moving_time=1.5,
            accel_time=0.5,
            blocking=True
        )

        # set the velocity and acceleration profiles to 0
        self.set_motor_registers(register='Profile_Velocity', value=0)
        self.set_motor_registers(register='Profile_Acceleration', value=0)

        try:
            robot_startup()
            while rclpy.ok():
                self.move_end_effector()
                self.rate.sleep()

        except KeyboardInterrupt:
            robot_shutdown()

    def move_end_effector(self) -> None:
        cartesian_pos_tolerance = 1e-3
        orientation_tolerance = 1e-3

        with self.lock:
            desired_pose = self.desired_pose

        # create a rotation matrix from the desired pose's quaternion
        desired_rotation_matrix = np.array(R.from_quat([
            desired_pose.orientation.x,
            desired_pose.orientation.y,
            desired_pose.orientation.z,
            desired_pose.orientation.w,
        ]).as_matrix())

        # apply a small offset to the orientation
        desired_rotation_matrix = desired_rotation_matrix @ self.pitch_offset

        # create a homogeneous transformation matrix from the
        # desired pose and the rotation matrix
        desired_trans_matrix = np.eye(4)
        desired_trans_matrix[:3, :3] = desired_rotation_matrix[:3, :3]
        desired_trans_matrix[0, 3] = desired_pose.position.x
        desired_trans_matrix[1, 3] = desired_pose.position.y
        desired_trans_matrix[2, 3] = desired_pose.position.z

        if self.track_other_robot and self.updated_monica_pose and self.updated_ross_pose:
            desired_trans_matrix = self.adjust_heading(desired_trans_matrix)

        trans_matrix = self.arm.get_ee_pose_command()

        # computing the cartesian position error
        position_error = np.abs(
            desired_trans_matrix[:3, 3] - trans_matrix[:3, 3])

        # Orientation error (compute using quaternions or matrices)
        desired_rot = R.from_matrix(desired_trans_matrix[:3, :3])
        current_rot = R.from_matrix(trans_matrix[:3, :3])
        rot_error = desired_rot.inv() * current_rot
        angle_error = rot_error.magnitude()  # radians

        if (np.any(position_error) > cartesian_pos_tolerance) or (angle_error > orientation_tolerance):
            joint_cmds, success = self.arm.set_ee_pose_matrix(
                T_sd=desired_trans_matrix,
                custom_guess=self.arm.get_joint_commands(),
                execute=False,
                blocking=False
            )

            if success:
                # print the first value of the joint commands
                self.log_debug(f'Waist joint commands: {joint_cmds[0]}')

                # Safety check to make sure waist in operating range (0,pi)
                if joint_cmds[0] < 0 or joint_cmds[0] > np.pi:
                    self.log_info(
                        'Waist joint command out of bounds, skipping')
                    return

                msg = JointGroupCommand(name="arm", cmd=joint_cmds)
                self.joint_group_pub.publish(msg)

            else:
                self.log_info('Failed to solve for target pose')

    def adjust_heading(self, T_sd):
        """
        Update the waist angle to account for the other robot's heading.
        @param T_sd: The desired transformation matrix which needs to be
        adjusted.
        """
        self.log_debug(f'Adjusting heading for {self.robot_name}')
        # compute the desired position of the end effector
        # within a virtual frame that removes the yaw angle
        yaw_angle = np.arctan2(T_sd[1, 3], T_sd[0, 3])
        T_sy = np.eye(4)
        T_sy[:3, :3] = np.array(
            R.from_euler('z', yaw_angle, degrees=False).as_matrix())
        T_yd = np.linalg.inv(T_sy) @ T_sd

        if self.robot_name == 'monica':
            # compute ross' base relative to monica's base
            T_rb_in_mb = np.linalg.inv(self.T_world_to_mm) @ self.T_world_to_rm

            # extract the angle from the transformation matrix
            self.current_theta = np.arctan2(T_rb_in_mb[1, 3], T_rb_in_mb[0, 3])

        elif self.robot_name == 'ross':
            T_rb_in_mb = np.linalg.inv(self.T_world_to_rm) @ self.T_world_to_mm

            # extract the angle from the transformation matrix
            self.current_theta = np.arctan2(T_rb_in_mb[1, 3], T_rb_in_mb[0, 3])
        else:
            return T_sd

        # adjust the desired transformation matrix
        T_sy[:3, :3] = np.array(
            R.from_euler('z', self.current_theta, degrees=False).as_matrix())
        return T_sy @ T_yd

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

    def update_monica_vicon_pose_cb(self, msg: Pose):
        # Get world to vicon transform
        self.T_world_to_mm[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        self.T_world_to_mm[0, 3] = msg.position.x
        self.T_world_to_mm[1, 3] = msg.position.y
        self.T_world_to_mm[2, 3] = msg.position.z

        # calculate world to manipulator transform
        self.T_world_to_mm = self.T_world_to_mm @ self.T_v_to_m
        self.updated_monica_pose = True

    def update_ross_vicon_pose_cb(self, msg: Pose):
        # Get world to vicon transform
        self.T_world_to_rm[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        self.T_world_to_rm[0, 3] = msg.position.x
        self.T_world_to_rm[1, 3] = msg.position.y
        self.T_world_to_rm[2, 3] = msg.position.z

        # calculate world to manipulator transform
        self.T_world_to_rm = self.T_world_to_rm @ self.T_v_to_m
        self.updated_ross_pose = True

    def log_info(self, msg):
        self.core.get_node().get_logger().info(f'{msg}')

    def log_debug(self, msg):
        self.core.get_node().get_logger().debug(f'{msg}')

    def set_motor_registers(self, register, value):
        future = self.arm.core.srv_set_reg.call_async(
            RegisterValues.Request(
                cmd_type='group',
                name=self.arm.group_name,
                reg=register,
                value=value,
            )
        )
        self.arm.core.get_node().wait_until_future_complete(future)


def main(args=None):
    p = argparse.ArgumentParser()
    p.add_argument('--robot_model')
    p.add_argument('--robot_name', default=None)
    p.add_argument('--track_other_robot', default='false')
    p.add_argument('args', nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = ArmController(ros_args, args=args)
    bot.start_robot()


if __name__ == '__main__':
    main()
