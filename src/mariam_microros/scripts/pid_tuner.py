#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist
import matplotlib.pyplot as plt
import numpy as np

import os
import math
import time

from threading import Thread

class PIDTuningNode(Node):
    def __init__(self, node_name='pid_tuning_node'):
        super().__init__(node_name)

        # Publishers for PID parameters
        self.pid_pub = self.create_publisher(
            Point,
            '/ross/pid',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/ross/cmd_vel',
            10
        )

        # Subscribers
        self.left_pid_sub = self.create_subscription(
            Float32,
            '/ross/left_pid_output',
            self.left_pid_callback,
            10
        )
        self.right_pid_sub = self.create_subscription(
            Float32,
            '/ross/right_pid_output',
            self.right_pid_callback,
            10
        )
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/ross/joint_states',
            self.joint_states_callback,
            10
        )

        self.left_pid_data = []
        self.right_pid_data = []
        self.left_front_linear_velocity_data = []
        self.right_front_linear_velocity_data = []
        self.left_back_linear_velocity_data = []
        self.right_back_linear_velocity_data = []
        self.joint_states_time_data = []

        # in meters
        self.wheel_radius = 0.3613 / (2.0 * math.pi)

        self.get_logger().info('PID Tuning Node initialized')

    def left_pid_callback(self, msg):
        self.left_pid_data.append(msg.data)

    def right_pid_callback(self, msg):
        self.right_pid_data.append(msg.data)

    def joint_states_callback(self, msg: JointState):           
        # Calculate linear velocity from wheel velocities
        left_front_linear_vel  = msg.velocity[0] * self.wheel_radius
        right_front_linear_vel = msg.velocity[1] * self.wheel_radius
        left_back_linear_vel   = msg.velocity[2] * self.wheel_radius
        right_back_linear_vel  = msg.velocity[3] * self.wheel_radius

        self.left_front_linear_velocity_data.append(left_front_linear_vel)
        self.right_front_linear_velocity_data.append(right_front_linear_vel)
        self.left_back_linear_velocity_data.append(left_back_linear_vel)
        self.right_back_linear_velocity_data.append(right_back_linear_vel)
        self.joint_states_time_data.append(msg.header.stamp.sec)

    def set_pid_params(self, Kp, Ki, Kd):
        msg = Point()
        msg.x = Kp
        msg.y = Ki
        msg.z = Kd
        self.pid_pub.publish(msg)

    def set_cmd_vel(self, msg: Twist):
        self.cmd_vel_pub.publish(msg)

    def plot_graphs(self, desired_vel):
        self.get_logger().info(f'Time data length: {len(self.joint_states_time_data)}')
        self.get_logger().info(f'Left PID data length: {len(self.left_pid_data)}')
        self.get_logger().info(f'Right PID data length: {len(self.right_pid_data)}')
        self.get_logger().info(f'Left front linear velocity data length: {len(self.left_front_linear_velocity_data)}')
        fig, axes = plt.subplots(2, 1, figsize=(16, 8))

        axes[0].plot(
            # for now, let's just interpret this as the PID output
            # near the point in time the wheel's are traveling
            # at a specific speed
            self.joint_states_time_data,
            self.left_pid_data,
            label='Left PID output'
        )
        
        axes[0].plot(
            self.joint_states_time_data,
            self.right_pid_data,
            label='Right PID output'
        )

        axes[0].set_title('PID output for each wheel')
        axes[0].set_xlabel('t')
        axes[0].set_ylabel('PID output (analog)')
        axes[0].legend()
        axes[0].grid(True)

        axes[1].plot(
            self.joint_states_time_data,
            np.full((len(self.joint_states_time_data), ), desired_vel),
            label='Desired Velocity'
        )

        axes[1].plot(
            self.joint_states_time_data,
            self.left_front_linear_velocity_data,
            label='Left-front'
        )
        
        axes[1].plot(
            self.joint_states_time_data,
            self.right_front_linear_velocity_data,
            label='Right-front'
        )
        
        axes[1].plot(
            self.joint_states_time_data,
            self.left_back_linear_velocity_data,
            label='Left-back'
        )
        
        axes[1].plot(
            self.joint_states_time_data,
            self.right_back_linear_velocity_data,
            label='Right-back'
        )

        axes[1].set_title('Linear velocity of each wheel')
        axes[1].set_xlabel('t')
        axes[1].set_ylabel('linear velocity (m / s)')
        axes[1].legend()
        axes[1].grid(True)

        fig.savefig(
            fname=os.path.join(
                os.getcwd(),
                'analysis',
                'linear_velocity_fig'
            )
        )
        plt.show()


def main():
    rclpy.init()
    node = PIDTuningNode('pid_tuning_node_publisher')

    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.linear.y = 0.0
    stop_msg.linear.z = 0.0
    stop_msg.angular.x = 0.0
    stop_msg.angular.y = 0.0
    stop_msg.angular.z = 0.0

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.10
    cmd_vel_msg.linear.y = 0.0
    cmd_vel_msg.linear.z = 0.0
    cmd_vel_msg.angular.x = 0.0
    cmd_vel_msg.angular.y = 0.0
    cmd_vel_msg.angular.z = 0.0

    Kp = 10.0
    Ki = 0.0
    Kd = 0.0

    input("Press Enter to collect data ")

    # node.set_pid_params(Kp, Ki, Kd)

    node.set_cmd_vel(cmd_vel_msg)
    time.sleep(2.0)
    node.set_cmd_vel(stop_msg)
    node.plot_graphs(cmd_vel_msg.linear.x)

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
