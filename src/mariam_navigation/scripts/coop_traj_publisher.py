#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray
from tf2_ros import TransformListener, Buffer
import tf_transformations
import numpy as np
import pandas as pd
import time
import sys
import os
import threading
from scipy.spatial.transform import Rotation as R

# Import the cooperative trajectory planning API
from coop_traj_api import opt_traj_params, traj
from coop_traj_viz import plot_summary
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Instructions
# cd ~/MARIAM/src/mariam_navigation/scripts
# python3 coop_traj_publisher.py

class CooperativeTrajectoryNode(Node):
    def __init__(self, trial_name=""):
        super().__init__('cooperative_trajectory_node')
        
        # TF2 setup for getting payload transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Qos profile
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Create a publisher for the pose array
        self.pose_array_pub = self.create_publisher(PoseArray, '/desired_poses', best_effort_qos)

        # Pose tracking [x, y, theta]
        self.base1_pose = None
        self.base2_pose = None
        self.payload_pose = None

        # Trajectory parameters
        self.trajectory_params = None
        self.trajectory_start_time = None
        self.trajectory_duration = 30.0  # seconds
        self.control_rate = 100.0  # Hz
        self.control_dt = 1.0 / self.control_rate
        
        # Plot saving configuration
        self.save_plots = True
        self.plot_prefix = f"../../../data/{trial_name}/ros2_coop_traj"  # Will save as ros2_coop_traj_summary.png
        self.animation_file_name = f"../../../data/{trial_name}/ros2_coop_traj_animation.gif"

        # Hard-coded goal relative to the payload's start pose
        self.relative_goal = [2.5, -1.0, 0.0]  # [x, y, theta] - modify as needed
        self.goal = [0.0, 0.0, 0.0]  # Will be set after getting initial transform

        # State tracking
        self.trajectory_active = False
        self.last_poses = {'base1': None, 'base2': None}
        self.last_time = None

        # Control timer
        self.traj_pub_timer = self.create_timer(self.control_dt, self.traj_pub_callback)
        
        # Service or trigger to start trajectory (for now, we'll start after getting initial transform)
        self.startup_timer = self.create_timer(1.0, self.startup_callback)
        
        self.get_logger().info("Cooperative Trajectory Node initialized")
        self.get_logger().info(f"Relative Goal: {self.relative_goal}")
        self.get_logger().info("Waiting for payload transform...")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().debug("Debugging active")

    def startup_callback(self):
        """Try to get initial payload transform and start trajectory"""
        if self.trajectory_active:
            self.startup_timer.cancel()
            return
        
        try:
            # Get current payload transform
            start_pose = self.get_transform("payload")
            if start_pose is not None and self.base1_pose is not None and self.base2_pose is not None:
                self.get_logger().info(f"Got initial payload pose: {start_pose}")
                self.goal = [
                    start_pose[0] + self.relative_goal[0],
                    start_pose[1] + self.relative_goal[1],
                    start_pose[2] + self.relative_goal[2]
                ]
                self.get_logger().info(f"Calculated goal pose: {self.goal}")
                self.start_trajectory(start_pose)
                self.startup_timer.cancel()
            else:
                self.get_logger().warn(f"Failed to get payload transform")

        except Exception as e:
            self.get_logger().error(f"Error in startup: {str(e)}")

    def get_transform(self, frame_id):
        """Get current transform from world to payload"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extract theta
            quat = transform.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            # if frame_id == "payload":
            #     return [0, 0, 0]

            return [x, y, theta]
            
        except Exception as e:
            self.get_logger().debug(f"Could not get transform for {frame_id}: {str(e)}")
            return None

    def start_trajectory(self, start_pose):
        """Generate and start executing trajectory"""
        try:
            self.get_logger().info(f"Planning trajectory from {start_pose} to {self.goal}")
            
            # Generate trajectory parameters
            self.trajectory_params = opt_traj_params(
                start=start_pose,
                goal=self.goal,
                T=self.trajectory_duration
            )

            # Log starting conditions
            payload_start_pose = self.get_transform('payload')
            base1_start_pose, base2_start_pose = traj(self.trajectory_params, t=0, T=self.trajectory_duration)
            self.get_logger().info(f"Payload starting conditions: {payload_start_pose[0]:.3f}, {payload_start_pose[1]:.3f}, {payload_start_pose[2]:.3f}")
            self.get_logger().info(f"Base 1 expected conditions: [{base1_start_pose[0]:.3f}, {base1_start_pose[1]:.3f}, {base1_start_pose[2]:.3f}]")
            self.get_logger().info(f"Base 1 actual conditions: [{self.base1_pose[0]:.3f}, {self.base1_pose[1]:.3f}, {self.base1_pose[2]:.3f}]")
            self.get_logger().info(f"Base 2 expected conditions: [{base2_start_pose[0]:.3f}, {base2_start_pose[1]:.3f}, {base2_start_pose[2]:.3f}]")
            self.get_logger().info(f"Base 2 actual conditions: [{self.base2_pose[0]:.3f}, {self.base2_pose[1]:.3f}, {self.base2_pose[2]:.3f}]")

            # Save trajectory summary plot
            if self.save_plots:
                try:
                    self.get_logger().info(f"Saving trajectory plot with prefix: {self.plot_prefix}")
                    plot_summary(self.trajectory_params, show=False, save_prefix=self.plot_prefix)
                    self.get_logger().info(f"Trajectory plot saved as {self.plot_prefix}_summary.png")
                    # animate_carry(self.trajectory_params, fps=30, head_len=0.18,
                    #                show_arms=True, show_box=True, show_pivots=True,
                    #                frame_step=25, path_stride=6, save_gif=self.animation_file_name)
                    # self.get_logger().info(f"Trajectory animation saved as {self.animation_file_name}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to save trajectory plot: {str(e)}")

            # Start trajectory execution
            self.trajectory_start_time = time.time()
            self.trajectory_active = True
            self.last_poses = {'base1': None, 'base2': None}
            
            self.get_logger().info("Trajectory planning complete, starting execution")
            
        except Exception as e:
            self.get_logger().error(f"Failed to plan trajectory: {str(e)}")

    def traj_pub_callback(self):
        """Main control loop - publish cmd_vel for both bases"""
        if not self.trajectory_active or self.trajectory_params is None:
            return
            
        try:
            # Calculate time since trajectory start
            current_time = time.time()
            t = current_time - self.trajectory_start_time
            self.get_logger().info(f"Time: {t:.2f}s")

            # Check if trajectory is complete
            if t >= self.trajectory_duration:
                self.stop_trajectory()
                return
            
            # Sample current poses from trajectory
            base1_pose, base2_pose = traj(self.trajectory_params, t, T=self.trajectory_duration)

            # Publish desired poses as PoseArray
            self.publish_pose_array(base1_pose, base2_pose)

            if t < 1.0 or t > (self.trajectory_duration - 1.0):  # First/last second
                self.get_logger().info(f"Desired poses: Base1: [{base1_pose[0]:.3f}, {base1_pose[1]:.3f}, {base1_pose[2]:.3f}], ")
                self.get_logger().info(f"               Base2: [{base2_pose[0]:.3f}, {base2_pose[1]:.3f}, {base2_pose[2]:.3f}]")

            # Store poses and time for next iteration
            self.last_poses['base1'] = base1_pose.copy()
            self.last_poses['base2'] = base2_pose.copy()
            self.last_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error in control callback: {str(e)}")
            self.stop_trajectory()


    def publish_pose_array(self, base1_pose, base2_pose):
        """Publish desired poses as a PoseArray"""
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = "world"
        
        pose1 = Pose()
        pose1.position.x = base1_pose[0]
        pose1.position.y = base1_pose[1]
        pose1.position.z = 0.0
        # TODO: make sure this order is correct
        quat1 = tf_transformations.quaternion_from_euler(0, 0, base1_pose[2])
        pose1.orientation.x = quat1[0]
        pose1.orientation.y = quat1[1]
        pose1.orientation.z = quat1[2]
        pose1.orientation.w = quat1[3]
        
        pose2 = Pose()
        pose2.position.x = base2_pose[0]
        pose2.position.y = base2_pose[1]
        pose2.position.z = 0.0
        quat2 = tf_transformations.quaternion_from_euler(0, 0, base2_pose[2])
        pose2.orientation.x = quat2[0]
        pose2.orientation.y = quat2[1]
        pose2.orientation.z = quat2[2]
        pose2.orientation.w = quat2[3]
        
        pose_array_msg.poses = [pose1, pose2]
        
        self.pose_array_pub.publish(pose_array_msg)

    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def stop_trajectory(self):
        """Stop trajectory execution and send zero velocities"""
        if self.trajectory_active:
            self.get_logger().info("Trajectory complete, stopping robots")
            
            # Send zero velocities
            zero_twist = Twist()
            self.base1_cmd_pub.publish(zero_twist)
            self.base2_cmd_pub.publish(zero_twist)
            
            self.trajectory_active = False
            self.trajectory_params = None
            self.last_poses = {'base1': None, 'base2': None}

    def destroy_node(self):
        """Clean shutdown"""
        self.stop_trajectory()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # Parse command-line arguments
    trial_name = ""
    if len(sys.argv) > 1:
        trial_name = sys.argv[1]
        print(f"Using trial name: {trial_name}")
        # Create a directory for trial
        os.makedirs(f"../../../data/{trial_name}", exist_ok=True)
    else:
        print("No trial name provided, using default file names")
    
    node = None
    try:
        node = CooperativeTrajectoryNode(trial_name=trial_name)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()