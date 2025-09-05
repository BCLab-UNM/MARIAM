#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Pose, PoseArray
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
from coop_traj_viz import plot_summary, animate_carry, plot_trajectories, save_trajectory_csv, plot_trajectory_components
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Instructions
# cd ~/MARIAM/src/mariam_navigation/scripts
# python3 coop_traj_node.py

class CooperativeTrajectoryNode(Node):
    def __init__(self, trial_name=""):
        super().__init__('cooperative_trajectory_node')
        
        # TF2 setup for getting payload transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Name mapping
        name_mapping = {
            'ross': 'base1',
            'monica': 'base2',
        }

        # Qos profile
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        # Publishers for desired poses, use best effort QoS
        self.base_desired_poses = self.create_publisher(PoseArray, '/desired_poses', best_effort_qos)

        # Subscribers for pose, use best effort QoS
        self.base1_pose_sub = self.create_subscription(Pose, '/world_ross_pose', self.base1_pose_callback, best_effort_qos)
        self.base2_pose_sub = self.create_subscription(Pose, '/world_monica_pose', self.base2_pose_callback, best_effort_qos)

        # Pose tracking [x, y, theta]
        self.base1_pose = None
        self.base2_pose = None
        self.payload_pose = None

        # Transform from vicon to base_link
        self.T_vm = np.array([  [ 1, 0, 0,  0.23],
                        [ 0, 1, 0, -0.075],
                        [ 0, 0, 1, -0.06],
                        [ 0, 0, 0,  1]])


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
        self.control_timer = self.create_timer(self.control_dt, self.control_callback)
        
        # Service or trigger to start trajectory (for now, we'll start after getting initial transform)
        self.startup_timer = self.create_timer(1.0, self.startup_callback)
        
        self.get_logger().info("Cooperative Trajectory Node initialized")
        self.get_logger().info(f"Relative Goal: {self.relative_goal}")
        self.get_logger().info("Waiting for payload transform...")
        # self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.get_logger().debug("Debugging active")

    def base1_pose_callback(self, msg):
        """Callback for base 1 pose updates"""
        # Use convert pose into transform
        T_wv = np.eye(4)
        T_wv[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        T_wv[0, 3] = msg.position.x
        T_wv[1, 3] = msg.position.y
        T_wv[2, 3] = msg.position.z

        # Get world to base_link transform
        T_wm = T_wv @ self.T_vm

        # Extract [x, y, theta] from the 3D transformation
        x = T_wm[0, 3]
        y = T_wm[1, 3]
        theta = np.arctan2(T_wm[1, 0], T_wm[0, 0])  # Yaw from rotation matrix
        
        self.base1_pose = np.array([x, y, theta])

    def base2_pose_callback(self, msg):
        """Callback for base 2 pose updates"""
        # Use convert pose into transform
        T_wv = np.eye(4)
        T_wv[:3, :3] = R.from_quat([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]).as_matrix()
        T_wv[0, 3] = msg.position.x
        T_wv[1, 3] = msg.position.y
        T_wv[2, 3] = msg.position.z

        # Get world to base_link transform
        T_wm = T_wv @ self.T_vm

        # Extract [x, y, theta] from the 3D transformation
        x = T_wm[0, 3]
        y = T_wm[1, 3]
        theta = np.arctan2(T_wm[1, 0], T_wm[0, 0])  # Yaw from rotation matrix

        self.base2_pose = np.array([x, y, theta])

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

    def control_callback(self):
        """Main control loop - publish cmd_vel for both bases"""
        if not self.trajectory_active or self.trajectory_params is None:
            return
            
        try:
            # Calculate time since trajectory start
            current_time = time.time()
            t = current_time - self.trajectory_start_time
            self.get_logger().debug(f"Time: {t:.2f}s")

            # Check if trajectory is complete
            if t >= self.trajectory_duration:
                self.stop_trajectory()
                return

            # Get dt
            if self.last_time is None:
                dt = self.control_dt
            else:
                dt = current_time - self.last_time
            
            # Sample current poses from trajectory
            base1_pose, base2_pose = traj(self.trajectory_params, t, T=self.trajectory_duration)
            
            # Convert to numpy arrays for easier math
            b1 = np.asarray(base1_pose, dtype=float)  # [x, y, theta]
            b2 = np.asarray(base2_pose, dtype=float)  # [x, y, theta]

            # Publish desired poses for visualization
            self.publish_robot_poses(b1, b2)

        except Exception as e:
            self.get_logger().error(f"Error in control callback: {str(e)}")
            self.stop_trajectory()

    def publish_robot_poses(self, b1, b2):
        """
        Publish 2D poses for two robots
        Args:
            b1: numpy array [x, y, theta] for robot 1
            b2: numpy array [x, y, theta] for robot 2
        """
        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"  # or whatever frame you're using

        # Robot 1
        pose1 = Pose()
        pose1.position.x = float(b1[0])
        pose1.position.y = float(b1[1])
        pose1.position.z = 0.0
        quat1 = tf_transformations.quaternion_from_euler(0, 0, b1[2])  # roll=0, pitch=0, yaw=theta
        pose1.orientation.x = quat1[0]
        pose1.orientation.y = quat1[1]
        pose1.orientation.z = quat1[2]
        pose1.orientation.w = quat1[3]

        # Robot 2
        pose2 = Pose()
        pose2.position.x = float(b2[0])
        pose2.position.y = float(b2[1])
        pose2.position.z = 0.0
        quat2 = tf_transformations.quaternion_from_euler(0, 0, b2[2])  # roll=0, pitch=0, yaw=theta
        pose2.orientation.x = quat2[0]
        pose2.orientation.y = quat2[1]
        pose2.orientation.z = quat2[2]
        pose2.orientation.w = quat2[3]

        # Add both poses to the array
        pose_array.poses = [pose1, pose2]

        # Publish using your publisher
        self.base_desired_poses.publish(pose_array)

    def stop_trajectory(self):
        """Stop trajectory execution and send zero velocities"""
        if self.trajectory_active:
            self.get_logger().info("Trajectory complete, stopping robots")
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