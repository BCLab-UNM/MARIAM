#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Pose
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
from coop_traj_viz import plot_summary, animate_carry, plot_trajectories, save_trajectory_csv
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

        self.trajectory_over_time = {
            'desired_ross': [],
            'desired_monica': [],
            'actual_ross': [],
            'actual_monica': [],
            'actual_payload': [],
            'ross_cmd_vel': [],
            'monica_cmd_vel': [],
            'dt': [],
            'ros_time': []
        }

        # Qos profile
        best_effort_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        # Publishers for cmd_vel, use best effort QoS
        self.base1_cmd_pub = self.create_publisher(Twist, '/ross/cmd_vel', best_effort_qos)
        self.base2_cmd_pub = self.create_publisher(Twist, '/monica/cmd_vel', best_effort_qos)

        # Subscribers for pose, use best effort QoS
        self.base1_pose_sub = self.create_subscription(Pose, '/world_ross_pose', self.base1_pose_callback, best_effort_qos)
        self.base2_pose_sub = self.create_subscription(Pose, '/world_monica_pose', self.base2_pose_callback, best_effort_qos)

        # Pose tracking [x, y, theta]
        self.base1_pose = None
        self.base2_pose = None
        self.payload_pose = None

        # Transform from vicon to base_link
        self.T_vm = np.array([  [ 1, 0, 0,  0.23],
                                [ 0, 1, 0, -0.07],
                                [ 0, 0, 1, -0.06],
                                [ 0, 0, 0,  1]])

        # Trajectory parameters
        self.trajectory_params = None
        self.trajectory_start_time = None
        self.trajectory_duration = 30.0  # seconds
        self.control_rate = 50.0  # Hz
        self.control_dt = 1.0 / self.control_rate
        
        # Plot saving configuration
        self.save_plots = True
        self.plot_prefix = f"../../../data/{trial_name}/ros2_coop_traj"  # Will save as ros2_coop_traj_summary.png
        self.animation_file_name = f"../../../data/{trial_name}/ros2_coop_traj_animation.gif"

        # Hard-coded goal relative to the payload's start pose
        self.relative_goal = [2.0, 0.5, 0.0]  # [x, y, theta] - modify as needed
        self.goal = [0.0, 0.0, 0.0]  # Will be set after getting initial transform

        # Closed-loop control gains
        self.control_gains = {
            'kp_linear': 0.5,        # Conservative gain for 0.2 m/s
            'kp_angular': 0.5,       # Conservative gain for 0.2 rad/s
            'ki_linear': 0.01,      # Integral gain for position 
            'ki_angular': 0.02,     # Integral gain for orientation
            'kd_linear': 0.01,     # Derivative gain for position 
            'kd_angular': 0.01,     # Derivative gain for orientation
            'integral_limit': 0.2,  # Limit for integral windup
            'max_linear_vel': 0.4,   # 2x expected speed (safety margin)
            'max_angular_vel': 0.4,  # 2x expected angular speed
            'deadband_linear': 0.01,
            'deadband_angular': 0.05 
        }

        # Add PID state tracking
        self.pid_state = {
            'base1': {
                'error_integral': np.array([0.0, 0.0, 0.0]),  # [x, y, theta]
                'error_last': np.array([0.0, 0.0, 0.0]),
                'error_history': []
            },
            'base2': {
                'error_integral': np.array([0.0, 0.0, 0.0]),
                'error_last': np.array([0.0, 0.0, 0.0]),
                'error_history': []
            }
        }

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
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
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

    def payload_pose_callback(self, msg):
        """Callback for payload pose updates"""
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

        # Extract [x, y, theta] from the 3D transformation
        x = T_wv[0, 3]
        y = T_wv[1, 3]
        theta = np.arctan2(T_wv[1, 0], T_wv[0, 0])

        self.payload_pose = np.array([x, y, theta])



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
            # Start a stream of 0 cmd_vel messages 
            self.publish_cmd_vel(self.base1_cmd_pub, [0, 0, 0])
            self.publish_cmd_vel(self.base2_cmd_pub, [0, 0, 0])
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
            
            # Calculate velocities using simple finite differences
            if self.last_poses['base1'] is None or self.last_poses['base2'] is None:
                v1 = np.zeros_like(b1)
                v2 = np.zeros_like(b2)
            else:
                v1 = (b1 - self.last_poses['base1']) / dt
                v2 = (b2 - self.last_poses['base2']) / dt

            # Reverse linear.x for base2 (Monica) since it drives backwards
            v2_reversed = v2.copy()
            v2_reversed[0] = -v2[0]  # reverse x velocity

            # Calculate PID corrections
            v1_corrected = self.calculate_corrected_velocity(v1, 'base1', b1, dt)
            v2_corrected = self.calculate_corrected_velocity(v2, 'base2', b2, dt)

            # Combine feedforward + correction
            v1_final = v1 + v1_corrected
            v2_final = v2 + v2_corrected

            v2_final_reversed = v2_final.copy()
            v2_final_reversed[0] = -v2_final[0]  # reverse x velocity

            # Publish the corrected velocities
            self.publish_cmd_vel(self.base1_cmd_pub, v1_final)
            self.publish_cmd_vel(self.base2_cmd_pub, v2_final_reversed)
            # add to CSV

            # Store poses and time for next iteration
            self.last_poses['base1'] = b1.copy()
            self.last_poses['base2'] = b2.copy()
            self.last_time = current_time

            # save desired base poses
            self.trajectory_over_time['desired_ross'].append(b1)
            self.trajectory_over_time['desired_monica'].append(b2)
            # save actual base poses
            self.trajectory_over_time['actual_ross'].append(self.base1_pose)
            self.trajectory_over_time['actual_monica'].append(self.base2_pose)
            # save actual payload poses
            self.trajectory_over_time['actual_payload'].append(self.get_transform('payload'))
            # save timing data
            self.trajectory_over_time['dt'].append(dt)
            self.trajectory_over_time['ros_time'].append(self.get_clock().now())
            # save cmd_vel data
            self.trajectory_over_time['ross_cmd_vel'].append(self.base1_cmd_pub)
            self.trajectory_over_time['monica_cmd_vel'].append(self.base2_cmd_pub)

        except Exception as e:
            self.get_logger().error(f"Error in control callback: {str(e)}")
            self.stop_trajectory()

    def calculate_corrected_velocity(self, feedforward_vel, base_name, desired_pose, dt=None):
        """
        Calculate PID correction to add to feedforward velocity
        
        Args:
            feedforward_vel: numpy array [vx, vy, vtheta]
            base_name: 'base1' or 'base2' 
            desired_pose: numpy array [x, y, theta] desired pose
            dt: actual time step (if None, uses self.control_dt)
            
        Returns:
            numpy array [vx_correction, vy_correction, vtheta_correction]
        """
        if dt is None:
            dt = self.control_dt
            
        try:
            # Get current actual pose of the base
            if base_name == "base1":
                actual_pose = self.base1_pose
            else:
                actual_pose = self.base2_pose
                actual_pose[2] += np.pi

            # Calculate pose errors in world frame
            error = np.array(desired_pose) - np.array(actual_pose)
            error[2] = self.wrap_angle(error[2])  # Wrap angular error

            self.get_logger().debug(f"Base {base_name} expected: {desired_pose}")
            self.get_logger().debug(f"Base {base_name} actual: {actual_pose}")
            self.get_logger().debug(f"Base {base_name} error: {error}")
            
            # Apply deadband
            error[0] = 0.0 if abs(error[0]) < self.control_gains['deadband_linear'] else error[0]
            error[1] = 0.0 if abs(error[1]) < self.control_gains['deadband_linear'] else error[1]
            error[2] = 0.0 if abs(error[2]) < self.control_gains['deadband_angular'] else error[2]
            
            # Get PID state for this base
            pid_state = self.pid_state[base_name]
            
            # Update integral with windup protection
            pid_state['error_integral'] += error * dt
            
            # Clamp integral terms
            integral_limit = self.control_gains['integral_limit']
            pid_state['error_integral'] = np.clip(pid_state['error_integral'], 
                                                -integral_limit, integral_limit)
            
            # Calculate derivative
            if len(pid_state['error_history']) > 0:
                error_derivative = (error - pid_state['error_last']) / dt
            else:
                error_derivative = np.zeros(3)
            
            # PID calculation
            linear_kp = self.control_gains['kp_linear']
            linear_ki = self.control_gains['ki_linear']
            linear_kd = self.control_gains['kd_linear']
            angular_kp = self.control_gains['kp_angular']
            angular_ki = self.control_gains['ki_angular']
            angular_kd = self.control_gains['kd_angular']
            
            correction = np.array([
                linear_kp * error[0] + linear_ki * pid_state['error_integral'][0] + linear_kd * error_derivative[0],
                linear_kp * error[1] + linear_ki * pid_state['error_integral'][1] + linear_kd * error_derivative[1],
                angular_kp * error[2] + angular_ki * pid_state['error_integral'][2] + angular_kd * error_derivative[2]
            ])
            
            # Apply velocity limits
            correction[0] = np.clip(correction[0], -self.control_gains['max_linear_vel'], self.control_gains['max_linear_vel'])
            correction[1] = np.clip(correction[1], -self.control_gains['max_linear_vel'], self.control_gains['max_linear_vel'])
            correction[2] = np.clip(correction[2], -self.control_gains['max_angular_vel'], self.control_gains['max_angular_vel'])
            
            # Update PID state
            pid_state['error_last'] = error.copy()
            pid_state['error_history'].append(error.copy())
            
            # Keep history manageable
            if len(pid_state['error_history']) > 100:
                pid_state['error_history'].pop(0)

            self.get_logger().debug(f"Base {base_name}    feedforward: {feedforward_vel}")
            self.get_logger().debug(f"Base {base_name} PID correction: {correction}")

            return correction
            
        except Exception as e:
            self.get_logger().error(f"Error calculating PID correction for {base_name}: {str(e)}")
            return np.zeros(3)

    def publish_cmd_vel(self, publisher, vel_array):
        """Publish a Twist message from numpy array [vx, vy, vtheta]"""
        msg = Twist()
        msg.linear.x = float(vel_array[0])
        msg.linear.y = float(vel_array[1])  # or 0.0 if you don't want lateral motion
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(vel_array[2])
        
        publisher.publish(msg)

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
        # Always plot trajectories and save data, regardless of how the node ended
        try:
            # Plot trajectories
            plot_trajectories(
                np.array(node.trajectory_over_time['desired_ross']),
                np.array(node.trajectory_over_time['desired_monica']),
                np.array(node.trajectory_over_time['actual_ross']),
                np.array(node.trajectory_over_time['actual_monica']),
                trial_name
            )
            save_trajectory_csv(node.trajectory_over_time, trial_name)
            print("Trajectories plotted and data saved successfully")
        except Exception as plot_error:
            print(f"Error creating plots or saving data: {plot_error}")
        
        try:
            node.destroy_node()
        except:
            pass
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()