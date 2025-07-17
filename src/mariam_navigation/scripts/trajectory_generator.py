#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
from scipy.interpolate import CubicSpline
import math
from typing import List, Tuple

# Custom message imports (you'll need to create these)
from mariam_navigation.msg import TrajectoryPoint, Trajectory


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Declare parameters
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 0.4)
        self.declare_parameter('max_linear_acceleration', 0.4)
        self.declare_parameter('max_angular_acceleration', 0.4)
        self.declare_parameter('trajectory_time_step', 0.1)
        self.declare_parameter('goal_tolerance', 0.05)
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.max_linear_acc = self.get_parameter('max_linear_acceleration').value
        self.max_angular_acc = self.get_parameter('max_angular_acceleration').value
        self.time_step = self.get_parameter('trajectory_time_step').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers and subscribers
        self.trajectory_pub = self.create_publisher(
            Trajectory, 
            '/planned_trajectory', 
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # State
        self.trajectory_generated = False
        
        self.get_logger().info('Trajectory Generator Node initialized')
        
    def goal_callback(self, msg: PoseStamped):
        """Handle incoming goal pose"""
        if self.trajectory_generated:
            self.get_logger().info('Trajectory already generated, ignoring new goal')
            return
            
        self.get_logger().info('Received goal pose, generating trajectory...')
        
        try:
            # Get current robot pose
            current_pose = self.get_current_pose()
            if current_pose is None:
                return
                
            # Generate trajectory
            trajectory = self.generate_trajectory(current_pose, msg)
            if trajectory is not None:
                self.trajectory_pub.publish(trajectory)
                self.trajectory_generated = True
                self.get_logger().info(f'Published trajectory with {len(trajectory.points)} points')
                
        except Exception as e:
            self.get_logger().error(f'Error generating trajectory: {str(e)}')
            
    def get_current_pose(self) -> PoseStamped:
        """Get current robot pose in map frame"""
        try:
            # Get transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_footprint', 
                rclpy.time.Time()
            )
            
            # Create current pose
            current_pose = PoseStamped()
            current_pose.header.frame_id = 'map'
            current_pose.header.stamp = self.get_clock().now().to_msg()
            current_pose.pose.position.x = transform.transform.translation.x
            current_pose.pose.position.y = transform.transform.translation.y
            current_pose.pose.position.z = transform.transform.translation.z
            current_pose.pose.orientation = transform.transform.rotation
            
            return current_pose
            
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {str(e)}')
            return None
            
    def generate_trajectory(self, start_pose: PoseStamped, goal_pose: PoseStamped) -> Trajectory:
        """Generate spline-based trajectory between start and goal poses"""
        
        # Transform goal to map frame if needed
        if goal_pose.header.frame_id != 'map':
            try:
                goal_pose = self.tf_buffer.transform(goal_pose, 'map')
            except Exception as e:
                self.get_logger().error(f'Failed to transform goal pose: {str(e)}')
                return None
        
        # Extract positions and orientations
        start_pos = [start_pose.pose.position.x, start_pose.pose.position.y]
        goal_pos = [goal_pose.pose.position.x, goal_pose.pose.position.y]
        
        start_yaw = self.quaternion_to_yaw(start_pose.pose.orientation)
        goal_yaw = self.quaternion_to_yaw(goal_pose.pose.orientation)
        
        # Check if we're already at goal
        distance_to_goal = np.linalg.norm(np.array(goal_pos) - np.array(start_pos))
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('Already at goal position')
            return None
            
        # Generate spline waypoints
        waypoints = self.generate_spline_path(start_pos, goal_pos, start_yaw, goal_yaw)
        
        # Generate velocity profile
        trajectory_points = self.generate_velocity_profile(waypoints)
        
        # Create trajectory message
        trajectory = Trajectory()
        trajectory.header.frame_id = 'map'
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.points = trajectory_points
        
        return trajectory
        
    def generate_spline_path(self, start_pos: List[float], goal_pos: List[float], 
                           start_yaw: float, goal_yaw: float) -> List[Tuple[float, float, float]]:
        """Generate smooth spline path between start and goal"""
        
        # Create control points for spline
        # Add intermediate points to ensure smooth curve
        distance = np.linalg.norm(np.array(goal_pos) - np.array(start_pos))
        
        # Create intermediate points based on initial heading
        num_control_points = max(3, int(distance / 0.5))  # At least 3 points
        
        # Simple approach: create points along a smooth curve
        t = np.linspace(0, 1, num_control_points)
        
        # Use the start and goal orientations to influence the curve
        start_direction = [np.cos(start_yaw), np.sin(start_yaw)]
        goal_direction = [np.cos(goal_yaw), np.sin(goal_yaw)]
        
        # Scale the direction vectors based on distance
        scale = distance * 0.3
        
        # Control points
        x_control = [start_pos[0]]
        y_control = [start_pos[1]]
        
        # Add intermediate control point influenced by start direction
        if num_control_points > 2:
            x_control.append(start_pos[0] + scale * start_direction[0])
            y_control.append(start_pos[1] + scale * start_direction[1])
            
        # Add intermediate control point influenced by goal direction
        if num_control_points > 3:
            x_control.append(goal_pos[0] - scale * goal_direction[0])
            y_control.append(goal_pos[1] - scale * goal_direction[1])
            
        x_control.append(goal_pos[0])
        y_control.append(goal_pos[1])
        
        # Create parameter array for spline
        t_control = np.linspace(0, 1, len(x_control))
        
        # Create cubic splines
        cs_x = CubicSpline(t_control, x_control)
        cs_y = CubicSpline(t_control, y_control)
        
        # Generate waypoints
        total_time = self.estimate_trajectory_time(distance)
        num_points = int(total_time / self.time_step) + 1
        t_waypoints = np.linspace(0, 1, num_points)
        
        waypoints = []
        for t_val in t_waypoints:
            x = cs_x(t_val)
            y = cs_y(t_val)
            
            # Calculate orientation as tangent to path
            dx = cs_x(t_val, 1)  # First derivative
            dy = cs_y(t_val, 1)
            yaw = math.atan2(dy, dx)
            
            waypoints.append((x, y, yaw))
            
        return waypoints
        
    def estimate_trajectory_time(self, distance: float) -> float:
        """Estimate total trajectory time based on distance and constraints"""
        # Simple trapezoidal velocity profile estimation
        # Time to accelerate to max velocity
        t_accel = self.max_linear_vel / self.max_linear_acc
        
        # Distance covered during acceleration
        d_accel = 0.5 * self.max_linear_acc * t_accel**2
        
        # If total distance is less than 2 * accel distance, we never reach max velocity
        if distance <= 2 * d_accel:
            # Triangular velocity profile
            t_total = 2 * math.sqrt(distance / self.max_linear_acc)
        else:
            # Trapezoidal velocity profile
            d_constant = distance - 2 * d_accel
            t_constant = d_constant / self.max_linear_vel
            t_total = 2 * t_accel + t_constant
            
        return t_total
        
    def generate_velocity_profile(self, waypoints: List[Tuple[float, float, float]]) -> List[TrajectoryPoint]:
        """Generate velocity profile for the waypoints"""
        
        if len(waypoints) < 2:
            return []
            
        trajectory_points = []
        total_distance = self.calculate_path_length(waypoints)
        total_time = self.estimate_trajectory_time(total_distance)
        
        for i, (x, y, yaw) in enumerate(waypoints):
            point = TrajectoryPoint()
            
            # Position
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = 0.0
            point.pose.orientation = self.yaw_to_quaternion(yaw)
            
            # Time
            point.time_from_start = rclpy.duration.Duration(seconds=i * self.time_step).to_msg()
            
            # Calculate velocities
            current_time = i * self.time_step
            linear_vel = self.calculate_linear_velocity(current_time, total_time)
            
            # Angular velocity based on change in heading
            if i > 0 and i < len(waypoints) - 1:
                prev_yaw = waypoints[i-1][2]
                next_yaw = waypoints[i+1][2]
                angular_vel = self.normalize_angle(next_yaw - prev_yaw) / (2 * self.time_step)
                angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
            else:
                angular_vel = 0.0
                
            point.velocity.linear.x = linear_vel
            point.velocity.angular.z = angular_vel
            
            # Calculate accelerations (simple finite difference)
            if i > 0:
                prev_linear_vel = trajectory_points[i-1].velocity.linear.x
                prev_angular_vel = trajectory_points[i-1].velocity.angular.z
                
                linear_acc = (linear_vel - prev_linear_vel) / self.time_step
                angular_acc = (angular_vel - prev_angular_vel) / self.time_step
                
                # Apply acceleration limits
                linear_acc = np.clip(linear_acc, -self.max_linear_acc, self.max_linear_acc)
                angular_acc = np.clip(angular_acc, -self.max_angular_acc, self.max_angular_acc)
                
                point.acceleration.linear.x = linear_acc
                point.acceleration.angular.z = angular_acc
            else:
                point.acceleration.linear.x = 0.0
                point.acceleration.angular.z = 0.0
                
            trajectory_points.append(point)
            
        return trajectory_points
        
    def calculate_linear_velocity(self, current_time: float, total_time: float) -> float:
        """Calculate linear velocity using trapezoidal profile"""
        
        # Time to accelerate to max velocity
        t_accel = self.max_linear_vel / self.max_linear_acc
        
        # Ensure we don't exceed total time
        t_accel = min(t_accel, total_time / 2)
        
        if current_time <= t_accel:
            # Acceleration phase
            return self.max_linear_acc * current_time
        elif current_time >= total_time - t_accel:
            # Deceleration phase
            return self.max_linear_acc * (total_time - current_time)
        else:
            # Constant velocity phase
            return min(self.max_linear_vel, self.max_linear_acc * t_accel)
            
    def calculate_path_length(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calculate total path length"""
        total_length = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            total_length += math.sqrt(dx**2 + dy**2)
        return total_length
        
    def quaternion_to_yaw(self, quaternion) -> float:
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def yaw_to_quaternion(self, yaw: float):
        """Convert yaw angle to quaternion"""
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
        
    def normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()