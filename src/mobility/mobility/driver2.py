import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import math
import socket
from tf_transformations import euler_from_quaternion

class PositionController(Node):
    def __init__(self):
        hostname = socket.gethostname()
        super().__init__(hostname+'_position_controller')
        
        # Dynamically create topic names with hostname
        odom_topic = f'/{hostname}/odom/wheel'
        pose_topic = f'/{hostname}/pose'
        cmd_vel_topic = f'/{hostname}/cmd_vel'

        # Subscribe to /odom to get current position and orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)
        # Subscribe to /pose to get target position
        self.pose_subscription = self.create_subscription(
            Pose,
            pose_topic,
            self.pose_callback,
            10)
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Initialize target position with None
        self.target_position_x = None
        self.target_position_y = None

        # Movement and rotation threshold
        self.distance_threshold = 0.1  # Meters
        self.angle_threshold = 0.1  # Radians

    def pose_callback(self, msg):
        # Update target position from the /pose topic
        self.target_position_x = msg.position.x
        self.target_position_y = msg.position.y
        self.get_logger().info(f'New target position set: ({self.target_position_x}, {self.target_position_y})')

    def odom_callback(self, msg):
        # Do not proceed if target position has not been set
        if self.target_position_x is None or self.target_position_y is None:
            return

        # Extract current position and orientation
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, theta_current = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calculate the angle to the target
        angle_to_target = math.atan2(self.target_position_y - current_y, self.target_position_x - current_x)

        # Calculate the shortest angular distance between the current orientation and the angle to target
        angle_diff = (angle_to_target - theta_current + math.pi) % (2 * math.pi) - math.pi

        # Calculate distance to target
        distance_to_target = math.sqrt((self.target_position_x - current_x) ** 2 + (self.target_position_y - current_y) ** 2)

        # Initialize the Twist message
        vel_cmd = Twist()

        if distance_to_target > self.distance_threshold:
            if abs(angle_diff) > self.angle_threshold:
                # Rotate towards the target
                vel_cmd.angular.z = 2.0 * angle_diff if angle_diff < math.pi else -2.0 * (2 * math.pi - angle_diff)
            else:
                # Move forward towards the target
                vel_cmd.linear.x = min(distance_to_target, 0.5)  # Cap max speed to 0.5 m/s
        else:
            # If within threshold distance, stop
            self.get_logger().info('Target reached, stopping.')

        # Publish the command
        self.publisher.publish(vel_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

