#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import socket


#So 0.3 meter/s  would be the max linear
class JoyToCmdVelNode(Node):
    def __init__(self):
        hostname = socket.gethostname()
        super().__init__(hostname+'_joy_to_cmd_vel_node')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/'+hostname+'monica/cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1]/8
        twist.angular.z = msg.axes[0]/8
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

