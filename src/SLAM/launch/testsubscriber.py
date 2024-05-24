#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        host = socket.gethostname()
        self.subscription = self.create_subscription(
            Image,
            '/{}/camera/color/image_raw'.format(host),  # Adjust topic name as needed
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info('Received message')

def main(args=None):
    rclpy.init(args=args)
    subscriber = SimpleSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
