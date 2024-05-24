#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import socket

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        host = socket.gethostname()
        self.publisher = self.create_publisher(Image, '/{}/camera/color/image_raw'.format(host), 10)

    def publish_message(self):
        msg = Image()
        # Populate the message with data
        # For simplicity, you can leave it empty for testing
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()
    # Publish messages at a regular interval for testing
    timer = publisher.create_timer(1, publisher.publish_message)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
