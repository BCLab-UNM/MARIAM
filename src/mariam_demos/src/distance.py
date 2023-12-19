#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float64
import math

class DistanceCalculatorNode(Node):
    def __init__(self):
        super().__init__('distance_calculator_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers for each distance topic
        self.publisher_monica_to_ross = self.create_publisher(Float64, 'MonicaToRoss', 10)
        self.publisher_payload_to_monica = self.create_publisher(Float64, 'PayloadToMonica', 10)
        self.publisher_payload_to_ross = self.create_publisher(Float64, 'PayloadToRoss', 10)

        self.timer = self.create_timer(0.1, self.calculate_and_publish_distances)

    def calculate_and_publish_distances(self):
        self.publish_distance('Monica', 'Ross', self.publisher_monica_to_ross)
        self.publish_distance('Payload', 'Monica', self.publisher_payload_to_monica)
        self.publish_distance('Payload', 'Ross', self.publisher_payload_to_ross)

    def publish_distance(self, frame1, frame2, publisher):
        try:
            # Look up the transformation between the specified frames
            trans = self.tf_buffer.lookup_transform(frame1, frame2, rclpy.time.Time())
            
            # Calculate the distance
            translation = trans.transform.translation
            distance = math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)

            # Publish the distance
            distance_msg = Float64()
            distance_msg.data = distance
            publisher.publish(distance_msg)

            self.get_logger().info(f'Distance between {frame1} and {frame2}: {distance} meters')
        except Exception as e:
            self.get_logger().error(f'Could not calculate distance between {frame1} and {frame2}: {e}')

def main(args=None):
    rclpy.init(args=args)
    distance_calculator_node = DistanceCalculatorNode()
    rclpy.spin(distance_calculator_node)
    distance_calculator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

