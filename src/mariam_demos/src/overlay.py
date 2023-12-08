import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class AprilTagListener(Node):
    def __init__(self):
        super().__init__("apriltag_listener")
        self.bridge = CvBridge()
        self.latest_detections = None

        self.image_subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10,
        )

        self.detections_subscription = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.detection_callback,
            10,
        )

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        print("Image Shape:", image.shape)
        print("Image Data Type:", image.dtype)
       
        if image.shape[2] == 1:  # Convert single-channel image to 3 channels
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # Process the image or draw detections
        if self.latest_detections is not None:
            self.draw_detected_tags(image)
        cv2.imshow("Tag Detections", image)
        cv2.waitKey(1)  # Display the image

    def draw_detected_tags(self, image):
        # Draw detected tags on the image
        # You can use the detection information to draw bounding boxes, IDs, etc.
        # Example: draw a rectangle around each detected tag
        detections = self.latest_detections
        for detection in detections:
            corners = detection.corners
            for i in range(4):
                cv2.line(image, (int(corners[i].x), int(corners[i].y)),
                         (int(corners[(i+1) % 4].x), int(corners[(i+1) % 4].y)), (0, 255, 0), 2)

    def detection_callback(self, msg):
        # Store the detection results for later processing in image_callback
        self.latest_detections = msg.detections

def main(args=None):
    rclpy.init(args=args)
    listener = AprilTagListener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

