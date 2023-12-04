#!/usr/bin/python3
import serial
import threading
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Joy
import serial
import threading
import math
import time
import rclpy
import math
import tf2_ros
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Joy, JointState
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class WheelOdomMoveNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE)
        # self.motor_ticks_right_publisher = self.create_publisher(Int32, 'motor_ticks_right', 10)
        # self.motor_ticks_left_publisher = self.create_publisher(Int32, 'motor_ticks_left', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Parameters
        # Unloaded no robot 0.3925986m https://www.robotshop.com/products/28-talon-tires-pair
        # wheel_circumference calculated with arm folded up and camera mast at 15 3/8 in(from plastic to plastic), with solar mounts still installed
        self.wheel_circumference_left = 0.3381375 #0.3651  # @TODO check this with arm folded up and with it extended with 50g weight
        self.wheel_circumference_right = 0.3386666666582 #0.3662  # @TODO check this with arm folded up and with it extended with 50g weight
        self.ticks_per_rotation = 2094.625  # @TODO Need figure out where this  ~4 is coming from I double checked the gear ratio not there
        self.wheel_base = 0.3397  # Front/Back 0.18668, left/right 0.2794m # left/right inside 0.2096 meters, left/right outside 0.3397 meters
        self.left_ticks = 0
        self.right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.joy_enabled = True  # get the param
        if self.joy_enabled:
            self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = TransformBroadcaster(self)

    def joy_callback(self, msg):
        # self.get_logger().info(str(msg.axes[3])+":"+str(msg.axes[4]))
        self.port.write((str(int((msg.axes[2] - msg.axes[5]) * 100)) + " " + str(
            int((msg.axes[2] + msg.axes[5]) * 100)) + " \n").encode())

    def cmd_vel_callback(self, msg):
        yaw = msg.angular.z
        v_left = msg.linear.x - yaw * self.wheel_base / 2.0
        v_right = msg.linear.x + yaw * self.wheel_base / 2.0
        self.port.write((str(int(v_left * 100)) + " " + str(int(v_right * 100)) + " \n").encode())

    def read_serial_data(self):
        while True:  # ok is not working here
            try:
                data = self.port.readline().decode().strip().split()  # Read a line from the serial port & Decode and split the received data
                if len(data) == 2:
                    try:
                        # self.motor_ticks_left_publisher.publish(Int32(data=int(data[0])))
                        # self.motor_ticks_right_publisher.publish(Int32(data=int(data[1])))
                        self.compute_odometry(int(data[0]), int(data[1]))
                    except ValueError:
                        print("Error parsing serial data:", data)
            except serial.SerialException:
                print("Error reading from serial port")
            except UnicodeDecodeError:
                print("Error decoding serial data:", data)

    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["left_wheel_joint", "right_wheel_joint"]
        js.position = [(self.left_ticks / self.ticks_per_rotation) * 2 * math.pi,
                       (self.right_ticks / self.ticks_per_rotation) * 2 * math.pi]
        self.joint_state_pub.publish(js)

    def compute_odometry(self, ticks_left, ticks_right):
        self.left_ticks += ticks_left
        self.right_ticks += ticks_right
        self.publish_joint_states()
        dleft = ticks_left / self.ticks_per_rotation * self.wheel_circumference_left
        dright = ticks_right / self.ticks_per_rotation * self.wheel_circumference_right

        dcenter = (dleft + dright) / 2.0
        dtheta = (dright - dleft) / self.wheel_base

        dx = dcenter * math.cos(self.th)
        dy = dcenter * math.sin(self.th)

        self.x += dx
        self.y += dy
        self.th += dtheta

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(position=Point(x=self.x, y=self.y, z=0.0), orientation=self.get_rotation())
        odom.twist.twist = Twist(linear=Vector3(x=dx, y=dy, z=0.0), angular=Vector3(x=0.0, y=0.0, z=dtheta))

        self.odom_pub.publish(odom)

        transform = tf2_ros.TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = self.get_rotation()
        self.broadcaster.sendTransform(transform)

    def get_rotation(self):
        q = quaternion_from_euler(0, 0, self.th)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomMoveNode()
    serial_thread = threading.Thread(target=node.read_serial_data)
    serial_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
