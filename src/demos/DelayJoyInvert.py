#!/usr/bin/python3
import serial
import threading
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Joy
import time


class DelayedJoyMoveBase(Node):
    def __init__(self):
        super().__init__('delayed_joy_move_base')
        self.subscription = self.create_subscription(
            Joy,
            '/joy_drive',
            self.joy_callback,
            10)
        self.leader_motor_publisher = self.create_publisher(Int32, 'motor', 10)
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE)
        # self.publisher = self.create_publisher(Float64MultiArray, 'left', 10)
        self.motor_ticks_right_publisher = self.create_publisher(Int32, 'motor_ticks_right', 10)
        self.motor_ticks_left_publisher = self.create_publisher(Int32, 'motor_ticks_left', 10)

    def joy_callback(self, msg):
        # Calculate the time difference between now and the message's timestamp
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        time_diff = msg_time - now

        # Calculate total wait time in nanoseconds
        additional_delay_ns = Duration(seconds=0.5).nanoseconds
        total_wait_time_ns = time_diff.nanoseconds + additional_delay_ns

        # If the total wait time is positive, wait for that duration
        if total_wait_time_ns > 0:
            wait_time_sec = total_wait_time_ns / 1e9
            # self.get_logger().info(f"Waiting for {wait_time_sec} seconds")
            time.sleep(wait_time_sec)

        #self.get_logger().info("Got: " + str(msg.axes[3]) + ":" + str(msg.axes[4]))
        left = ((-1)**msg.buttons[4]) * int((msg.axes[4] - msg.axes[3]) * 50)
        right = ((-1)**msg.buttons[4]) * int((msg.axes[4] + msg.axes[3]) * 50)
        self.port.write((str(left) + " " + str(right) + "\n").encode())

    def read_serial_data(self):
        while True:  # ok is not working here
            try:
                data = self.port.readline().decode().strip().split()  # Read a line from the serial port & Decode and split the received data
                #self.get_logger().info(str(data))
                if len(data) == 2:
                    try:
                        self.motor_ticks_left_publisher.publish(Int32(data=int(data[0])))
                        self.motor_ticks_right_publisher.publish(Int32(data=int(data[1])))
                    except ValueError:
                        print("Error parsing serial data:")
            except serial.SerialException:
                print("Error reading from serial port")
            except UnicodeDecodeError:
                print("Error decoding serial data:")


def main(args=None):
    rclpy.init(args=args)
    node = DelayedJoyMoveBase()
    try:
        serial_thread = threading.Thread(target=node.read_serial_data)
        serial_thread.start()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except AttributeError:  # Exit a bit more gracefully
        pass
    print("Goodbye")


if __name__ == '__main__':
    main()
