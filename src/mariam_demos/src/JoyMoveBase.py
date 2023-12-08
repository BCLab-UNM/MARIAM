#!/usr/bin/python3
import serial
import threading
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Joy

class JoyMoveBase(Node):

    def __init__(self):
        super().__init__('joy_move_base')
        self.leader_motor_publisher = self.create_publisher(Int32, 'motor', 10)
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        #self.publisher = self.create_publisher(Float64MultiArray, 'left', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.motor_ticks_right_publisher = self.create_publisher(Int32, 'motor_ticks_right', 10)
        self.motor_ticks_left_publisher = self.create_publisher(Int32, 'motor_ticks_left', 10)
        

    def joy_callback(self, msg):
        #self.get_logger().info(str(msg.axes[3])+":"+str(msg.axes[4]))
        self.port.write((str(int((msg.axes[4]-msg.axes[3])*50)) + " " + str(int((msg.axes[4]+msg.axes[3])*50)) + "\t").encode())
        #self.leader_motor_publisher.publish(Int32(data=int()))
        #self.publisher.publish(left_msg)
        
    def read_serial_data(self):
        while True: # ok is not working here
            try:
                data = self.port.readline().decode().strip().split()  # Read a line from the serial port & Decode and split the received data
                #self.get_logger().info(str(data))
                if len(data) == 2:
                    try:
                        self.motor_ticks_left_publisher.publish(Int32(data=int(data[0])))
                        self.motor_ticks_right_publisher.publish(Int32(data=int(data[1])))
                    except ValueError:
                        print("Error parsing serial data:", line)
            except serial.SerialException:
                print("Error reading from serial port")
            except UnicodeDecodeError:
                print("Error decoding serial data:", line)

def main(args=None):
    rclpy.init(args=args)
    node = JoyMoveBase()
    try:
        serial_thread = threading.Thread(target=node.read_serial_data)
        serial_thread.start()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        #node.leader_motor_publisher.destroy()
        #node.leader_motor_hz_publisher.destroy()
    except AttributeError:  # Exit a bit more gracefully
        pass
    print("Goodbye")  # as the node might be gone at this point so node.get_logger().info("Goodbye") might not work)


if __name__ == '__main__':
    main()
