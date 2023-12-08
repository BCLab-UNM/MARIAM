#!/usr/bin/python3
import serial
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32


class SineMoveBase(Node):

    def __init__(self):
        super().__init__('sine_move_base')
        self.leader_motor_hz_publisher = self.create_publisher(Float32, 'leader_motor_hz', 10)
        self.leader_motor_publisher = self.create_publisher(Int32, 'leader_motor', 10)
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.frequency_range = [0.1, 0.2, 0.5, 1, 2, 4]  # Frequencies from 0.5 Hz to 10 Hz
        self.number_of_periods = 3
        self.amplitude = 70.0  # You can adjust this to control the amplitude of the sine wave.
        self.sampling_rate = 300  # Adjust this value based on your requirements (samples per second).
        self.sin_waves()

    def generate_sin_wave(self, frequency):
        for i in range(int(1 / frequency * self.sampling_rate)):
            t = i / self.sampling_rate
            value = self.amplitude * math.sin(2 * math.pi * frequency * t)
            self.port.write((str(int(value)) + "\t").encode())
            self.leader_motor_publisher.publish(Int32(data=int(value)))
            self.get_logger().info(str(value))
            time.sleep(1 / self.sampling_rate)

    def sin_waves(self):
        for frequency in self.frequency_range:
            self.get_logger().info(f"Frequency: {frequency} Hz")
            self.leader_motor_hz_publisher.publish(Float32(data=float(frequency)))
            for _ in range(self.number_of_periods):  # number of periods
                self.generate_sin_wave(frequency)
        self.port.write(("0" + "\t").encode())  # Stop the motors
        # Close both publishers and shutdown the node
        self.leader_motor_publisher.destroy()
        self.leader_motor_hz_publisher.destroy()
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SineMoveBase()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except AttributeError:  # Exit a bit more gracefully
        pass
    print("Goodbye")  # as the node might be gone at this point so node.get_logger().info("Goodbye") might not work)


if __name__ == '__main__':
    main()
