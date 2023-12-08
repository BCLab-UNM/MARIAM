#!/usr/bin/python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, Float32


class JointEffortMoveBase:
    def __init__(self, node: Node, topic: str, initial_effort: float):
        self.node = node
        self.topic = topic
        self.initial_effort = initial_effort
        self.effort_threshold = 30.0
        self.effort_offset = 50.0
        self.value = 0.0
        self.max = 80.0
        self.min = -80.0
        self.ramp = 8.0
        
        self.subscriber = node.create_subscription(
            JointState, topic, self.callback, 10)
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.fallower_motor_publisher = self.node.create_publisher(Int32, 'fallower_motor', 10)

    def callback(self, data: JointState):
        if self.initial_effort is None:
            self.initial_effort = data.effort
        else:
            target_value = (data.effort[2]-self.initial_effort[2]-data.effort[1]+self.initial_effort[1])/10 + self.effort_offset
                
            if(self.value < target_value):
                self.value = self.value + self.ramp if self.value + self.ramp < self.max else self.max
            else:
                self.value = self.value - self.ramp if self.value - self.ramp > self.min else self.min
                
            #if(data.effort[2] > self.initial_effort[2]) :
            #    self.value = self.value + .1
            #else: 
            #    self.value = self.value - .1
            #print(self.value)
            self.port.write((str(int(self.value))+"\t").encode())
            self.fallower_motor_publisher.publish(Int32(data=int(self.value)))
                
            #else: # Stop
            #    self.port.write("0\t".encode())

def main():
    rclpy.init()
    node = Node("armEffort")
    subscriber = JointEffortMoveBase(node, "/ross_arm/joint_states", None)
    rclpy.spin(node)

if __name__ == "__main__":
    main()


