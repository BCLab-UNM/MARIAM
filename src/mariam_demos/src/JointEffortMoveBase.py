#!/usr/bin/python3
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointEffortMoveBase:
    def __init__(self, node: Node, topic: str, initial_effort: float):
        self.node = node
        self.topic = topic
        self.initial_effort = initial_effort
        self.effort_threshold = 80.0

        self.subscriber = node.create_subscription(
            JointState, topic, self.callback, 10)
        self.port = serial.Serial("/dev/ttyACM0", baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)

    def callback(self, data: JointState):
        if self.initial_effort is None:
            self.initial_effort = data.effort
        else:
            for i in range(len(data.name)):
                data.effort[i] -= self.initial_effort[i]
                direction = "extending" if data.effort[i] > 0 else "contracting"
                if abs(data.effort[i]) > self.effort_threshold:
                    print(f"{i} {data.name[i]} {direction} effort: {data.effort[i]}.")
                    
            if abs(data.effort[2]+data.effort[3]) > self.effort_threshold: #elbow + wrist
                self.port.write((str(int((data.effort[2]+data.effort[3])/10))+"\t").encode())
            else: # Stop
                self.port.write("0\t".encode())

def main():
    rclpy.init()
    node = Node("armEffort")
    subscriber = JointEffortMoveBase(node, "/px100/joint_states", None)
    rclpy.spin(node)

if __name__ == "__main__":
    main()


