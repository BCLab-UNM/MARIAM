import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import csv

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS, JointGroupCommand

class JointStatesPlotter(Node):
    def __init__(self):
        super().__init__('joint_states_plotter')
        self.subscription = self.create_subscription(
            JointState,
            '/ross_arm/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.move_arm_publisher = self.create_publisher(JointGroupCommand, '/ross_arm/commands/joint_group', 10)
        self.torque_data = {}
        self.joint_zeros = {}
        self.position_zeros = []

    def joint_states_callback(self, msg):
        if not self.position_zeros:
            self.position_zeros =  msg.position[:5]
            print("msg.positions:", self.position_zeros)
        for joint, effort, position in zip(msg.name, msg.effort, msg.position):
            if joint not in self.torque_data:
                self.torque_data[joint] = []
                self.joint_zeros[joint] = effort
                
            #/1000 converts to a torque scaler and multiply by 1.4 N.m which is our Gear Ratio
            self.torque_data[joint].append((effort-self.joint_zeros[joint])/1000*1.4)
            #
            self.move_arm_publisher.publish(JointGroupCommand(name="all", cmd=self.position_zeros))
            
    def plot_torque_data(self):
        for joint, efforts in self.torque_data.items():
            plt.figure()
            plt.plot(efforts)
            plt.title(f'Effort Index for Joint {joint}')
            plt.xlabel('Time')
            plt.ylabel('torque in nm')
            plt.grid(True)
            
            # Save plot as an image
            plt.savefig(f'joint_{joint}_effort_plot.png')
            plt.close()

            # Save effort data to CSV
            csv_filename = f'joint_{joint}_torque_data.csv'
            with open(csv_filename, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['Time', 'Torque'])
                for idx, effort in enumerate(efforts):
                    csv_writer.writerow([idx, effort])

def main(args=None):
    rclpy.init(args=args)
    plotter = JointStatesPlotter()

    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        pass

    plotter.plot_torque_data()
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
