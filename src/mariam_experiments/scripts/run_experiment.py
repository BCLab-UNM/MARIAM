from fabric import SerialGroup
import rclpy
from rclpy.node import Node


class ExperimentNode(Node):

    def __init__(self):
        super().__init__('experiment_node')

        # a variable to keep track of the current stage of the experiment
        self.experiment_stage = 'drive_up'

        self.get_logger().info('Starting up the robots...')
        self.start_robots()

    def control_loop(self):
        """
        This method is the main control loop for the experiment.
        It will be used to monitor the state of each robot, and detect which
        tasks need to be performed next.
        """

        try:
            while rclpy.ok():
                #                 
        
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
        

    def start_robots(self):
        serial_group = SerialGroup(
            'ross.local', 'monica.local',
            user='swarmie',
            # searches for SSH keys
            connect_kwargs={'look_for_keys': True}
        ).run('cd MARIAM')

        for (conn, _), robot_name in zip(serial_group.items(), ['ross', 'monica']):
            self.get_logger().info(f'Starting robot: {robot_name}')
            conn.run(f'ros2 launch mariam_experiments experiment.launch.py robot_name:={robot_name} use_gazebo:=false use_admittance_control:=true')
            self.get_logger().info(f'Robot {robot_name} started successfully.')


