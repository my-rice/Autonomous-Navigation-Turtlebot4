import rclpy
from rclpy.node import Node
from discovery_interface.srv import DiscoveryService
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import yaml
import math

class Discovery(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.srv = self.create_service(DiscoveryService, 'discovery_mode', self.discovery_mode_callback)
        self.get_logger().info("Service 'discovery_mode' is ready")
        self.navigator = TurtleBot4Navigator()
        self.read_parameters()
        self.founded = False

    def read_parameters(self):
        with open('config.yaml', 'r') as file:
            config = yaml.safe_load(file)

        self.policy = config['policy']['array_based']
        self.radius = config['policy']['radius']

    def discovery_mode_callback(self, request, response):
        self.get_logger().info(f'Incoming request\na: {request.goal_pose_x} b: {request.goal_pose_y}')
        self.founded = False
        try:
            self.navigator.startToPose(self.navigator.getPoseStamped([request.goal_pose_x, request.goal_pose_y], request.angle))
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            response.next_action = "error"

        while not self.founded:
            

        
        # Se il ciclo si conclude senza ricevere il segnale di "STOP action",
        # imposta la prossima azione nella risposta come "not found"
        if not self.founded:
            response.next_action = "not found"

        return response

def main(args=None):
    rclpy.init(args=args)
    discovery_node = Discovery("discovery_node")
    rclpy.spin(discovery_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
