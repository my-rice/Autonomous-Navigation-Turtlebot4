import os
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.declare_parameter('service_name','')
        service_name = self.get_parameter('service_name').get_parameter_value().string_value

        self.client = self.create_client(ChangeState, service_name)
        self.timer = self.create_timer(1.0, self.check_service_availability)

    def check_service_availability(self):
        if self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.client.srv_name} is available.')
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)

if __name__ == '__main__':
    main()