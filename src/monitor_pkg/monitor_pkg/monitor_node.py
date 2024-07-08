import os
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class ExitException(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.declare_parameter('service_name', '')
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self.get_logger().info('MY SERVICE NAME IS: ' + str(self.service_name))


        self.client = self.create_client(GetState, self.service_name, callback_group=MutuallyExclusiveCallbackGroup())
        self.timer = self.create_timer(1.0, self.check_service_availability, callback_group=MutuallyExclusiveCallbackGroup())

    def check_service_availability(self):
        if self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service {self.client.srv_name} is available.')
            request = GetState.Request()
            future = self.client.call_async(request)
            self.get_logger().info(f'Service {self.client.srv_name} is available.')
            while not future.done():
                self.get_logger().info(f'Waiting for service {self.client.srv_name} to be active.')
                rclpy.spin_once(self, timeout_sec=0.5)
            self.get_logger().info(f'Service {self.client.srv_name} is available.')

            if future.result() is not None:
                response = future.result()
                if response.current_state.label == 'active':
                    self.get_logger().info('Service is active.')
                    raise ExitException('Service is active.')
                else:
                    self.get_logger().error('AIAIAIAIIA' + str(self.service_name))
            else:
                self.get_logger().error('The service: ' + str(self.service_name) + ' is not active.')
                self.get_logger().info('Shutting down the node.')
        else:
            self.get_logger().error('The service: ' + str(self.service_name) + ' is not active.')
            self.get_logger().info('Shutting down the node.')

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(monitor_node)

    try:
        executor.spin()
    except ExitException as e:
        monitor_node.get_logger().info(e.message)
    finally:
        executor.shutdown()
        monitor_node.destroy_node()
        print("Morto")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
