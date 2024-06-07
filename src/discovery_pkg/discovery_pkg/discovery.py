import rclpy
from rclpy.lifecycle import State, TransitionCallbackReturn, Node
from example_interfaces.srv import AddTwoInts

class Discovery(Node):
    def __init__(self, node_name,**kwargs) -> None:
        super().__init__(node_name,**kwargs)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'")
        return TransitionCallbackReturn.SUCCESS

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

def main(args=None) -> None:
  rclpy.init(args=args)
  lifecycle_node = Discovery("lifecycle_node")
  rclpy.spin(lifecycle_node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
