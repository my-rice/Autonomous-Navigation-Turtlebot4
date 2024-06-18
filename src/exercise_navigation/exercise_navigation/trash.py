import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        

        self.exclusive_group = MutuallyExclusiveCallbackGroup()
        self.reentrant_group = ReentrantCallbackGroup()
        # Create a timer that triggers every second
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.exclusive_group)
        
        # Create another timer that triggers every half second
        self.another_timer = self.create_timer(0.5, self.another_timer_callback, callback_group=self.reentrant_group)
        
        # Create a subscriber
        self.subscription = self.create_subscription(Bool,'test',self.subscription_callback,10, callback_group=self.reentrant_group)
        
    def timer_callback(self):
        self.get_logger().info('Timer callback started')
        # Simulate a long-running task
        import time
        time.sleep(5)  # Simulates a task taking 5 seconds
        self.get_logger().info('Timer callback finished')
    
    def another_timer_callback(self):
        self.get_logger().info('Another timer callback running')

    def subscription_callback(self, msg):
        self.get_logger().info(f'Subscription callback received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    # Use MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # Spin the executor to process callbacks
        executor.spin()
    finally:
        # Shutdown the node and clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
