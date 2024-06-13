import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class cmd_vel_Publisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def go(self):
        msg = Twist()
        msg.linear.x = 0.25
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.57 * factor
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.angular.z)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        
    def timer_callback(self):
        
        msg = Twist()
        msg.linear.x = 0.5
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1



def main():
    rclpy.init()

    cmd_vel_publisher = cmd_vel_Publisher()
    rclpy.spin(cmd_vel_publisher)

    # Finished navigating, dock
    #navigator.dock()

    rclpy.shutdown()