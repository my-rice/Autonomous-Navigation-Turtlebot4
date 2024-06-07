import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # Import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class InfoHandler(Node):

    def __init__(self):
        super().__init__("Info") #Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.amcl_pose = None    
        self.finished = False

        self.navigator = TurtleBot4Navigator()
        self.run()

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")

    
    def run(self):
        while not self.finished:
            if self.amcl_pose is not None:
                self.get_logger().info(f"Pose: {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y}")
            else:
                self.get_logger().info("Pose not available")
            time.sleep(1)





def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    info = InfoHandler()


    executor.add_node(info)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    info.destroy_node()
    rclpy.shutdown()
