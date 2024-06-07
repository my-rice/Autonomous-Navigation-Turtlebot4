# import rclpy
# from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor  # Import MultiThreadedExecutor
# from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
# import time
# from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


# class InfoHandler(Node):

#     def __init__(self):
#         super().__init__("Info") #Init node
#         self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
#         self.amcl_pose = None    
#         self.finished = False

#     def pose_callback(self, msg):
#         self.amcl_pose = msg
#         self.get_logger().info(f"Pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")

    
    


# class Planner(Node):

#     def __init__(self,info):
#         super().__init__("Planner")
#         self.info = info
#         self.finished = False
#         self.navigator = TurtleBot4Navigator()
#         self.run()

#     def run(self):
#         while not self.finished:
#             self.amcl_pose = self.info.amcl_pose
#             if self.amcl_pose is not None:
#                 self.get_logger().info(f"Pose: {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y}")
#             else:
#                 self.get_logger().info("Pose not available")
#             time.sleep(1)

# def main(args=None):
#     rclpy.init(args=args)

#     executor = MultiThreadedExecutor()

#     info = InfoHandler()
#     planner = Planner(info)

#     executor.add_node(info)
#     executor.add_node(planner)

#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         pass

#     info.destroy_node()
#     planner.destroy_node()
#     rclpy.shutdown()


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class InfoHandler(Node):

    def __init__(self):
        super().__init__("Info") # Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.amcl_pose = None
        self.finished = False
        self.navigator = TurtleBot4Navigator()

        # Create a timer that triggers every second
        self.timer = self.create_timer(1.0, self.run)

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")

    def run(self):
        if self.amcl_pose is not None:
            self.get_logger().info(f"Pose: {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y}")
        else:
            self.get_logger().info("Pose not available")

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

if __name__ == '__main__':
    main()