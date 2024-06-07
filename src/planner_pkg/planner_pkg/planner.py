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
        self.sub_odom = self.create_subscription(Odometry, "/odom", self.odom_callback, 10) # INUTILE
        #self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 10)
        self.amcl_pose = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        
        self.pose = None
        self.amcl_pose = None
        self.odom = None
        self.finished = False

        self.loop()
    
    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")


    
    def odom_callback(self, msg):
        self.odom = msg
        self.get_logger().info(f"Odometry: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
    


    def loop(self):

        navigator = TurtleBot4Navigator()

        finished = False
        
        # Set initial pose
        # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH) # TODO: The initial pose will be given dynamically
        # navigator.setInitialPose(initial_pose)

        # Wait for Nav2
        #navigator.waitUntilNav2Active()


        while not self.finished:
            if self.amcl_pose is not None:
                self.get_logger().info("self.acml_pose is not None", self.amcl_pose)
            else:
                self.get_logger().info("Planning...")
            time.sleep(2)



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
