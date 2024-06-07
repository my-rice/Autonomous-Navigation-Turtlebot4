import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor  # Import MultiThreadedExecutor
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

class PlannerHandler(Node):

    def __init__(self):
        super().__init__("Info") # Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)


        self.amcl_pose = None
        self.finished = False

        self.next_goal = None
        self.navigator = TurtleBot4Navigator()

        #self.build_p_map()
        # Create a timer that triggers every second
        self.timer = self.create_timer(0.5, self.run)

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")


    def build_p_map(self):
        self.get_logger().info("Building the map")
        
        self.map = dict()

        # TODO: load the map from a file instead of hardcoding it
        A = (57.5, -2)
        B = (56.5,-11.5)
        C = (38,-1)
        D = (38,-10)
        E = (17.5,-0.5)
        F = (17.2,-10)
        G = (-2.5,0)
        H = (-3.5,-9.5)
        I = (-22,0.5)
        J = (-22,-8.5)
        self.map[A] = [B, C]
        self.map[B] = [A, D]
        self.map[C] = [A, D, E]
        self.map[D] = [B, C, F]
        self.map[E] = [C, F, G]
        self.map[F] = [D, E, H]
        self.map[G] = [E, H, I]
        self.map[H] = [F, G, J]
        self.map[I] = [G, J]
        self.map[J] = [H, I]



    def run(self):
        if self.amcl_pose is not None:
            # print the pose
            self.get_logger().info(f"Pose: {self.amcl_pose.pose.pose.position.x}, {self.amcl_pose.pose.pose.position.y}")
        else:
            self.get_logger().info("Waiting for the pose")

     
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    info = PlannerHandler()
    executor.add_node(info)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()