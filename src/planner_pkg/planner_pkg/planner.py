import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped

#import DiscoveryService
import yaml
import random
import threading


class PlannerHandler(Node):

    def __init__(self):
        super().__init__("Info") # Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)

        # Subscribe to a service
        #self.client = self.create_client(DiscoveryService, 'discovery_mode')
        self.amcl_pose = None
        self.nav_thread = None
        self.navigator = TurtleBot4Navigator()

        # Wait for Nav2
        self.navigator.waitUntilNav2Active()


        self.build_p_map()
        # Wait for the initial pose
        while self.amcl_pose is None:
            self.get_logger().info("Waiting for the initial pose")
            rclpy.spin_once(self)

        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.get_logger().info('Connected to service: discovery_mode')
        # self.req = DiscoveryService.Request()
        

        self.next_goal = self.discovery_mode()

        # Create a timer that triggers every second
        self.timer = self.create_timer(0.5, self.run)

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")


    def build_p_map(self):
        self.get_logger().info("Building the map")
        
        self.map = dict()

        # with open("config.yaml") as file: # TODO: how to read the yaml file from a ros2 node?
        #     data = yaml.load(file, Loader=yaml.FullLoader)
        #     # take goals_coordinates from the yaml file and store it in variables A, B, C, D, E, F, G, H, I, J
        # goals_coordinates = data["goals_coordinates"]
        # A = goals_coordinates["A"]
        # B = goals_coordinates["B"]
        # C = goals_coordinates["C"]
        # D = goals_coordinates["D"]
        # E = goals_coordinates["E"]
        # F = goals_coordinates["F"]
        # G = goals_coordinates["G"]
        # H = goals_coordinates["H"]
        # I = goals_coordinates["I"]
        # J = goals_coordinates["J"]


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

    def send_request(self, goal_pose_x, goal_pose_y, direction):
        self.req.goal_pose_x = goal_pose_x
        self.req.goal_pose_y = goal_pose_y
        self.req.angle = direction
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def discovery_mode(self):
        self.get_logger().info("Discovery mode")
        
        # given the current pose of the robot, we need to find the nearest goal
        # get the current pose of the robot
        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        current_pose = (x, y)

        # find the nearest goal
        nearest_goal = None
        min_distance = float("inf")
        for goal in self.map:
            distance = (goal[0] - x) ** 2 + (goal[1] - y) ** 2
            if distance < min_distance:
                min_distance = distance
                nearest_goal = goal
        
        # select a random neighbor of the nearest goal
        neighbors = self.map[nearest_goal]
        next_goal = neighbors[random.randint(0, len(neighbors) - 1)]
        
        return next_goal

    def start_navigation(self, x, y):
        self.navigator.startToPose(self.navigator.getPoseStamped((x, y), TurtleBot4Directions.NORTH))

    def run(self):
        
        if self.amcl_pose is not None:
            if self.nav_thread is None or not self.nav_thread.is_alive():
                self.next_goal = self.discovery_mode()
                self.get_logger().info(f"***\n NEXT GOAL: {self.next_goal} \n***")
                x, y = map(float, self.next_goal)
                # Start the navigation in a separate thread
                self.nav_thread = threading.Thread(target=self.start_navigation, args=(x, y))
                self.nav_thread.start()


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