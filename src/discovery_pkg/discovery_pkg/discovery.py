import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from discovery_interface.action import DiscoveryAction
from rclpy.action import ActionServer
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import yaml
import threading
import math
from rclpy.action import CancelResponse, GoalResponse
import time
import cv_bridge
from sensor_msgs.msg import CompressedImage
from discovery_pkg.qreader import QReader
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Discovery(Node):
    """
    This class is the main class for the discovery mode, it handles the action server and the navigation of the robot based
    on a given policy. It also handles the image processing and the road sign detection.
    """
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.mutual_exclusion_group = MutuallyExclusiveCallbackGroup()

        self._action_server = ActionServer(
            self,
            DiscoveryAction,
            'discovery_mode',
            self.discovery_mode_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.mutual_exclusion_group)
        
        self.get_logger().info("Action Server 'discovery_mode' is ready")
        self.navigator = TurtleBot4Navigator()

        self.declare_parameter('config_file', '')
        self.config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        self.read_parameters(self.config_path)
        self.found = False
        self.road_sign = None
        self.amcl_pose = None
        self.parallel_group = MutuallyExclusiveCallbackGroup()
        self.is_navigating = False

        self._bridge = cv_bridge.CvBridge()
        self._detector = QReader(model_size="n")
        self.active = False 
        self._image_sub = self.create_subscription(CompressedImage, "/oakd/rgb/preview/image_raw/compressed", self.on_image_read, 10, callback_group=self.parallel_group) 
        self.sign_sub = self.create_subscription(String, "/test_sign", self.on_signal_received_test, 10,callback_group=self.parallel_group)


        self.mutex = threading.Lock()
        self.mutex_sign = threading.Lock()
        self.decrease_spin_dist = 0

        self.publisher_marker = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker = Marker()
        self.get_logger().info("Subscribed to topics")

    def destroy_node(self):
        """ Destroy the node."""
        self.navigator.destroy_node()
        self._action_server.destroy()
        self._image_sub.destroy()
        self.sign_sub.destroy()
        super().destroy_node()

    def on_signal_received_test(self, msg):
        """Test callback for signal received used for testing in simulation."""
        self.get_logger().info("Test callback signal received")
        if self.active:
            self.found = True
            self.road_sign = msg.data
            self.navigator.cancelTask()
            self.get_logger().info(msg.data)

    def plot_point_on_rviz(self,test_points):
        """Plot points on rviz."""
        marker = self.marker
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_points"
        marker.type = Marker.POINTS  
        marker.action = Marker.ADD

        points = []
        for point in test_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            self.get_logger().info(f"Point: {p.x}, {p.y}")
            p.z = 0.0
            points.append(p)

        marker.points = points

        marker.scale.x = 0.2  # Point width
        marker.scale.y = 0.2  # Point height

        # Set color for the points
        marker.color.a = 1.0  # Transparency
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0  # Green color
        marker.color.b = 0.0  # Blue color

        self.publisher_marker.publish(marker)


    def goal_callback(self, goal_request):
        """Goal callback for the action server."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Cancel callback for the action server."""
        self.get_logger().info('Received cancel request')
        with self.mutex:
            if self.is_navigating:
                self.get_logger().info('Cancelling navigation thread')
                self.navigator.cancelTask()
            self.road_sign = "Canceled"
            self.found = True
            

        return CancelResponse.ACCEPT

    def read_parameters(self, config_path):
        """Read parameters from the config file."""
        if config_path is None:
            config_path = 'src/discovery_pkg/config/config.yaml'

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.n_points = config['policy']['n_points']
        self.spin_dist = config['policy']['spin_dist']
        self.decrease_spin_dist = config['policy']['decrease_spin_dist']

    def policy_straight(self, goal_x, goal_y, angle, start_x, start_y, n_points=4):
        """Policy for straight navigation."""
        increment_x = (goal_x - start_x) / n_points
        increment_y = (goal_y - start_y) / n_points
        points = [(start_x + increment_x * i, start_y + increment_y * i, angle) for i in range(1, n_points)]
        points_to_visualize = [(start_x + increment_x * i, start_y + increment_y * i, angle) for i in range(0, n_points-1)]
        
        # self.plot_point_on_rviz(points_to_visualize)

        return points

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points, spin_dist):
        """"This method implements the exploration policy for the robot. It navigates the robot via a series of intermediate points. It also spins the robot to detect road signs inside the crossing area."""
        actual_spin_dist = spin_dist
        self.get_logger().info(f"Starting navigation to {goal_x}, {goal_y} with angle {angle} from {start_x}, {start_y}")
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        # Loop through the intermediate points, until a road sign is found or the policy is completed
        for num_point,point in enumerate(points):
            with self.mutex_sign:
                if(self.found):
                    return
            self.get_logger().info(f"Intermediate point: {point}")
            # For each intermediate point, spin the robot left and right to detect road signs
            for i in [-1, 2, -1]:
                try:
                    with self.mutex:
                        self.get_logger().info("Start spinning.")
                        self.navigator.spin(spin_dist=math.radians(actual_spin_dist * i))
                        self.is_navigating = True
                    while not self.navigator.isTaskComplete():
                        self.get_logger().info("Spinning...")
                    self.is_navigating = False
                    
                    time.sleep(1)
                    with self.mutex_sign:
                        if self.found:
                            self.get_logger().info(f"Found road sign: {self.road_sign}")
                            return
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.road_sign = "Error"
                    return
            actual_spin_dist -= self.decrease_spin_dist
            # Navigate to the next intermediate point
            if num_point != len(points)-1:
                try:
                    with self.mutex:
                        self.navigator.goToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                        self.is_navigating = True
                    while not self.navigator.isTaskComplete():
                        self.get_logger().info("Navigating to next point...")
                    self.is_navigating = False
                    with self.mutex_sign:
                        if self.found:
                            self.get_logger().info(f"Found road sign: {self.road_sign}")
                            return
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.road_sign = "Error"
                    return


    def discovery_mode_callback(self, goal_handle):
        """ Callback for the discovery mode action server. It handles the action server requests and starts the navigation."""
        self.get_logger().info("Discovery mode callback")
        self.active = True
        goal = goal_handle.request
        result = DiscoveryAction.Result()

        self.get_logger().info(f'Incoming request\n x: {goal.goal_pose_x} y: {goal.goal_pose_y} angle: {goal.angle} start_x: {goal.start_pose_x} start_y: {goal.start_pose_y}')

        self.found = False
        self.road_sign = None
        # Exploration policy for the robot based on the given goal and start points and angle
        self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y, self.n_points, self.spin_dist)
        
        self.active = False
        # If the goal is canceled, return the result with the next action as "Canceled"
        if goal_handle.is_cancel_requested:
            self.get_logger().info("Cancel requested")
            goal_handle.canceled()
            result.next_action = "Canceled"
            self.active = False

            return result
        # If a road sign is not found during the exploration policy, do the backup policy: more intermediate points and more spinning
        if self.road_sign is None:
            increment_x = (goal.goal_pose_x - goal.start_pose_x)
            increment_y = (goal.goal_pose_y - goal.start_pose_y)
            if(goal.start_discovery):
                start_pose_x = goal.start_pose_x
                start_pose_y = goal.start_pose_y
            else:
                start_pose_x = goal.start_pose_x-increment_x/4
                start_pose_y = goal.start_pose_y-increment_y/4
            
            self.is_navigating = True
            self.navigator.goToPose(self.navigator.getPoseStamped((start_pose_x,start_pose_y), goal.angle))
            while not self.navigator.isTaskComplete():
                self.get_logger().info("Repositionating the robot before crossing entrance...")
            self.is_navigating = False
            self.active = True
            goal_pose_x = goal.goal_pose_x-increment_x/6
            goal_pose_y = goal.goal_pose_y-increment_y/6


            self.start_navigation(goal_pose_x, goal_pose_y, goal.angle, start_pose_x, start_pose_y, self.n_points+1, self.spin_dist+10)
       
            
            self.active = False

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.next_action = "Canceled"
            self.active = False
            self.get_logger().info(f"Canceled Goal handle: {goal_handle}")
            return result

        result.next_action = (self.road_sign or "random").lower()
        self.get_logger().info(f"Result next action: {result.next_action}")
        goal_handle.succeed()
        return result

    def on_image_read(self, msg):
        self.get_logger().info("Callback image read")
        if self.active and not self.found:
            self.get_logger().info("Image received")

            image_msg = msg
            cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
            decoded_qrs, locations = self._detector.detect_and_decode(
                image=cv_image, return_detections=True
            )
            if decoded_qrs:
                for content, location in zip(decoded_qrs, locations):
                    if content:
                        with self.mutex_sign:
                            self.found = True
                            self.road_sign = content
                            self.get_logger().info("Road sign detected: "+ str(content))
                            self.navigator.cancelTask()


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    discovery_node = Discovery("discovery_node")
    executor.add_node(discovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        discovery_node.get_logger().info("Keyboard Interrupt , shutting down...")
    except Exception:
        discovery_node.get_logger().info("Exception, shutting down...")
    discovery_node.destroy_node()
    
