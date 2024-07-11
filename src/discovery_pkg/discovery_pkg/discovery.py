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
        self.navigator.destroy_node()
        self._action_server.destroy()
        self._image_sub.destroy()
        self.sign_sub.destroy()
        super().destroy_node()

    def on_signal_received_test(self, msg):
        self.get_logger().info("Callback signal received")
        if self.active:
            self.found = True
            self.road_sign = msg.data
            self.navigator.cancelTask()
            self.get_logger().info(msg.data)

    def plot_point_on_rviz(self,test_points):
        """Plot the ideal relocation point on rviz."""
        marker = self.marker
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "test_points"
        marker.type = Marker.POINTS  # Change marker type to POINTS
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
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        with self.mutex:
            if self.is_navigating:
                self.get_logger().info('Cancelling navigation thread')
                self.navigator.cancelTask()
            self.road_sign = "Canceled"
            self.found = True
            

        return CancelResponse.ACCEPT

    def read_parameters(self, config_path):
        if config_path is None:
            config_path = 'src/discovery_pkg/config/config.yaml'

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.n_points = config['policy']['n_points']
        self.spin_dist = config['policy']['spin_dist']
        self.decrease_spin_dist = config['policy']['decrease_spin_dist']

    def policy_straight(self, goal_x, goal_y, angle, start_x, start_y, n_points=4):
        
        increment_x = (goal_x - start_x) / n_points
        increment_y = (goal_y - start_y) / n_points
        points = [(start_x + increment_x * i, start_y + increment_y * i, angle) for i in range(1, n_points)]
        points_2 = [(start_x + increment_x * i, start_y + increment_y * i, angle) for i in range(0, n_points-1)]
        
        self.plot_point_on_rviz(points_2)

        return points

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points, spin_dist):
        actual_spin_dist = spin_dist
        self.get_logger().info(f"Starting navigation to {goal_x}, {goal_y} with angle {angle} from {start_x}, {start_y}")
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        for num_point,point in enumerate(points):
            with self.mutex_sign:
                if(self.found):
                    return
            self.get_logger().info(f"Point: {point}")
            for i in [-1, 2, -1]:
                try:
                    with self.mutex:
                        self.get_logger().info("Primo spin")
                        self.navigator.spin(spin_dist=math.radians(actual_spin_dist * i))
                        self.is_navigating = True
                    self.get_logger().info("Primo while")
                    while not self.navigator.isTaskComplete():
                        self.get_logger().info("Waiting for spin task completion 1")
                        # rclpy.spin_once(self, timeout_sec=0.1)
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
            if num_point != len(points)-1:
                try:
                    with self.mutex:
                        self.navigator.goToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                        self.is_navigating = True
                    while not self.navigator.isTaskComplete():
                        self.get_logger().info("Waiting for navigation task completion 2")
                            # rclpy.spin_once(self, timeout_sec=0.1)
                    self.is_navigating = False
                    with self.mutex_sign:
                        if self.found:
                            self.get_logger().info(f"Found road sign: {self.road_sign}")
                            return
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.road_sign = "Error"
                    return

        self.get_logger().info("Navigation completed successfully")

    def discovery_mode_callback(self, goal_handle):
        self.get_logger().info("Discovery mode callback")
        self.active = True
        goal = goal_handle.request
        result = DiscoveryAction.Result()

        self.get_logger().info(f'Incoming request\n x: {goal.goal_pose_x} y: {goal.goal_pose_y} angle: {goal.angle} start_x: {goal.start_pose_x} start_y: {goal.start_pose_y}')

        self.found = False
        self.road_sign = None
        self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y, self.n_points, self.spin_dist)
        
        self.active = False

        if goal_handle.is_cancel_requested:
            self.get_logger().info("Cancel requested")
            goal_handle.canceled()
            result.next_action = "Canceled"
            self.active = False

            return result

        if self.road_sign is None:
            increment_x = (goal.goal_pose_x - goal.start_pose_x)
            increment_y = (goal.goal_pose_y - goal.start_pose_y)
            if(goal.start_discovery):
                start_pose_x = goal.start_pose_x
                start_pose_y = goal.start_pose_y
            else:
                start_pose_x = goal.start_pose_x-increment_x/3
                start_pose_y = goal.start_pose_y-increment_y/3
            self.navigator.goToPose(self.navigator.getPoseStamped((start_pose_x,start_pose_y), goal.angle))
            while not self.navigator.isTaskComplete():
                        self.get_logger().info("Repositionating the robot before crossing entrance")
            self.active = True
            self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, start_pose_x, start_pose_y, self.n_points+2, self.spin_dist+10)
       
            
            self.get_logger().info("Ready to join navigation thread 2")
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
        self.get_logger().info(f"Goal handle: {goal_handle}")
        return result

    def on_image_read(self, msg):
        # self.get_logger().info("Callback image read")
        if self.active and not self.found:
            # self.get_logger().info("Image received")

            image_msg = msg
            cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
            decoded_qrs, locations = self._detector.detect_and_decode(
                image=cv_image, return_detections=True
            )
            if decoded_qrs:
                for content, location in zip(decoded_qrs, locations):
                    if content:
                        self.get_logger().info("Ready to take mutex")
                        with self.mutex_sign:
                            self.found = True
                            self.road_sign = content
                            self.get_logger().info("SEGNALE LETTO: "+ str(content))
                            self.navigator.cancelTask()
                        self.get_logger().info("Mutex released")


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    discovery_node = Discovery("discovery_node")
    executor.add_node(discovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        discovery_node.get_logger().info("Keyboard Interrupt (SIGINT)")
    except Exception:
        discovery_node.get_logger().info("Exception")
    discovery_node.destroy_node()
    
