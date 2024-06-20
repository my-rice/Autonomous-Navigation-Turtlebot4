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
from sig_rec.qreader import QReader
from rclpy.callback_groups import ReentrantCallbackGroup

class Discovery(Node):
    def __init__(self, node_name, config_path=None, **kwargs):
        super().__init__(node_name, **kwargs)
        self._action_server = ActionServer(
            self,
            DiscoveryAction,
            'discovery_mode',
            self.discovery_mode_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info("Action Server 'discovery_mode' is ready")
        self.navigator = TurtleBot4Navigator()
        self.read_parameters(config_path)
        self.found = False
        self.road_sign = None
        self.amcl_pose = None
        self.parallel_group = ReentrantCallbackGroup()
        self.is_navigating = False
        # ROBADIADO
        self._bridge = cv_bridge.CvBridge()
        self._detector = QReader(model_size="n")
        self.active = False # METTI FALSE DOPO
        self._image_sub = self.create_subscription(CompressedImage, "/oakd/rgb/preview/image_raw/compressed", self.on_imageread, 10, callback_group=self.parallel_group) # METTI PREVIEW DOPO
        # self.sign_sub = self.create_subscription(String, "/test_sign", self.on_davide, 10)
        # ROBADIADO

        self.cancel_requested = False

        self.mutex = threading.Lock()
        self.mutex_sign = threading.Lock()
        # self.timer = self.create_timer(0.5, self.run)

        self.get_logger().info("Subscribed to topics")


    # def on_davide(self, msg):
    #     if self.active:
    #         self.found = True
    #         self.road_sign = msg.data
    #         self.navigator.cancelTask()
    #         self.get_logger().info(msg.data)

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        with self.mutex:
            if self.is_navigating:
                self.get_logger().info('Cancelling navigation thread')
                self.navigator.cancelTask()
                self.cancel_requested = True
                self.road_sign = "Canceled"
                self.found = True
            

        return CancelResponse.ACCEPT

    def read_parameters(self, config_path):
        if config_path is None:
            config_path = 'src/discovery_pkg/config/config.yaml'

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.policy = config['policy']['array_based']
        self.n_points = config['policy']['n_points']
        self.spin_dist = config['policy']['spin_dist']

    def policy_straight(self, goal_x, goal_y, angle, start_x, start_y, n_points=4):
        r = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        increment_x = (goal_x - start_x) / n_points
        increment_y = (goal_y - start_y) / n_points
        points = [(start_x + increment_x * i, start_y + increment_y * i, angle) for i in range(1, n_points)]
        return points

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points, spin_dist):
        actual_spin_dist = spin_dist
        self.get_logger().info(f"Starting navigation to {goal_x}, {goal_y} with angle {angle} from {start_x}, {start_y}")
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        for point in points:
            with self.mutex_sign:
                if(self.found):
                    break
            self.get_logger().info(f"Point: {point}")
            for i in [-1, 2, -1]:
                try:
                    with self.mutex:
                        self.navigator.spin(spin_dist=math.radians(actual_spin_dist * i))
                        self.is_navigating = True

                    while not self.navigator.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)
                    self.is_navigating = False
                    
                    rclpy.spin_once(self, timeout_sec=1.0)
                    with self.mutex_sign:
                        if self.found:
                            self.get_logger().info(f"Found road sign: {self.road_sign}")
                            break
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.road_sign = "Error"
                    break
            actual_spin_dist -= 5
            try:
                with self.mutex:
                    self.navigator.goToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                    self.is_navigating = True
                while not self.navigator.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)
                self.is_navigating = False
                with self.mutex_sign:
                    if self.found:
                        self.get_logger().info(f"Found road sign: {self.road_sign}")
                        break
            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                self.road_sign = "Error"
                break

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

        if self.cancel_requested:
            self.get_logger().info("Cancel requested")
            goal_handle.abort()
            self.cancel_requested = False
            result.next_action = "Canceled"
            self.active = False
            return result

        if self.road_sign is None:
            self.active = True
            increment_x = (goal.goal_pose_x - goal.start_pose_x)
            increment_y = (goal.goal_pose_y - goal.start_pose_y)
            self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x-increment_x/3, goal.start_pose_y-increment_y/3, self.n_points+2, self.spin_dist+10)
       
            
            self.get_logger().info("Ready to join navigation thread 2")
            self.active = False

        if self.cancel_requested:
            goal_handle.abort()
            self.cancel_requested = False
            result.next_action = "Canceled"
            self.active = False
            self.get_logger().info(f"Canceled Goal handle: {goal_handle}")
            return result

        result.next_action = (self.road_sign or "random").lower()
        self.get_logger().info(f"Result next action: {result.next_action}")
        goal_handle.succeed()
        self.get_logger().info(f"Goal handle: {goal_handle}")
        return result

    def on_imageread(self, msg):
        self.get_logger().info("Image received")
        if self.active and not self.found:
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
                            self.navigator.cancelTask()
                            self.get_logger().info("SEGNALE LETTO: "+ str(content))

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    discovery_node = Discovery("discovery_node")
    executor.add_node(discovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        discovery_node.destroy()
