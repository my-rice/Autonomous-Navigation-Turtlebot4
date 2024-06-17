import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from discovery_interface.action import DiscoveryAction
from rclpy.action import ActionServer
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
import yaml
import threading
from std_msgs.msg import String
import math
from rclpy.action import CancelResponse, GoalResponse
import time
import cv_bridge
from sensor_msgs.msg import CompressedImage
from sig_rec.qreader import QReader


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
        
        #ROBADIADO
        self._bridge = cv_bridge.CvBridge()
        self._detector = QReader(model_size="n")
        self.active = False
        self._image_sub = self.create_subscription(CompressedImage, "/oakd/rgb/preview/image_raw/compressed", self.on_imageread, 10)
        #ROBADIADO

        self.nav_thread = None
        self.cancel_requested = False
        # Create a subscription for the /tests topic
        #self.doublesub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.mutex = threading.Lock()


        self.get_logger().info("Subscribed to /output_command topic")



    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')

        with self.mutex:
            if self.nav_thread is not None and self.nav_thread.is_alive():
                self.get_logger().info('Cancelling navigation thread')
                self.navigator.cancelTask()
                #self.nav_thread.join()
                self.get_logger().info('Navigation thread cancelled')
                self.cancel_requested = True

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
        points = []

        for i in range(1,n_points):
            x = start_x + increment_x * i
            y = start_y + increment_y * i
            points.append((x, y, angle))
        return points

    def approach(self):
        raise NotImplementedError

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points=4, spin_dist=45):
        # points = self.policy_arc(goal_x, goal_y, angle, start_x, start_y)
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        for point in points:
            self.get_logger().info(f"Point: {point}")
            for i in [-1,2,-1]:
                try:
                    self.navigator.spin(spin_dist=math.radians(spin_dist*i))
                    while not self.navigator.isTaskComplete():
                      rclpy.spin_once(self, timeout_sec=1)
                    time.sleep(0.5)
                    if self.found == True:
                        self.get_logger().info(f"found road_sign: {self.road_sign}")
                        break
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.road_sign = "Error"
                    break
            try:
                # self.navigator.goToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                # while not self.navigator.isTaskComplete():
                #     rclpy.spin_once(self, timeout_sec=1) # TODO capire qual è meglio fra startToPose e goToPose
                self.navigator.startToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                if self.found == True:
                    self.get_logger().info(f"found road_sign: {self.road_sign}")
                    break
            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                self.road_sign = "Error"
                break
            
        self.get_logger().info("Navigation completed successfully")

    

    def discovery_mode_callback(self, goal_handle):
        self.active = True
        goal = goal_handle.request
        result = DiscoveryAction.Result()

        self.get_logger().info(f'Incoming request\n x: {goal.goal_pose_x} y: {goal.goal_pose_y} angle: {goal.angle} start_x: {goal.start_pose_x} start_y: {goal.start_pose_y}')

        self.found = False
        self.road_sign = None
        # self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y)
       
        # Start the navigation in a separate thread
        with self.mutex:
            self.nav_thread = threading.Thread(target=self.start_navigation, args=(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y, self.n_points, self.spin_dist))
            self.nav_thread.start()

        # Wait for the navigation to complete, allowing other callbacks to be processed
        self.nav_thread.join()  # Wait until the navigation thread completes
        self.nav_thread = None

        if self.cancel_requested:
            goal_handle.abort()
            self.cancel_requested = False
            result.next_action = "Canceled"
            self.active = False
            return result
        else:
            if(self.road_sign is None):
                with self.mutex:
                    increment_x = (goal.goal_pose_x - goal.start_pose_x)
                    increment_y = (goal.goal_pose_y - goal.start_pose_y) 
                    self.nav_thread = threading.Thread(target=self.start_navigation, args=(goal.start_pose_x-increment_x/3, goal.start_pose_y-increment_y/3, goal.angle, goal.goal_pose_x, goal.goal_pose_y, self.n_points+2, self.spin_dist+20))
                    self.nav_thread.start()
                self.nav_thread.join()  # Wait until the navigation thread completes
                self.nav_thread = None

            if self.cancel_requested:
                goal_handle.abort()
                self.cancel_requested = False
                result.next_action = "Canceled"
                self.active = False
                return result

            else:
                if(self.road_sign is None):
                    result.next_action = "random" 
                else:
                    result.next_action = (str(self.road_sign)).lower()
                        
                self.get_logger().info("result next action: " + result.next_action)
                goal_handle.succeed()
                return result


    def on_imageread(self, msg):
        if(self.active): 
            image_msg = msg
            cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
            decoded_qrs, locations = self._detector.detect_and_decode(
                image=cv_image, return_detections=True
            )
            if len(decoded_qrs) != 0:
                for content, location in zip(decoded_qrs, locations):
                    if(content != None):
                        self.found = True
                        self.road_sign = content
                        self.navigator.cancelTask()
                        self.get_logger().info(content)




def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    discovery_node = Discovery("discovery_node")
    executor.add_node(discovery_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        discovery_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()