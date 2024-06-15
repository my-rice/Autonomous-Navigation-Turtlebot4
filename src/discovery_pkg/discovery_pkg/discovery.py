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
        self.signal = None
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

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points=4):
        # points = self.policy_arc(goal_x, goal_y, angle, start_x, start_y)
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        for point in points:
            self.get_logger().info(f"Point: {point}")
            for i in [-1,2,-1]:
                try:
                    self.navigator.spin(spin_dist=math.radians(self.spin_dist*i))
                    while not self.navigator.isTaskComplete():
                      rclpy.spin_once(self, timeout_sec=1)
                    if self.found == True:
                        self.get_logger().info(f"found signal: {self.signal}")
                        break
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    # self.signal = "Error"
                    break
            try:
                # self.navigator.goToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                # while not self.navigator.isTaskComplete():
                #     rclpy.spin_once(self, timeout_sec=1) # TODO capire qual è meglio fra startToPose e goToPose
                self.navigator.startToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]))
                if self.found == True:
                    self.get_logger().info(f"found signal: {self.signal}")
                    break
            except Exception as e:
                self.get_logger().info(f"Error: {e}")
                # self.signal = "Error"
                break

        # if(self.signal is None):
        #     #approach
        #     approach = self.approach()
        self.get_logger().info("Navigation completed successfully")

    

    def discovery_mode_callback(self, goal_handle):
        self.active = True
        goal = goal_handle.request
        result = DiscoveryAction.Result()

        self.get_logger().info(f'Incoming request\n x: {goal.goal_pose_x} y: {goal.goal_pose_y} angle: {goal.angle} start_x: {goal.start_pose_x} start_y: {goal.start_pose_y}')

        self.found = False
        self.signal = None
        # self.start_navigation(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y)
       
        # Start the navigation in a separate thread
        with self.mutex:
            self.nav_thread = threading.Thread(target=self.start_navigation, args=(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y, self.n_points))
            self.nav_thread.start()

        # Wait for the navigation to complete, allowing other callbacks to be processed
        self.nav_thread.join()  # Wait until the navigation thread completes
        self.nav_thread = None
        self.activate = False
        if self.cancel_requested:
            goal_handle.abort()
            self.cancel_requested = False
            result.next_action = "Canceled"
            return result
        else:
            # if(self.signal=='None'):
            #     result.next_action = "None"
            # elif(self.signal is not None):
            #     result.next_action = self.signal
            # else:
            #     result.next_action = "Error"
            if(self.signal is None):
                result.next_action = "right" # TODO Da fare random
            else:
                result.next_action = (str(self.signal)).lower()
                    
            self.get_logger().info("result next action: " + result.next_action)
            goal_handle.succeed()
            return result


    def on_imageread(self, msg):
        if(self.active): 
            image_msg = msg
            cv_image = self._bridge.compressed_imgmsg_to_cv2(image_msg)
            """
            qrdetector = cv.QRCodeDetector()
            ret,points,code = qrdetector.detectAndDecode(cv_image)
            out = String()
            out.data = ret
            self.get_logger().info(ret)
            self._command_pub.publish(out)
            """
            decoded_qrs, locations = self._detector.detect_and_decode(
                image=cv_image, return_detections=True
            )
            # Print the results
            #print(f"Image: {frame} -> {len(decoded_qrs)} QRs detected.")
            if len(decoded_qrs) == 0:
                out = String()
                out.data = "NOCODE"
                self.get_logger().info(out.data)
                self._command_pub.publish(out)
            else:
                for content, location in zip(decoded_qrs, locations):
                    #print(f"Content: {content}. Position: {tuple(location[BBOX_XYXY])}"
                    out = String()
                    out.data = "None" if (content is None) else content
                    if(out.data == "None"):
                        pass # FARE QUALCOSA
                    elif(out.data != "NOCODE"):
                        self.found = True
                        self.signal = out.data
                        self.navigator.cancelTask()
                    self.get_logger().info(out.data)
                    self._command_pub.publish(out)




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