
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
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
from rclpy.action import CancelResponse, GoalResponse

class Discovery(Node):
    def __init__(self, node_name, config_path=None, **kwargs):
        super().__init__(node_name, **kwargs)
        self._action_server = ActionServer(
                    self,
                    DiscoveryAction,
                    'discovery_mode',    
                    self.discovery_mode_callback,
                    goal_callback=self.goal_callback,
                    cancel_callback=self.cancel_callback
                )
        self.get_logger().info("Action Server 'discovery_mode' is ready")
        self.navigator = TurtleBot4Navigator()
        self.read_parameters(config_path)
        self.founded = False
        self.signal = None
        self.amcl_pose = None
        self.mutex = threading.Lock()
        # Create a subscription for the /tests topic
        self.sub = self.create_subscription(String, '/output_command', self.signal_callback, 10)
        self.doublesub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.in_discovery = False
        self.get_logger().info("Subscribed to /output_command topic")

        self.nav_thread = None
        self.cancel_requested = False

        self.i = 0 # TODO : Da rimuovere

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

    # def policy_arc(self, goal_x, goal_y, angle, start_x, start_y): # TODO : Aggiungere dove spostarsi prima in base all'incrocio
    #     r = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
    #     increment_x = (goal_x - start_x) / 4
    #     increment_y = (goal_y - start_y) / 4
    #     points = []

    #     for i in range(4):
    #         x = start_x + increment_x * i
    #         y = start_y + increment_y * i
    #         points.append((x, y, angle))

    #     theta = math.atan2(goal_y - start_y, goal_x - start_x)
    #     angles = [math.radians(angle) for angle in [22.5, 45, 67.5, -22.5, -45, -67.5]]

    #     for angle_offset in angles:
    #         arc_angle = theta + angle_offset
    #         x = start_x + r * math.cos(arc_angle)
    #         y = start_y + r * math.sin(arc_angle)
    #         points.append((x, y, angle + angle_offset))
    #     self.get_logger().info(f"Points: {points}")
    #     return points
    
    def policy_straight(self, goal_x, goal_y, angle, start_x, start_y, n_points=4):
        r = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        increment_x = (goal_x - start_x) / n_points
        increment_y = (goal_y - start_y) / n_points
        points = []

        for i in range(n_points):
            x = start_x + increment_x * i
            y = start_y + increment_y * i
            points.append((x, y, angle))
        return points

    def approach(self):
        raise NotImplementedError

    def start_navigation(self, goal_x, goal_y, angle, start_x, start_y, n_points):
        # points = self.policy_arc(goal_x, goal_y, angle, start_x, start_y)
        points = self.policy_straight(goal_x, goal_y, angle, start_x, start_y, n_points)
        for point in points:
            self.get_logger().info(f"Point: {point}")
            for i in [0,-1,1]:
                try:
                    self.navigator.startToPose(self.navigator.getPoseStamped((point[0], point[1]), point[2]+i*65))
                    time.sleep(0.5)
                    if self.founded == True:
                        self.get_logger().info(f"Founded signal: {self.signal}")
                        break
                except Exception as e:
                    self.get_logger().info(f"Error: {e}")
                    self.signal = "Error"
                    break
            time.sleep(0.5)


        #self.navigator.startToPose(self.navigator.getPoseStamped((goal_x, goal_y), angle))
        # if(self.signal is None):
        #     #approach
        #     approach = self.approach()
        self.get_logger().info("Navigation completed successfully")

    

    def discovery_mode_callback(self, goal_handle):
        self.in_discovery = True
        goal = goal_handle.request
        # self.get_logger().info(f'Incoming request\n x: {goal.goal_pose_x} y: {goal.goal_pose_y} angle: {goal.angle} start_x: {goal.start_pose_x} start_y: {goal.start_pose_y}')

        self.founded = False
        self.signal = None

        # # Start the navigation in a separate thread
        with self.mutex:
            self.nav_thread = threading.Thread(target=self.start_navigation, args=(goal.goal_pose_x, goal.goal_pose_y, goal.angle, goal.start_pose_x, goal.start_pose_y, self.n_points))
            self.nav_thread.start()

            # Wait for the navigation to complete, allowing other callbacks to be processed
        self.nav_thread.join()  # Wait until the navigation thread completes
        self.nav_thread = None
        if self.cancel_requested:
            goal_handle.abort()
            self.cancel_requested = False
            return DiscoveryAction.Result()
        else:
            self.get_logger().info("Discovery mode callback")
            result = DiscoveryAction.Result()
            self.get_logger().info("Discovery mode callback 2")
            goal_handle.succeed()
            self.get_logger().info("Discovery mode callback 3")

            # if(self.signal=='None'):
            #     result.next_action = "None"
            # elif(self.signal is not None):
            #     result.next_action = self.signal
            # else:
            #     result.next_action = "Error"

            signs =  ["right", "right", "stop"]
            result.next_action = signs[self.i]
            self.i += 1
            # result.next_action = (str(self.signal)).lower()
            # if(self.signal is None):
            #     result.next_action = "straighton"
            self.in_discovery = False
        return result



    def signal_callback(self, msg):
        if(self.in_discovery == True):
            #self.get_logger().info(f'Received signal: {msg.data}')

            if msg.data == "None":
                # self.founded = True
                # self.navigator.cancelTask()
                pass
            elif msg.data != "NOCODE":
                self.get_logger().info(f'Received signal: {msg.data}')
                self.signal = msg.data  
                self.founded = True
                self.navigator.cancelTask()
            else:

                pass

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        
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
