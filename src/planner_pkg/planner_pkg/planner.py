import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import KidnapStatus

# import boolean msg
from std_msgs.msg import Bool

from rclpy.action import ActionClient
from discovery_interface.action import DiscoveryAction
import yaml
import random
import threading
import time
import math
from pyquaternion import Quaternion


class GoalNotValidException(Exception):
    """Exception raised when the goal is not valid."""

    def __init__(self, message="Goal is not valid"):
        self.message = message
        super().__init__(self.message)

class ExitException(Exception):
    """Exception raised when the robot has reached the final goal."""

    def __init__(self, message="The robot has reached the final goal"):
        self.message = message
        super().__init__(self.message)


class DiscoveryActionClient(Node):
    def __init__(self):
        super().__init__('discovery_action_client')
        self.action_client = ActionClient(self, DiscoveryAction, 'discovery_mode')

    def send_goal(self,goal_pose_x, goal_pose_y, start_pose_x, start_pose_y, angle):

        goal_msg = DiscoveryAction.Goal()
        goal_msg.goal_pose_x = goal_pose_x
        goal_msg.goal_pose_y = goal_pose_y
        goal_msg.start_pose_x = start_pose_x
        goal_msg.start_pose_y = start_pose_y
        goal_msg.angle = angle
        
        self.get_logger().info('Action server is ready, sending goal ...')
        self.current_goal_handle = self.action_client.send_goal_async(goal_msg)  
        return self.current_goal_handle

    def cancel_goal(self):
        if self.current_goal_handle is not None:
            self.action_client.cancel_goal_async(self.current_goal_handle)
            self.get_logger().info('Goal cancelled.')

    def wait_for_server(self, timeout_sec=1.0):
        if(self.action_client.wait_for_server(timeout_sec)):
            return True
        return False

    def get_result_callback(self, future_handle):
        result = future_handle.result().result
        self.get_logger().info('Result: {0}'.format(result.next_action))
        return result.next_action


class PlannerHandler(Node):

    def __init__(self):
        super().__init__("Info") # Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        
        
        # self.kidnapped_sub = self.create_subscription(KidnapStatus, "/kidnap_status", self.kidnapped_callback, 10)
        
        # TOPIC FOR TESTING PURPOSES: to test the recovery mode, we need to kidnap the robot
        self.test_sub = self.create_subscription(Bool, "/test", self.test_callback, 10)
        

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create an action client for the discovery mode
        self.discovery_action_client = DiscoveryActionClient()
        self.declare_parameter('config_file', '')
        self.config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.get_logger().info("Config file: " + self.config_path)
        self.read_parameters()
        print("Timer period: ", self.timer, "Timeout: ", self.timeout)
        self.amcl_pose = None
        self.nav_thread = None
        self.navigator = TurtleBot4Navigator()
        self.initial_pose_flag = False
        self.first_discovery = True
        self.is_kidnapped = False
        self.flag = True


        self.last_kidnapped = False

        self.build_p_map()
        # Wait for Nav2.

        while self.initial_pose_flag == False:
            rclpy.spin_once(self, timeout_sec=1)

        self.navigator.waitUntilNav2Active()

        if(self.discovery_action_client.wait_for_server(self.timeout)):
            self.get_logger().info("Action server is ready")
        else:
            self.get_logger().info("Action server is not ready, shutting down...")
            rclpy.shutdown() #TODO: raise exception and exit
            exit()
       
        self.timer = self.create_timer(self.timer, self.run)

    def read_parameters(self):
        self.get_logger().info("Reading parameters from the config file: " + self.config_path)
        with open(self.config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.timer = config['planner_parameters']['timer_period']
        self.timeout = config['planner_parameters']['timeout']
        self.nodes = config['map']['nodes']
        self.connections = config['map']['connections']

    def pose_callback(self, msg):
        # if(not self.initial_pose_flag):
        #     self.initial_pose_flag = True
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        

    def kidnapped_callback(self, msg):
        self.is_kidnapped = msg.is_kidnapped
        self.get_logger().info("Kidnapped status: " + str(self.is_kidnapped))
        
    def listener_callback(self, msg):

        if self.initial_pose_flag == False:
            self.initial_pose_flag = True
            angle = self.get_angle(msg.pose.pose.orientation)
            initial_pose = self.navigator.getPoseStamped([msg.pose.pose.position.x, msg.pose.pose.position.y], angle)
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info('Initial pose received: {0} {1} {2}'.format(msg.pose.pose.position.x, msg.pose.pose.position.y, angle))

    def test_callback(self, msg):
        self.is_kidnapped = msg.data
        self.get_logger().info("Kidnapped status: " + str(self.is_kidnapped))

        if self.is_kidnapped == True and self.last_kidnapped == False:
            # if the robot has been kidnapped, then we need to restart the navigation from a specific point. This is not a proper way to handle the kidnapped mode but it is a recovery mode by kidnapping the robot
            self.get_logger().info("The robot is in kidnapped mode")
            self.abort() # abort the current goal

        if self.is_kidnapped == False and self.last_kidnapped == True:
            self.get_logger().info("The robot is deployed")
            self.relocate()     
        self.last_kidnapped = self.is_kidnapped



    def build_p_map(self):
        self.get_logger().info("Building the map")
        
        self.map = dict()

        coordinates_lookup = {key: tuple(value['coordinates']) for key, value in self.nodes.items()}
        
        for key, value in self.connections.items():
            coord_key = coordinates_lookup[key]
            self.map[coord_key] = [(coordinates_lookup[conn['point']], conn['direction']) for conn in value]




    def compute_next_angle(self, current_angle, result):
        if result == "right":
            current_angle -= 90
        elif result == "left":
            current_angle += 90
        elif result == "goback":
            current_angle += 180

        if current_angle >= 360:
            current_angle -= 360
        if current_angle < 0:
            current_angle += 360
        return current_angle
    
    def action_result2goal(self,result, initial_pose, current_goal_pose):

        # approximate the angle to the nearest 90 degrees
        next_goal_angle = initial_pose[2]
        self.get_logger().info("The initial pose is: " + str(initial_pose))
        self.get_logger().info("THe result is: " + result)
        # convert frame angle to standard frame (remove the offset)
        next_goal_angle = next_goal_angle + 90

        self.get_logger().info("The standard angle is: " + str(next_goal_angle))

        next_goal_angle = self.compute_next_angle(next_goal_angle, result)

        if next_goal_angle < 45 or next_goal_angle >= 315: # TODO: change this to directions N, E, S, W
            next_goal_angle = "right"
        elif next_goal_angle < 135:
            next_goal_angle = "up"
        elif next_goal_angle < 225:
            next_goal_angle = "left"
        elif next_goal_angle < 315:
            next_goal_angle = "down"
        self.get_logger().info("Current goal pose: " + str(current_goal_pose))
    
        self.get_logger().info("The next goal angle is: " + str(next_goal_angle))


        neighbors = self.map[current_goal_pose]
        for neighbor in neighbors:
            if neighbor[1] == next_goal_angle:
                self.get_logger().info("The next goal is: " + str(neighbor[0]) + "given that the robot is in the pose: " + str(initial_pose) + " and the result is: " + result)
                return neighbor[0]
        

        raise GoalNotValidException("The goal is not valid, the road sign is: " + result + " the robot pose is (" + str(initial_pose[0]) + ", " + str(initial_pose[1]) + ", " + str(initial_pose[2]) + ") and the goal is (" + str(current_goal_pose[0]) + ", " + str(current_goal_pose[1]) + ")"+ " the neighbors are: " + str(neighbors)+ " the next goal angle is: " + str(next_goal_angle))

    def order_by_distance(self, intersection_points, x, y):
        x1, y1 = intersection_points[0]
        x2, y2 = intersection_points[1]
        distance1 = (x1 - x) ** 2 + (y1 - y) ** 2
        distance2 = (x2 - x) ** 2 + (y2 - y) ** 2
        if distance1 < distance2:
            return intersection_points
        else:
            return intersection_points[::-1] # reverse the list

    def discovery_mode(self,current_pose):

        self.get_logger().info("The robot is in discovery mode")
        
        x = current_pose[0]
        y = current_pose[1]
        theta = current_pose[2]

        # find the nearest goal if it is the first discovery
        if self.first_discovery == True:
            
            min_distance = float("inf")
            for goal in self.map:
                distance = (goal[0] - x) ** 2 + (goal[1] - y) ** 2
                if distance < min_distance:
                    min_distance = distance
                    self.next_goal = goal
        
        
        if(self.first_discovery == True):
            self.get_logger().info("The first discovery")
            discovery_goal_intersects, angle = self.get_intersection_points(self.next_goal, theta, "straighton")
            points = self.order_by_distance(discovery_goal_intersects, x, y)
            self.action_payload = (points[1][0],points[1][1],angle)
            self.first_discovery = False
            self.get_logger().info("THe action payload is: " + str(self.action_payload )+ "and the nearest goal is: " + str(self.next_goal))

        self.get_logger().info("sending the goal to the discovery action server: " + str(self.action_payload) + " and the current pose is: " + str(current_pose))
        # send the goal to the discovery action server and wait for the result

        future_goal = self.discovery_action_client.send_goal(float(self.action_payload[0]), float(self.action_payload[1]), float(x), float(y), float(self.action_payload[2]))
        rclpy.spin_until_future_complete(self.discovery_action_client, future_goal)

        self.get_logger().info("spin 2")
        goal_handle = future_goal.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.discovery_action_client, get_result_future)
        self.get_logger().info("spin 3")
        result = get_result_future.result().result.next_action
        self.get_logger().info("spin 4" + result)
        return result
        
    def get_intersection_points(self, next_goal, theta, result="straighton",rho=1):
        x1,y1 = next_goal

        xc, yc = next_goal

        # Convert frame angle to standard frame (remove the offset)
        standard_angle = theta #+ 90
        standard_angle = self.compute_next_angle(standard_angle, result)

        # Portiamo l'angolo tra 0 e 360 gradi
        angle = standard_angle #- 90
        if angle < 0:
            angle += 360
        if angle >= 360:
            angle -= 360

        if standard_angle == 90:
            x_intersect1 = xc
            y_intersect1 = yc + rho
            x_intersect2 = xc
            y_intersect2 = yc - rho
            intersection_points = [(x_intersect1, y_intersect1), (x_intersect2, y_intersect2)]
            return intersection_points, angle

        self.get_logger().info(f"Standard angle: {standard_angle}")
        m = math.tan(math.radians(standard_angle))
        q = y1 - m * x1
        self.get_logger().info(f"m: {m}, q: {q}")

        # Coefficienti dell'equazione quadratica
        A = 1 + m**2
        B = 2 * (m * q - m * yc - xc)
        C = xc**2 + yc**2 + q**2 - 2 * yc * q - rho**2

        # Risolvi l'equazione quadratica per trovare x
        delta = B**2 - 4 * A * C

        if delta < 0:
            raise ValueError("No intersection points")

        x_intersect1 = (-B + math.sqrt(delta)) / (2 * A)
        x_intersect2 = (-B - math.sqrt(delta)) / (2 * A)

        # Calcola i punti di intersezione corrispondenti su y
        y_intersect1 = m * x_intersect1 + q
        y_intersect2 = m * x_intersect2 + q

        # Riportare i punti nel sistema originale (x verso l'alto, y verso sinistra)
        intersection_points = [(x_intersect1,y_intersect1), (x_intersect2, y_intersect2)]
        self.get_logger().info(f"Intersection points: {intersection_points} and angle: {angle}")
        return intersection_points, angle

    def start_navigation(self, x, y, angle=0):
        self.navigator.startToPose(self.navigator.getPoseStamped((x, y), angle))

    def get_angle(self, quaternion):
        q = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        euler = q.yaw_pitch_roll
        yaw = euler[0]
        # convert the yaw angle to degrees
        theta = yaw * 180 / 3.14159265359
        return theta

    def relocate(self):
        self.get_logger().info("Relocating the robot")
        ### Get the ideal relocation pose of the robot
        points, angle = self.get_intersection_points(self.last_goal,self.nav_goal[2],"straighton",rho=4)
        points = self.order_by_distance(points, self.nav_goal[0], self.nav_goal[1])
        self.ideal_relocation = points[0]
    
        self.navigator.setInitialPose(self.navigator.getPoseStamped(self.ideal_relocation, angle))
        self.get_logger().info("The ideal relocation is: " + str(self.ideal_relocation))
        

    # def kidnapped_mode(self):
        # self.relocate() # TODO: change the name of the function
        # while self.is_kidnapped:
        #     self.get_logger().info("The robot is still kidnapped")
        #     time.sleep(0.5)
        
        # self.get_logger().info("The robot is no longer kidnapped")
        # self.flag = True
        # if the robot has been kidnapped, then we need to recompute the current location


    def abort(self):
        # Cancel the current goal
        self.get_logger().info("Aborting the current goal")
        if self.nav_thread is not None:
            self.navigator.cancelTask()
            self.nav_thread.join()
            self.nav_thread = None
            self.flag = True



    def run(self):
        
        if self.initial_pose_flag == False or self.is_kidnapped:
            return        

        if self.flag and (self.nav_thread is None or not self.nav_thread.is_alive()):
            self.flag = False #TODO change the name of the flag, it is not clear!!! 


            # get the current pose of the robot
            x = self.amcl_pose.pose.pose.position.x
            y = self.amcl_pose.pose.pose.position.y
            theta = self.get_angle(self.amcl_pose.pose.pose.orientation)

            # Transition to discovery mode
            result = self.discovery_mode((x, y, theta))


            self.get_logger().info("RESULT: " + str(result))
            
            # if the result is stop, then it means that the robot has reached the final goal
            if result == "stop":
                self.get_logger().info("The robot has stopped")
                self.abort()

                raise ExitException("The robot has reached the final goal")

            self.last_goal = self.next_goal # save the last goal

            # compute the next goal based on the result
            self.next_goal = self.action_result2goal(result, (x, y, theta), self.next_goal)
            
            # compute the navigation goal: which is a particular point on the way to the next goal where the robot has to search for the road sign
            intersection_points, angle = self.get_intersection_points(self.next_goal, theta, result)
            points = self.order_by_distance(intersection_points, x, y)
            self.nav_goal = (points[0][0], points[0][1], angle)
            self.action_payload = (points[1][0],points[1][1],angle)
            
            self.get_logger().info("THe nav goal is: " + str(self.nav_goal))
            self.get_logger().info("THe action payload is: " + str(self.action_payload))

            self.get_logger().info(f"***\n NEXT GOAL: {self.next_goal} \n***")
        
            # Start the navigation in a separate thread
            x, y ,angle= map(float, self.nav_goal)
            self.nav_thread = threading.Thread(target=self.start_navigation, args=(x, y, angle))
            self.nav_thread.start()
        
        if not self.nav_thread.is_alive():
            # if the robot has reached the goal, then we need to join the thread and set the flag to True, so that the robot can compute the next goal
            self.flag = True
            self.nav_thread.join()
            self.nav_thread = None






def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    info = PlannerHandler()
    executor.add_node(info)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    except ExitException as e:
        info.get_logger().info("The robot has reached the final goal")
    finally:
        info.get_logger().error(f"An error occurred: {e}")
    

    info.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()