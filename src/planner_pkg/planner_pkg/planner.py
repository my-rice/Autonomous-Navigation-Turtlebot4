import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import KidnapStatus

from std_msgs.msg import Bool

from rclpy.action import ActionClient
from discovery_interface.action import DiscoveryAction
import yaml
import math
from pyquaternion import Quaternion
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import random

import threading
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point

from enum import Enum

import time


# create enun for states
class States(Enum):
    STARTUP = 0
    FIRST_LOCALIZATION = 1
    FIRST_NAVIGATION = 2
    DISCOVERY = 3
    NAVIGATION = 4
    RECOVERY = 5
    STOP = 6
    

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
    """ Action client for the discovery mode """
    def __init__(self):
        super().__init__('discovery_action_client')
        self._goal_handle = None
        self.action_client = ActionClient(self, DiscoveryAction, 'discovery_mode')

    def send_goal(self,goal_pose_x, goal_pose_y, start_pose_x, start_pose_y, angle):
        """" Implement the send_goal method. It prepares the goal message with the starting pose and the goal pose of the discovery and sends it to the action server."""
        goal_msg = DiscoveryAction.Goal()
        goal_msg.goal_pose_x = goal_pose_x
        goal_msg.goal_pose_y = goal_pose_y
        goal_msg.start_pose_x = start_pose_x
        goal_msg.start_pose_y = start_pose_y
        goal_msg.angle = angle
        
        # wait for the action server to be ready
        self.action_client.wait_for_server()

        self.get_logger().info('Action server is ready, sending goal ...')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)  

        self.send_goal_future.add_done_callback(self.goal_response_callback)
        return self.send_goal_future
    

    def goal_response_callback(self, future):
        """Callback function for the goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle

    def cancel_goal(self):
        """Cancel the goal."""
        if self._goal_handle is not None:

            self.send_goal_future.cancel()

            self.get_logger().info('Canceling goal')
            # Cancel the goal
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)
            
            #return future
            self.get_logger().info('Goal canceled')

    def cancel_done(self, future):
        """Callback function for the cancel response."""
        self.get_logger().info('Cancel done Callback')
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')
        self._goal_handle = None



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

        self.timer_mutual_exclusion_group = MutuallyExclusiveCallbackGroup()
        self.parallel_group = ReentrantCallbackGroup()
        self.kidnap_group = MutuallyExclusiveCallbackGroup()
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10, callback_group=self.parallel_group)
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.publisher_marker = self.create_publisher(Marker, 'visualization_marker', 10)


        self.initial_pose_sub = self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_pose_callback,10, callback_group=self.parallel_group)

        # Create an action client for the discovery mode
        self.discovery_action_client = DiscoveryActionClient()

        # Read the parameters from the config file
        self.declare_parameter('config_file', '')
        self.config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.get_logger().info("Config file: " + self.config_path)
        self.read_parameters()
        

        # initialize the state of the robot to the startup state
        self.state = States.STARTUP

        # Create a subscription to the kidnapped topic only after the initial pose has been set
        self.kidnapped_sub = self.create_subscription(KidnapStatus, "/kidnap_status", self.kidnapped_callback, qos_reliable, callback_group=self.kidnap_group)

        # TOPIC FOR TESTING IN SIMULATION: to test the recovery mode, we need to kidnap the robot
        # self.test_sub = self.create_subscription(Bool, "/test", self.kidnapped_test_callback, 10, callback_group=self.kidnap_group)
        


        self.get_logger().info("The robot is on_startup")
        self.amcl_pose = None
        self.navigator = TurtleBot4Navigator()
        self.initial_pose_flag = False
        self.first_discovery = True
        self.is_kidnapped = False
        self.last_goal = None
        self.last_nav_goal = None
        self.next_goal = None
        self.nav_goal = None
        self.result = None
        self.last_kidnapped = False
        self.discovery_flag = False

        # Build the map used by the planner to compute the next goal
        self.build_p_map()

        while self.initial_pose_flag == False: # TODO: Is it necessary to wait for the initial pose? In this way?
            self.get_logger().info("Please, set the initial pose ...")
            rclpy.spin_once(self, timeout_sec=1)

        # Wait until the navigation2 is active
        self.navigator.waitUntilNav2Active()
        self.navigator.clearAllCostmaps()

        # Wait until the action server is ready
        if(self.discovery_action_client.wait_for_server(self.timeout)):
            self.get_logger().info("Action server is ready")
        else:
            self.get_logger().info("Action server is not ready, shutting down...")
            rclpy.shutdown() #TODO: raise exception and exit
            exit()

        # Start the timer for the planner that will run the main loop
        self.get_logger().info("self.timer_period: " + str(self.timer_period))
        self.timer = self.create_timer(timer_period_sec=self.timer_period, callback=self.run, callback_group=self.timer_mutual_exclusion_group)

    def destroy_node(self):
        self.navigator.destroy_node()
        self.discovery_action_client.destroy()
        rclpy.try_shutdown()
        super().destroy_node()

    def init_planner_internal_state(self):
        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        orientation = self.amcl_pose.pose.pose.orientation
        theta = self.get_angle(orientation)
        pose = (x, y, theta)
        self.next_goal,self.last_goal = self.compute_first_goal(pose)

        approach_angle = self.get_approach_angle(self.next_goal, self.last_goal, orientation)
        intersection_points, angle = self.get_intersection_points(self.next_goal, approach_angle, rho=self.rho)
        points = self.order_by_distance(intersection_points, x, y)
        self.action_payload = (points[1][0],points[1][1],angle)
        self.nav_goal = (points[0][0], points[0][1], angle)

        self.last_nav_goal = pose


    def read_parameters(self):
        """" Read the parameters from the config file."""
        self.get_logger().info("Reading parameters from the config file: " + self.config_path)
        with open(self.config_path, 'r') as file:
            config = yaml.safe_load(file)

        self.timer_period = config['planner_parameters']['timer_period']
        self.timeout = config['planner_parameters']['timeout']
        self.rho = config['planner_parameters']['rho']
        self.nodes = config['map']['nodes']
        self.connections = config['map']['connections']

    def pose_callback(self, msg):
        """Callback function for the pose topic. It updates the current pose of the robot."""
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        
   
    def initial_pose_callback(self, msg):
        """Callback function for the initial pose topic. It sets the initial pose of the robot."""
        self.get_logger().info("Initial pose callback: " + str(msg.pose.pose.position.x) + " " + str(msg.pose.pose.position.y))
        if self.initial_pose_flag == False:
            self.initial_pose_flag = True
            angle = self.get_angle(msg.pose.pose.orientation)
            initial_pose = self.navigator.getPoseStamped([msg.pose.pose.position.x, msg.pose.pose.position.y], angle)
            self.amcl_pose = msg
            self.navigator.setInitialPose(initial_pose)
            self.get_logger().info('Initial pose received: {0} {1} {2}'.format(msg.pose.pose.position.x, msg.pose.pose.position.y, angle))

    def kidnapped_callback(self, msg):
        """Callback function for the kidnapped topic. It updates the kidnapped status of the robot, and it handles the kidnapped mode. This is a part of the recovery mode."""
        # self.get_logger().info("Kidnapped status: " + str(self.is_kidnapped))
        if msg.is_kidnapped == True and self.last_kidnapped == False:
            # if the robot has been kidnapped, then we need to restart the navigation from a specific point. This is not a proper way to handle the kidnapped mode but it is a recovery mode by kidnapping the robot
            self.get_logger().info("The robot is in kidnapped mode, the last_nav_goal is:" + str(self.last_nav_goal) + " and the last goal is: " + str(self.last_goal))
            self.is_kidnapped = msg.is_kidnapped
            # self.abort() # abort the current goal
            # self.get_logger().info("ABORTED 2")
            # self.plot_point_on_rviz()

        if msg.is_kidnapped == False and self.last_kidnapped == True:
            self.get_logger().info("The robot is deployed")
            # self.relocate()     
            self.is_kidnapped = msg.is_kidnapped # ONLY AFTER THE ROBOT HAS BEEN RELOCATED, THEN WE CAN SET THE FLAG TO FALSE
            # self.timer.reset()
        
        self.last_kidnapped = self.is_kidnapped

    def kidnapped_test_callback(self, msg):
        """Callback function that behave the same as kidnapped_callback. It is used only for testing in simulation."""
        self.get_logger().info("Kidnapped status: " + str(self.is_kidnapped))
        if msg.data == True and self.last_kidnapped == False:
            # if the robot has been kidnapped, then we need to restart the navigation from a specific point. This is not a proper way to handle the kidnapped mode but it is a recovery mode by kidnapping the robot
            self.get_logger().info("The robot is in kidnapped mode, the last_nav_goal is:" + str(self.last_nav_goal) + " and the last goal is: " + str(self.last_goal))            
            self.is_kidnapped = msg.data

        if msg.data == False and self.last_kidnapped == True:
            self.get_logger().info("The robot is deployed")
            self.is_kidnapped = msg.data # ONLY AFTER THE ROBOT HAS BEEN RELOCATED, THEN WE CAN SET THE FLAG TO FALSE
        
        self.last_kidnapped = self.is_kidnapped

    def build_p_map(self):
        """Build the map used by the planner to compute the next goal. The map is a dictionary where the key is the coordinates of the node and the value is a list of tuples. Each tuple contains the coordinates of the neighbor node and the direction to reach the neighbor node."""
        self.get_logger().info("Building the map")
        
        self.map = dict()

        coordinates_lookup = {key: tuple(value['coordinates']) for key, value in self.nodes.items()}
        
        for key, value in self.connections.items():
            coord_key = coordinates_lookup[key]
            self.map[coord_key] = [(coordinates_lookup[conn['point']], conn['direction']) for conn in value]

    def compute_next_angle(self, current_angle, result):
        """Compute the next angle based on the result of the road sign. The angle is approximated to the nearest 90 degrees."""
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
        """Compute the next goal based on the result of the road sign. The goal is a node in the map."""
        next_goal_angle = initial_pose[2]
        self.get_logger().info("The initial pose is: " + str(initial_pose))
        self.get_logger().info("THe result is: " + result)

        next_goal_angle = self.compute_next_angle(next_goal_angle, result)

        if next_goal_angle < 45 or next_goal_angle >= 315: 
            next_goal_angle = "north"
        elif next_goal_angle < 135:
            next_goal_angle = "west"
        elif next_goal_angle < 225:
            next_goal_angle = "south"
        elif next_goal_angle < 315:
            next_goal_angle = "east"


        neighbors = self.map[current_goal_pose]
        for neighbor in neighbors:
            if neighbor[1] == next_goal_angle:
                self.get_logger().info("The next goal is: " + str(neighbor[0]) + ", given that the robot is in the pose: " + str(initial_pose) + " and the result is: " + result)
                return neighbor[0]
        
        raise GoalNotValidException("The goal is not valid, the road sign is: " + result + " the robot pose is (" + str(initial_pose[0]) + ", " + str(initial_pose[1]) + ", " + str(initial_pose[2]) + ") and the goal is (" + str(current_goal_pose[0]) + ", " + str(current_goal_pose[1]) + ")"+ " the neighbors are: " + str(neighbors)+ " the next goal angle is: " + str(next_goal_angle))

    def order_by_distance(self, intersection_points, x, y):
        """Utility function to order the intersection points by distance from the robot."""
        x1, y1 = intersection_points[0]
        x2, y2 = intersection_points[1]
        distance1 = (x1 - x) ** 2 + (y1 - y) ** 2
        distance2 = (x2 - x) ** 2 + (y2 - y) ** 2
        if distance1 < distance2:
            return intersection_points
        else:
            return intersection_points[::-1] # reverse the list

    def discovery_mode(self,current_pose):
        """Discovery mode of the robot. The robot is in discovery mode when it is searching for the road sign. 
        This method sends the goal to the discovery action server and waits for the result. The result is the 
        action that the robot has to perform based on the road sign. This method also handles the first discovery 
        mode."""
        self.get_logger().info("The robot is in discovery mode")
        
        x = current_pose[0]
        y = current_pose[1]
        theta = current_pose[2]
        
        self.get_logger().info("sending the goal to the discovery action server: " + str(self.action_payload) + " and the current pose is: " + str(current_pose))
        
        # send the goal to the discovery action server and wait for the result
        self.navigator.clearAllCostmaps()
        future_goal = self.discovery_action_client.send_goal(float(self.action_payload[0]), float(self.action_payload[1]), float(x), float(y), float(self.action_payload[2]))
        self.get_logger().info("Waiting" + str(future_goal))
        while not future_goal.done():
            self.get_logger().info("Waiting for the result ... ")
            rclpy.spin_once(self.discovery_action_client, timeout_sec=0.5)
        self.get_logger().info("The result is ready, getting the result ... ")

        goal_handle = future_goal.result()
        get_result_future = goal_handle.get_result_async()
        self.get_logger().info("Waiting" + str(get_result_future))

        recovery_flag = False

        while not get_result_future.done() and not get_result_future.cancelled(): #and not recovery_flag:
            #self.get_logger().info("done: " + str(get_result_future.done()) + " cancelled: " + str(get_result_future.cancelled()))
            
            if self.is_kidnapped:
                # cancel the current goal
                self.get_logger().info("The robot is kidnapped, canceling the current goal")
                if recovery_flag == False:
                    self.discovery_action_client.cancel_goal()
                recovery_flag = True
            
            self.get_logger().info("Waiting for the result:")
            rclpy.spin_once(self.discovery_action_client, timeout_sec=0.5)

        result = get_result_future.result().result.next_action
        self.navigator.clearAllCostmaps()

        return result
        
    def get_intersection_points(self, next_goal, approach_angle, rho=3):
        """Compute the intersection points of the robot with the circle centered in the next goal and with radius rho.
          The intersection points are computed based on the approach angle of the robot. The approach angle is 
          the angle between two consecutive angles. The intersection points are the most significant points of the 
          circle, which are calculated by intersecting the circle with the line between the two goals. 
          The intersection points are ordered by distance from the robot."""
        x1,y1 = next_goal
        xc, yc = next_goal
        
        if approach_angle < 0:
            approach_angle += 360
        if approach_angle >= 360:
            approach_angle -= 360

        # if the approach angle is 0 or 180, then the intersection points are on the x axis
        if approach_angle == 90 or approach_angle == 270:
            intersection_points = [(xc, yc + rho), (xc, yc - rho)]
            return intersection_points, approach_angle

        m = math.tan(math.radians(approach_angle))
        q = y1 - m * x1
        
        # Coefficients of the quadratic equation
        A = 1 + m**2
        B = 2 * (m * q - m * yc - xc)
        C = xc**2 + yc**2 + q**2 - 2 * yc * q - rho**2

        # Compute the discriminant
        delta = B**2 - 4 * A * C

        if delta < 0:
            raise ValueError("No intersection points")

        # Compute the x coordinates
        x_intersect1 = (-B + math.sqrt(delta)) / (2 * A)
        x_intersect2 = (-B - math.sqrt(delta)) / (2 * A)

        # Compute the y coordinates
        y_intersect1 = m * x_intersect1 + q
        y_intersect2 = m * x_intersect2 + q

        # The intersection points
        intersection_points = [(x_intersect1,y_intersect1), (x_intersect2, y_intersect2)]
        return intersection_points, approach_angle

    def get_angle(self, quaternion):
        """Compute the yaw angle from the quaternion. The yaw angle is the angle around the z-axis."""
        q = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        euler = q.yaw_pitch_roll
        yaw = euler[0]
        # convert the yaw angle to degrees
        theta = yaw * 180 / 3.14159265359
        return theta
    

    def relocate(self):
        """Relocate the robot to the ideal relocation point. The ideal relocation point is the intersection point of the robot with the circle centered in the last goal and with radius 4."""
        self.get_logger().info("Relocating the robot")
        ### Get the ideal relocation pose of the robot

        self.get_logger().info("The last goal is: " + str(self.last_goal) + " and the last nav goal is: " + str(self.last_nav_goal) + " and the next goal is: " + str(self.next_goal))
        points, angle = self.get_intersection_points(self.last_goal,self.last_nav_goal[2], rho=4)
        points = self.order_by_distance(points, self.last_nav_goal[0], self.last_nav_goal[1])
        ideal_relocation = points[0]
        self.action_payload = (points[1][0],points[1][1],angle) # the action payload is the last point of the crossing where the robot has to search for the traffic sign
        self.navigator.setInitialPose(self.navigator.getPoseStamped(ideal_relocation, angle))

        self.get_logger().info("The ideal relocation is: " + str(ideal_relocation))
        self.next_goal = self.last_goal
        self.nav_goal = self.last_nav_goal
        return ideal_relocation, angle
        

    # def abort(self):
    #     # Cancel the current goal
    #     self.timer.cancel()
    #     self.get_logger().info("Aborting the current goal")
    #     self.get_logger().info("The last goal is: " + str(self.last_goal) + " and the last nav goal is: " + str(self.last_nav_goal) + " and the next goal is: " + str(self.next_goal))
        
    #     if self.nav_thread is not None:
    #         self.navigator.cancelTask()
    #         self.get_logger().info(" NAV THREAD ACTIVE ")
    #         self.nav_thread.join()
    #         self.nav_thread = None
    #         #self.flag = True
    #         #self.discovery_flag = False
    #     self.discovery_action_client.cancel_goal()

    def compute_first_goal(self, pose):
        """Compute the first goal of the robot. The first goal is the nearest goal to the robot based on the robot's orientation and given the characteristics of the map. The last goal is the goal that it is located behind the robot."""
        x, y, theta = pose
        if(theta<0):
            theta += 360
        if(theta>=360):
            theta -= 360
        possible_goals = []
        possible_last_goals = []
        if theta < 45 or theta >= 315: 
            
            for x_m,y_m  in self.map.keys():
                if x_m > x:
                    possible_goals.append((x_m,y_m))
                elif x_m < x:
                    possible_last_goals.append((x_m,y_m))

        elif theta < 135:
            
            for x_m,y_m  in self.map.keys():
                if y_m > y:
                    possible_goals.append((x_m,y_m))
                elif y_m < y:
                    possible_last_goals.append((x_m,y_m))

        elif theta < 225:
            
            for x_m,y_m  in self.map.keys():
                if x_m < x:
                    possible_goals.append((x_m,y_m))
                elif x_m > x:
                    possible_last_goals.append((x_m,y_m))

        elif theta < 315:
            
            for x_m,y_m  in self.map.keys():
                if y_m < y:
                    possible_goals.append((x_m,y_m))
                elif y_m > y:
                    possible_last_goals.append((x_m,y_m))

        # the next goal is the nearest goal to the robot
        min_distance = float("inf")
        for goal in possible_goals:
            distance = (goal[0] - x) ** 2 + (goal[1] - y) ** 2
            if distance < min_distance:
                min_distance = distance
                next_goal = goal
        
        # the last goal is the nearest goal to the robot
        min_distance = float("inf")
        for goal in possible_last_goals:
            distance = (goal[0] - x) ** 2 + (goal[1] - y) ** 2
            if distance < min_distance:
                min_distance = distance
                last_goal = goal


        return next_goal,last_goal

    # def start_navigation(self, x, y, angle=0):
    #     self.navigator.startToPose(self.navigator.getPoseStamped((x, y), angle))

    def on_startup(self):
        # Initialize the variables for the planner used in the run method
        self.get_logger().info("The robot is on_startup")
        self.init_planner_internal_state()
        
        self.state = States.FIRST_LOCALIZATION

    def on_first_localization(self):
        self.get_logger().info("The robot is on_first_localization")
        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        theta = self.get_angle(self.amcl_pose.pose.pose.orientation)
        pose = (x, y, theta)
        distance_to_goal = (self.next_goal[0] - x) ** 2 + (self.next_goal[1] - y) ** 2
        if distance_to_goal > self.rho:
            # if the robot is too far from the goal, then we need to approach the goal
            self.state = States.FIRST_NAVIGATION
        else:
            self.nav_goal = pose # If the robot is close to the goal then the navigation goal is the current pose of the robot, so there will be no problem for the discovery mode
            self.state = States.DISCOVERY

    def on_first_navigation(self):
        self.get_logger().info("The robot is on_first_navigation")
        x, y ,angle= map(float, self.nav_goal)
        
        if self.is_kidnapped:
            # transition to the recovery mode
            self.state = States.RECOVERY
            return
        
        self.navigator.goToPose(self.navigator.getPoseStamped((x, y), angle))
        while not self.navigator.isTaskComplete():
            self.get_logger().info("Waiting for first navigation task completion")
            if self.is_kidnapped:
                # cancel the current goal
                self.navigator.cancelTask()
                self.state = States.RECOVERY
                return

        if not self.is_kidnapped:
            self.state = States.DISCOVERY
        else:
            self.state = States.RECOVERY
            return



    def on_discovery(self):
        self.get_logger().info("The robot is on_discovery")


        if self.is_kidnapped:
            # transition to the recovery mode, here is safe.
            self.state = States.RECOVERY
            return

        self.first_discovery = False

        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        theta = self.get_angle(self.amcl_pose.pose.pose.orientation)
        pose = (x, y, theta)

        # NOTE: Updating the last goal and the last navigation goal is THE only dangerous operation in the planner, considering that the robot can be kidnapped at any time.
        self.last_goal = self.next_goal # save the last goal
        self.last_nav_goal = self.nav_goal # save the last navigation goal

        self.result = self.discovery_mode(pose)

        if self.result == "Canceled": # If the discovery mode has been canceled, then we need to abort the current iteration
            # transition to the recovery mode
            self.state = States.RECOVERY
            return


        self.get_logger().info("RESULT: " + str(self.result))
        
        # if the result is stop, then it means that the robot has reached the final goal
        if self.result == "stop":
            self.state = States.STOP
            return
                 
        if self.result == "random":
            # take all the possible neighbors 
            self.get_logger().info("The robot has to choose a random goal")
            neighbors = self.map[self.next_goal]
            self.next_goal = random.choice(neighbors)[0]
            self.get_logger().info("The next (random) goal is: " + str(self.next_goal))
        else:
            # compute the next goal based on the result
            self.next_goal = self.action_result2goal(self.result, (x, y, theta), self.next_goal)

        # computing the next navigation goal
        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        
        # compute the angolar coefficient of the line that connects the next goal to the last goal. This will be the approach angle to the next goal
        approach_angle = self.get_approach_angle(self.next_goal, self.last_goal, self.amcl_pose.pose.pose.orientation)
        
        intersection_points, angle = self.get_intersection_points(self.next_goal, approach_angle)
        points = self.order_by_distance(intersection_points, x, y)
        self.nav_goal = (points[0][0], points[0][1], angle)
        self.action_payload = (points[1][0],points[1][1],angle)

        
        self.state = States.NAVIGATION
            


    def on_navigation(self):
        self.get_logger().info("The robot is on_navigation")


        if self.is_kidnapped:
            # transition to the recovery mode
            self.state = States.RECOVERY
            return


        self.get_logger().info("THe nav goal is: " + str(self.nav_goal))
        #self.get_logger().info("THe action payload is: " + str(self.action_payload))
        self.get_logger().info(f"\n***********\nNEXT GOAL: {self.next_goal}\n***********")

        # Start the navigation in a separate thread
        x, y ,angle= map(float, self.nav_goal)


        self.navigator.goToPose(self.navigator.getPoseStamped((x, y), angle))
        while not self.navigator.isTaskComplete():
            self.get_logger().info("Waiting for navigation task completion")
            if self.is_kidnapped:
                # cancel the current goal
                self.navigator.cancelTask()
                self.state = States.RECOVERY
                return

        self.state = States.DISCOVERY


    def on_stop(self):
        self.get_logger().info("The robot in on_stop")
        raise ExitException("The robot has reached the final goal")

    def plot_arrow_on_rviz(self, ideal_relocation, angle):
        # Calculate endpoint of the arrow based on angle
        self.get_logger().info("Plotting the point on rviz")
        arrow_length = 1.0  # Length of the arrow (adjust as needed)
        end_point = Point()
        end_point.x = ideal_relocation[0] + arrow_length * math.cos(math.radians(angle))
        end_point.y = ideal_relocation[1] + arrow_length * math.sin(math.radians(angle))
        end_point.z = 0.0

        # Create a Marker for the arrow
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "map"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "relocation_arrow"
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Set the position and orientation of the arrow
        start_point = Point()
        start_point.x = ideal_relocation[0]
        start_point.y = ideal_relocation[1]
        start_point.z = 0.0

        arrow_marker.points = [start_point, end_point]

        # Set the scale of the arrow (shaft diameter, head diameter, head length)
        arrow_marker.scale.x = 0.05  # Shaft diameter
        arrow_marker.scale.y = 0.1   # Head diameter
        arrow_marker.scale.z = 0.1   # Head length

        # Set the color of the arrow
        arrow_marker.color.a = 1.0  # Transparency
        arrow_marker.color.r = 1.0  # Red color
        arrow_marker.color.g = 0.0  # Green color
        arrow_marker.color.b = 0.0  # Blue color

        # Publish the arrow marker
        self.publisher_marker.publish(arrow_marker)


    def on_recovery(self):
        self.get_logger().info("The robot is on_recovery")
        

        if self.first_discovery:
            self.navigator.setInitialPose(self.navigator.getPoseStamped(self.last_nav_goal[0:2], self.last_nav_goal[2]))
            self.plot_arrow_on_rviz(self.last_nav_goal[0:2], self.last_nav_goal[2])
        else:
            point,angle = self.relocate()
            self.plot_arrow_on_rviz(point,angle)


        # wait for the kidnapped status to be False
        while self.is_kidnapped:
            self.get_logger().info("The robot is in kidnapped mode, waiting for the robot to be deployed")
            #rclpy.spin_once(self, timeout_sec=1)
            time.sleep(1)
        
        self.get_logger().info("The robot has been relocated, the next goal is: " + str(self.next_goal) + " and the last goal is: " + str(self.nav_goal))
        if self.first_discovery:
            self.state = States.FIRST_LOCALIZATION
        else:
            self.state = States.NAVIGATION

    def run(self):

        self.get_logger().info("The robot is in the state: " + str(self.state))

        if self.state == States.STARTUP:
            self.on_startup()
        elif self.state == States.FIRST_LOCALIZATION:
            self.on_first_localization()
        elif self.state == States.FIRST_NAVIGATION:
            self.on_first_navigation()
        elif self.state == States.DISCOVERY:
            self.on_discovery()
        elif self.state == States.NAVIGATION:
            self.on_navigation()
        elif self.state == States.RECOVERY:
            self.on_recovery()
        elif self.state == States.STOP:
            self.on_stop()
        else:
            # Handle unknown state
            raise ValueError("Unknown state: " + str(self.state))
        
        #self.get_logger().info("run method completed")
                


    def get_approach_angle(self, next_goal, last_goal, current_orientation: Quaternion):
        """Compute the approach angle to the next goal. The approach angle is the angle between the line connecting the last goal and the next goal and the x-axis of the map."""
        if last_goal != next_goal:
                approach_angle = math.atan2(next_goal[1] - last_goal[1], next_goal[0] - last_goal[0]) * 180 / math.pi
        else:
            approach_angle = self.get_angle(current_orientation) # From the pose of the robot
            approach_angle += self.compute_next_angle(self.result)
            if approach_angle >= 360:
                approach_angle -= 360
            
        if approach_angle < 0:
            approach_angle += 360
        
        return approach_angle


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    planner = PlannerHandler()
    executor.add_node(planner)

    try:
        while rclpy.ok():
            executor.spin()
    except KeyboardInterrupt:
        planner.get_logger().info("KeyboardInterrupt")
    
    except ExitException as e:
        planner.get_logger().info("The robot has reached the final goal")
    finally:
        planner.get_logger().info("The planner is shutting down")
    
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()