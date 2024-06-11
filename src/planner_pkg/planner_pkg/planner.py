import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.action import ActionClient
from discovery_interface.action import DiscoveryAction
import yaml
import random
import threading
import time

from pyquaternion import Quaternion


ENGAGE_DISTANCE = 3


class GoalNotValidException(Exception):
    """Exception raised when the goal is not valid."""

    def __init__(self, message="Goal is not valid"):
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
        
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is ready, sending goal ...')
        return self.action_client.send_goal_async(goal_msg)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.next_action))
        return result.next_action


class PlannerHandler(Node):

    def __init__(self):
        super().__init__("Info") # Init node
        self.sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)

        # Create an action client for the discovery mode
        self.discovery_action_client = DiscoveryActionClient()

        self.amcl_pose = None
        self.nav_thread = None
        self.navigator = TurtleBot4Navigator()
        self.initial_pose_flag = False

        self.flag = True
        # Wait for Nav2
        self.navigator.waitUntilNav2Active()


        self.build_p_map()
        # Wait for the initial pose

        while self.initial_pose_flag == False:
            # get the confirmation that the initial pose has been set by the user in rviz
            self.get_logger().info("Waiting for the initial pose")
            input("Press Enter to confirm the initial pose")
            self.initial_pose_flag = True
            self.get_logger().info("Initial pose set")

        while self.amcl_pose is None:
            self.get_logger().info("Waiting for the initial pose")
            rclpy.spin_once(self)

        #self.next_goal = self.discovery_mode() # The first thing self.run() will do is to call self.discovery_mode()

        # Create a timer that triggers every second
        self.timer = self.create_timer(2, self.run)

    def pose_callback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info(f"Pose callback: {msg.pose.pose.position.x}, {msg.pose.pose.position.y}")
        


    def build_p_map(self):
        self.get_logger().info("Building the map")
        
        self.map = dict()

        # TODO: how to read the yaml file from a ros2 node? The following code does not work
        # with open("config.yaml") as file: 
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

        # TODO: For now it is hardcoded. It has be changed
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
        self.map[A] = [(B,"right"), (C,"down")]
        self.map[B] = [(A,"left"), (D,"down")]
        self.map[C] = [(A,"up"), (D,"right"), (E,"down")]
        self.map[D] = [(B,"up"), (C,"left"), (F,"down")]
        self.map[E] = [(C,"up"), (F,"right"), (G,"down")]
        self.map[F] = [(D,"up"), (E,"left"), (H,"down")]
        self.map[G] = [(E,"up"), (H,"right"), (I,"down")]
        self.map[H] = [(F,"up"), (G,"left"), (J,"down")]
        self.map[I] = [(G,"up"), (J,"right")]
        self.map[J] = [(H,"up"), (I,"left")]

    def action_result2goal(self,result, initial_pose, current_goal_pose):

        # approximate the angle to the nearest 90 degrees
        next_goal_angle = initial_pose[2]

        # convert frame angle to standard frame (remove the offset)
        next_goal_angle = next_goal_angle + 90

        self.get_logger().info("The next goal angle is: " + str(next_goal_angle))

        if result == "right":
            next_goal_angle -= 90
        elif result == "left":
            next_goal_angle += 90
        elif result == "goback":
            next_goal_angle += 180

        if next_goal_angle >= 360:
            next_goal_angle -= 360
        if next_goal_angle < 0:
            next_goal_angle += 360

        if next_goal_angle < 45 or next_goal_angle >= 315:
            next_goal_angle = "right"
        elif next_goal_angle < 135:
            next_goal_angle = "up"
        elif next_goal_angle < 225:
            next_goal_angle = "left"
        elif next_goal_angle < 315:
            next_goal_angle = "down"
        
        

        neighbors = self.map[current_goal_pose]
        for neighbor in neighbors:
            if neighbor[1] == next_goal_angle:

                self.get_logger().info("The next goal is: " + str(neighbor[0]) + "given that the robot is in the pose: " + str(initial_pose) + " and the result is: " + result)
                return neighbor[0]
        

        raise GoalNotValidException("The goal is not valid, the road sign is: " + result + " the robot pose is (" + str(initial_pose[0]) + ", " + str(initial_pose[1]) + ", " + str(initial_pose[2]) + ") and the goal is (" + str(current_goal_pose[0]) + ", " + str(current_goal_pose[1]) + ")"+ " the neighbors are: " + str(neighbors)+ " the next goal angle is: " + str(next_goal_angle))


    def discovery_mode(self):

        self.get_logger().info("The robot is in discovery mode")
        
        
        # get the current pose of the robot
        x = self.amcl_pose.pose.pose.position.x
        y = self.amcl_pose.pose.pose.position.y
        theta = self.get_angle(self.amcl_pose.pose.pose.orientation)

        # find the nearest goal
        nearest_goal = None
        min_distance = float("inf")
        for goal in self.map:
            distance = (goal[0] - x) ** 2 + (goal[1] - y) ** 2
            if distance < min_distance:
                min_distance = distance
                nearest_goal = goal
        
        # select a random neighbor of the nearest goal
        # neighbors = self.map[nearest_goal]
        # next_goal = neighbors[random.randint(0, len(neighbors) - 1)]

        
        # ****************************
        # send the goal to the discovery action server and wait for the result
        
        future = self.discovery_action_client.send_goal(float(nearest_goal[0]), float(nearest_goal[1]), float(x), float(y), 0)
        rclpy.spin_until_future_complete(self.discovery_action_client, future)

        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.discovery_action_client, get_result_future)

        result = get_result_future.result().result.next_action

        self.get_logger().info("RESULT: " + str(result))
        next_goal = self.action_result2goal(result, (x, y, theta), nearest_goal)

        
        return next_goal

    def start_navigation(self, x, y, angle=0):
        self.navigator.startToPose(self.navigator.getPoseStamped((x, y), angle))

    def get_angle(self, quaternion):
        q = Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        euler = q.yaw_pitch_roll
        yaw = euler[0]
        # convert the yaw angle to degrees
        theta = yaw * 180 / 3.14159265359
        return theta



    def run(self):
        
        if self.amcl_pose is None or self.initial_pose_flag == False:
            return        

        if self.flag and (self.nav_thread is None or not self.nav_thread.is_alive()):
            self.flag = False 
            # if the robot is in proximity of the goal, then we need to discover the next goal and start the navigation
            self.next_goal = self.discovery_mode()
            self.get_logger().info(f"***\n NEXT GOAL: {self.next_goal} \n***")
            x, y = map(float, self.next_goal)
            angle = self.get_angle(self.amcl_pose.pose.pose.orientation)
            # Start the navigation in a separate thread
            self.nav_thread = threading.Thread(target=self.start_navigation, args=(x, y, angle))
            self.nav_thread.start()
        
        if self.nav_thread.is_alive():
            # if the robot is near 1 meter from the goal, then the robot has reached the goal, so we kill the thread
            distance = (self.amcl_pose.pose.pose.position.x - self.next_goal[0]) ** 2 + (self.amcl_pose.pose.pose.position.y - self.next_goal[1]) ** 2
            self.get_logger().info(f"Distance: {distance}")
            if distance < ENGAGE_DISTANCE:
                self.get_logger().info("The robot has reached the goal, killing the thread")
                self.navigator.cancelTask()
                self.nav_thread.join()
                self.nav_thread = None
                self.flag = True
        # if the robot is near 1 meter from the goal, then the robot has reached the goal, so we kill the thread




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