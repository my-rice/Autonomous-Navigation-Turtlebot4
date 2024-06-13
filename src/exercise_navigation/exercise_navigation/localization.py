import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock 
    #if not navigator.getDockedStatus():
    #    navigator.info('Docking before initialising pose')
    #    navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_poses = []
    goal_poses.append(navigator.getPoseStamped([1.0, 0.0], TurtleBot4Directions.NORTH))
    goal_poses.append(navigator.getPoseStamped([2.0, 0.0], TurtleBot4Directions.EAST))
    goal_poses.append(navigator.getPoseStamped([2.0, -1.0], TurtleBot4Directions.NORTH))
    goal_poses.append(navigator.getPoseStamped([3.0, -1.0], TurtleBot4Directions.WEST))
    goal_poses.append(navigator.getPoseStamped([3.0, 0.0], TurtleBot4Directions.WEST))
    goal_poses.append(navigator.getPoseStamped([3.0, 1.0], TurtleBot4Directions.WEST))
    
    # Undock
    #navigator.undock()

    # Navigate through poses
    for goal_pose in goal_poses:
        input("Press any key to continue...")
        navigator.startToPose(goal_pose)

    # Finished navigating, dock
    #navigator.dock()

    rclpy.shutdown()