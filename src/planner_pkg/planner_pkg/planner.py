import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def get_next_pose(current_pose, direction):
    x, y, z = current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z
    qx, qy, qz, qw = current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w
    if direction == TurtleBot4Directions.NORTH:
        y += 1
    elif direction == TurtleBot4Directions.EAST:
        x += 1
    elif direction == TurtleBot4Directions.SOUTH:
        y -= 1
    elif direction == TurtleBot4Directions.WEST:
        x -= 1
    return TurtleBot4Navigator.getPoseStamped([x, y], direction)

def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock 
    #if not navigator.getDockedStatus():
    #    navigator.info('Docking before initialising pose')
    #    navigator.dock()

    finished = False
    
    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH) # TODO: The initial pose will be given dynamically
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()


    while not finished: # TODO: The condition to finish the navigation will be given dynamically
        current_pose = navigator.getPose()
        # Get the next direction to move
        direction = navigator.getNextDirection(current_pose)
        
        # Print the pose
        navigator.info(f'Current pose: {current_pose}')
        # Print the direction
        navigator.info(f'Next direction: {direction}')

        navigator.startToPose(get_next_pose(current_pose, direction))

        

    # Set goal poses
    # goal_poses = []
    # goal_poses.append(navigator.getPoseStamped([1.0, 0.0], TurtleBot4Directions.NORTH))
    # goal_poses.append(navigator.getPoseStamped([2.0, 0.0], TurtleBot4Directions.EAST))
    # goal_poses.append(navigator.getPoseStamped([2.0, -1.0], TurtleBot4Directions.NORTH))
    # goal_poses.append(navigator.getPoseStamped([3.0, -1.0], TurtleBot4Directions.WEST))
    # goal_poses.append(navigator.getPoseStamped([3.0, 0.0], TurtleBot4Directions.WEST))
    # goal_poses.append(navigator.getPoseStamped([3.0, 1.0], TurtleBot4Directions.WEST))

    # # Undock
    # #navigator.undock()

    # # Navigate through poses
    # for goal_pose in goal_poses:
    #     input("Press any key to continue...")
    #     navigator.startToPose(goal_pose)

    # Finished navigating, dock
    #navigator.dock()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
