import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.clock import Duration

class Handler(Node):
    def __init__(self):
        super().__init__("no_localization_node") #Init node
        self.get_logger().set_level(LoggingSeverity.DEBUG)
        self.create_timer(0.1, self.move_callback) #Creating a timer
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.ok = False

    def move_callback(self):
        if self.ok:
            self.cmd_pub.publish(self.msg)

    def rotate(self, clockwise=True):
        factor = -1 if clockwise else 1
        msg = Twist()
        msg.angular.z = 1.57 * factor #rad/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False

    def go(self):
        msg = Twist()
        msg.linear.x = 0.25 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(4)
        self.ok = False

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0 #m/s
        self.msg = msg
        self.ok = True
        self.sleep(1)
        self.ok = False
    
    def sleep(self, time_seconds):
        self.get_clock().sleep_for(Duration(seconds=time_seconds)) #sleep for <time_seconds> seconds
    
    def loop(self):
        self.get_logger().info("Node starting...")
        self.sleep(1)
        self.get_logger().info("Moving forward...")
        self.go() #Moving forward for 4 seconds
        self.get_logger().info("Rotating...")
        self.rotate() #Rotating for 1 seconds
        self.get_logger().info("Moving forward...")
        self.go() #Move forward for 4 seconds
        self.get_logger().info("Rotating anti-clockwise...")
        self.rotate(clockwise=False) #Rotating for 1 seconds
        self.get_logger().info("Moving forward...")
        self.go() #Move forward for 4 seconds
        self.get_logger().info("Rotating anti-clockwise...")
        self.rotate(clockwise=False) #Rotating for 1 seconds
        self.get_logger().info("Moving forward...")
        self.go() #Move forward for 4 seconds
        self.get_logger().info("Moving forward...")
        self.go()
        self.get_logger().info("Stopping...")
        self.stop()
        self.get_logger().info("End!")

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    handller = Handler()
    param = rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True) #Creating 'use_sim_time' node parameter
    handller.set_parameters([param]) #Setting 'use_sim_time' node parameter
    executor.add_node(handller) #Adding node to executor
    executor.create_task(handller.loop) #Creating a task with callable given as input
    try:
        executor.spin() #Running loop - bocking call
    except KeyboardInterrupt:
        pass

    handller.destroy_node()

if __name__ == "__main__":
    main()