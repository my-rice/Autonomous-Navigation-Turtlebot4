import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import yaml

class CircleMarkerPublisher(Node):

    def __init__(self):
        super().__init__('circle_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.angle_resolution = 0.1  # radians
        self.large_radius = 3.0  # meters
        self.small_radius = 1.0  # meters
        self.declare_parameter('config_file', '')
        self.config_path = self.get_parameter('config_file').get_parameter_value().string_value
        self.load_config(self.config_path)

    def load_config(self, config_file):
        # Load the configuration file
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)
        self.coordinates = [self.convert_to_float(node['coordinates']) for node in self.config['map']['nodes'].values()]

    def convert_to_float(self, coord):
        return [float(coord[0]), float(coord[1])]

    def timer_callback(self):
        for i, coord in enumerate(self.coordinates):
            self.publish_circle_marker(coord, self.large_radius, i, 1.0, 0.0, 0.0)  # Large circle in red
            self.publish_circle_marker(coord, self.small_radius, i + len(self.coordinates), 0.0, 1.0, 0.0)  # Small circle in green
            self.publish_center_point(coord, i + 2 * len(self.coordinates))  # Center point

    def publish_circle_marker(self, coord, radius, marker_id, r, g, b):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "circle"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Transparency
        marker.color.r = r  # Red color
        marker.color.g = g  # Green color
        marker.color.b = b  # Blue color

        # Create points for the circle centered at (coord[0], coord[1])
        points = []
        for angle in self.frange(0, 2 * math.pi, self.angle_resolution):
            x = coord[0] + radius * math.cos(angle)
            y = coord[1] + radius * math.sin(angle)
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            points.append(point)

        # Add the first point at the end to close the circle
        points.append(points[0])
        marker.points = points

        self.publisher_.publish(marker)
        self.get_logger().info(f'Publishing circle marker {marker_id} at ({coord[0]}, {coord[1]}) with radius {radius}')

    def publish_center_point(self, coord, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "center_point"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = coord[0]
        marker.pose.position.y = coord[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Sphere diameter
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Transparency
        marker.color.g = 1.0  # Green color

        self.publisher_.publish(marker)
        self.get_logger().info(f'Publishing center point marker {marker_id} at ({coord[0]}, {coord[1]})')

    def frange(self, start, stop, step):
        while start < stop:
            yield start
            start += step

def main(args=None):
    rclpy.init(args=args)
    node = CircleMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
