import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
from tf2_ros import TransformListener, Buffer
from sklearn.cluster import AgglomerativeClustering
import tf_transformations
from std_msgs.msg import Int16
from smrr_interfaces.msg import Entities
### This node can convert lidar readings to line segments
### Need to create a node to filter out humans from lidar data and send static obs lidar reading to this node.

class StaticObstacles(Node):
    def __init__(self):
        super().__init__('static_obstacle_pulisher')

        # Create a publisher for visualization markers
        self.local_map_subscriber = self.create_subscription(Entities, 'local_lines_array', self.local_callback, 10)
        self.floor_number_subscriber = self.create_subscription(Int16, 'floor_number',self.floor_callback, 10)
        self.map_publisher = self.create_publisher(Marker, 'combined_line_segments', 10)

        self.local_lines = []
        self.floor_lines = []
        self.floor = -1

        # Timer to control marker update rate (e.g., 1 Hz)
        #self.timer = self.create_timer(0.5, self.publish_marker)

    def local_callback(self, msg):
        self.local_lines = msg
        self.local_lines = []
        line_count = msg.count
        for i in range(line_count):
            x1 = msg.x[2*i]
            y1 = msg.y[2*i]
            x2 = msg.x[2*i+1]
            y2 = msg.y[2*i+1]
            line = [[x1, y1],[x2, y2]]
            self.local_lines.append(line)
        self.combine_lines()
        # Extract robot's pose from odometry
        # print(self.local_lines)

    def combine_lines(self):
        combine_array = self.local_lines

        for i in range(len(self.floor_lines)):
            combine_array.append(self.floor_lines[i])

        print(len(combine_array))

    def floor_callback(self, msg):
        self.floor = msg
        self.floor_lines = []

def main(args=None):
    rclpy.init(args=args)
    lidar_line_extraction_node = StaticObstacles()
    rclpy.spin(lidar_line_extraction_node)

    lidar_line_extraction_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
