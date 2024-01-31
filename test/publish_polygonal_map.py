import sys
import yaml

import rclpy
from rclpy.node import Node
#from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from polygonal_map_msgs.msg import PolygonalMap
from geometry_msgs.msg import Polygon, Point32

class PolygonalMapPublisher(Node):

    def __init__(self, polygonal_map_file):
        super().__init__('polygonal_map_publisher')
        #latched: QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(PolygonalMap, 'polygonal_map', 1)
        self.msg = self.read_polygonal_map(polygonal_map_file)
        self.timer = self.create_timer(1.0, self.publish_map)

    def read_polygonal_map(self, polygonal_map_file):
        msg = PolygonalMap()
        with open(polygonal_map_file, 'r') as file:
            try:
                polygonal_map = yaml.full_load(file)
                if not isinstance(polygonal_map, list):
                    raise yaml.YAMLError("Polygonal map must be a list of obstacles")
                for obs in polygonal_map:
                    obs_msg = Polygon()
                    for point in obs:
                        point_msg = Point32()
                        point_msg.x = float(point[0])
                        point_msg.y = float(point[1])
                        obs_msg.points.append(point_msg)
                  
                    msg.obstacles.append(obs_msg)
                    
            except yaml.YAMLError as exc:
                print(exc)

        return msg

    def publish_map(self):
        self.msg.header.frame_id = "map"
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)

    if (len(sys.argv) < 2):
        print("Usage: publish_polygonal_map.py <polygonal_map.yaml>")
        exit()

    polygonal_map_publisher = PolygonalMapPublisher(sys.argv[1])

    rclpy.spin(polygonal_map_publisher)

    polygonal_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()