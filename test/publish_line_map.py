import sys
import yaml

import rclpy
from rclpy.node import Node
#from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from polygonal_map_msgs.msg import LineMap, LineSegment
from geometry_msgs.msg import Point

class LineMapPublisher(Node):

    def __init__(self, line_map_file):
        super().__init__('line_map_publisher')
        #latched: QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(LineMap, 'line_map', 1)
        self.msg = self.read_line_map(line_map_file)
        self.timer = self.create_timer(1.0, self.publish_map)

    def read_line_map(self, line_map_file):
        msg = LineMap()
        with open(line_map_file, 'r') as file:
            try:
                line_map = yaml.full_load(file)
                if not isinstance(line_map, list):
                    return msg
                for line in line_map:
                    line_msg = LineSegment()
                    line_msg.start.x = float(line[0][0])
                    line_msg.start.y = float(line[0][1])
                    line_msg.end.x = float(line[1][0])
                    line_msg.end.y = float(line[1][1])
                    msg.lines.append(line_msg)
                    
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
        print("Usage: publish_line_map.py <line_map.yaml>")
        exit()

    line_map_publisher = LineMapPublisher(sys.argv[1])

    rclpy.spin(line_map_publisher)

    line_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()