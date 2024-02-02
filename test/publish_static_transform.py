import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_transform_test_publisher')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.pubslish_transform()

    def pubslish_transform(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'test'

        t.transform.translation.x = 1.0
        t.transform.translation.y = 5.0
        t.transform.translation.z = 2.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.3420201
        t.transform.rotation.w = 0.9396926

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()