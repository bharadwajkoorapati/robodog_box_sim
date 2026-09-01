#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class WallMarkerPublisher(Node):
    def __init__(self):
        super().__init__('wall_marker_publisher')
        self.pub = self.create_publisher(Marker, 'wall_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'test_wall'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 1.5
        m.pose.position.y = 0.0
        m.pose.position.z = 0.5
        m.pose.orientation.w = 1.0
        m.scale.x = 0.1
        m.scale.y = 2.0
        m.scale.z = 1.0
        m.color.r = 0.8
        m.color.g = 0.2
        m.color.b = 0.2
        m.color.a = 1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    node = WallMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()