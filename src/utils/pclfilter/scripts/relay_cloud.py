#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class Relay(Node):
    def __init__(self):
        super().__init__('cloud_relay')
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cb, 10)
        self.pub = self.create_publisher(PointCloud2, '/gicp_map', 10)
        self.get_logger().info('Relay started: /cloud_registered -> /gicp_map')

    def cb(self, msg):
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
