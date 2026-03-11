#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class TestCloudPublisher(Node):
    def __init__(self):
        super().__init__('test_cloud_publisher')
        self.pub = self.create_publisher(PointCloud2, '/gicp_map', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)
        self.count = 0

    def timer_cb(self):
        # create a synthetic cloud with many ground points (z ~ 0.0) and several obstacle clusters
        pts = []
        # dense ground grid (ensure enough points)
        for xi in range(200):
            for yi in range(20):
                x = float(xi) * 0.02
                y = float(yi) * 0.02 - 0.2
                z = 0.0
                pts.append((x, y, z))
        # several obstacle clusters above ground
        for cx in [3.0, 4.0, 5.0]:
            for i in range(200):
                x = cx + (i % 10) * 0.02
                y = 0.5 + (i // 10) * 0.02
                z = 1.0 + ((i % 5) * 0.01)
                pts.append((x, y, z))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        cloud = point_cloud2.create_cloud_xyz32(header, pts)
        self.pub.publish(cloud)
        self.count += 1
        if self.count >= 200:
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TestCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
