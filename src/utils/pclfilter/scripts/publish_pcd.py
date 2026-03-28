#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def read_pcd_ascii(path):
    points = []
    data_type = None
    with open(path, 'r') as f:
        # read header
        while True:
            line = f.readline()
            if not line:
                break
            line = line.strip()
            if line.startswith('DATA'):
                parts = line.split()
                data_type = parts[1] if len(parts) > 1 else ''
                break
        if data_type is None:
            raise RuntimeError('Invalid PCD: missing DATA header')
        if 'ascii' not in data_type.lower():
            raise RuntimeError('Only ASCII PCD files are supported by this script')
        # remaining lines are points
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            except ValueError:
                continue
            points.append((x, y, z))
    return points


class PCDPublisher(Node):
    def __init__(self, pcd_path: str, topic: str = '/gicp_map'):
        super().__init__('pcd_publisher')
        self.topic = topic
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.get_logger().info(f'Reading PCD: {pcd_path}')
        points = read_pcd_ascii(pcd_path)
        self.points = points
        self.get_logger().info(f'Loaded {len(self.points)} points')
        # publish repeatedly after short delay to allow subscribers to connect
        self.create_timer(0.5, self.publish_once)

    def publish_once(self):
        if not self.points:
            self.get_logger().warn('No points to publish')
            return
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        msg = pc2.create_cloud_xyz32(header, self.points)
        self.pub.publish(msg)
        self.get_logger().info(f'Published {len(self.points)} points to {self.topic}')


def main(argv=None):
    if argv is None:
        argv = sys.argv
    if len(argv) < 2:
        print('Usage: publish_pcd.py <pcd_path> [topic]')
        return 2
    pcd_path = argv[1]
    topic = argv[2] if len(argv) > 2 else '/gicp_map'

    rclpy.init()
    node = PCDPublisher(pcd_path, topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
