#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudRangeChecker(Node):
    def __init__(self):
        super().__init__('pointcloud_range_checker')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/gicp_map',
            self.callback,
            10)
        self.get_logger().info('等待点云数据...')

    def callback(self, msg):
        x_coords = []
        y_coords = []
        z_coords = []
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x_coords.append(point[0])
            y_coords.append(point[1])
            z_coords.append(point[2])
        
        if not x_coords:
            self.get_logger().warn('点云为空!')
            return
        
        x_arr = np.array(x_coords)
        y_arr = np.array(y_coords)
        z_arr = np.array(z_coords)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'点云总数: {len(x_coords)} 点')
        self.get_logger().info(f'X 范围: [{x_arr.min():.3f}, {x_arr.max():.3f}]')
        self.get_logger().info(f'Y 范围: [{y_arr.min():.3f}, {y_arr.max():.3f}]')
        self.get_logger().info(f'Z 范围: [{z_arr.min():.3f}, {z_arr.max():.3f}]')
        self.get_logger().info('=' * 50)
        
        # 显示部分点坐标示例
        sample_indices = np.linspace(0, len(x_coords)-1, min(10, len(x_coords)), dtype=int)
        self.get_logger().info('\n示例点坐标:')
        for i in sample_indices:
            self.get_logger().info(f'  点 {i:5d}: x={x_coords[i]:7.3f}, y={y_coords[i]:7.3f}, z={z_coords[i]:7.3f}')
        
        # 只处理一帧后退出
        self.get_logger().info('\n已完成，按 Ctrl+C 退出')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRangeChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
