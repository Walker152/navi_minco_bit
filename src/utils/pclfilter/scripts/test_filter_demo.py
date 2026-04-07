#!/usr/bin/env python3
"""
测试点云过滤效果的演示脚本
发布包含多个区域的点云数据，用于测试过滤功能
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
from std_msgs.msg import Header

class FilterTestPublisher(Node):
    def __init__(self):
        super().__init__('filter_test_publisher')
        
        # 发布原始点云
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        
        # 参数
        self.declare_parameter('publish_rate', 2.0)  # 发布频率
        self.declare_parameter('add_noise', True)    # 是否添加噪声
        self.declare_parameter('animate', False)      # 是否动画效果
        
        rate = self.get_parameter('publish_rate').value
        self.add_noise = self.get_parameter('add_noise').value
        self.animate = self.get_parameter('animate').value
        
        self.timer = self.create_timer(1.0/rate, self.publish_pointcloud)
        self.count = 0
        
        self.get_logger().info('=== Filter Test Publisher Started ===')
        self.get_logger().info('Publishing to: /cloud_registered')
        self.get_logger().info(f'Rate: {rate} Hz')
        self.get_logger().info(f'Noise: {self.add_noise}')
        self.get_logger().info('=====================================')

    def create_grid_points(self, x_range, y_range, z_range, density=10):
        """创建网格点云"""
        points = []
        x_min, x_max = x_range
        y_min, y_max = y_range
        z_min, z_max = z_range
        
        x_points = np.linspace(x_min, x_max, density)
        y_points = np.linspace(y_min, y_max, density)
        z_points = np.linspace(z_min, z_max, max(2, density//2))
        
        for x in x_points:
            for y in y_points:
                for z in z_points:
                    points.append([x, y, z])
        
        return points

    def create_random_points(self, x_range, y_range, z_range, count=100):
        """创建随机点云"""
        points = []
        for _ in range(count):
            x = np.random.uniform(*x_range)
            y = np.random.uniform(*y_range)
            z = np.random.uniform(*z_range)
            points.append([x, y, z])
        return points

    def publish_pointcloud(self):
        points = []
        
        # 根据cube.yaml配置:
        # 区域1: [0.0-10.0, -5.0-5.0, 0.0-3.0] - 应该被过滤
        # 区域2: [20.0-30.0, 0.0-10.0, 0.0-3.0] - 应该被过滤
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'Publishing Point Cloud #{self.count}')
        
        # 1. 创建应该被保留的点云（区域外）
        # 区域A: 负x方向（机器人后方）
        points_keep_1 = self.create_random_points(
            x_range=(-10, -1),
            y_range=(-5, 5),
            z_range=(0, 2),
            count=150
        )
        
        # 区域B: 中间区域 (10-20 m)
        points_keep_2 = self.create_random_points(
            x_range=(10, 20),
            y_range=(-5, 10),
            z_range=(0, 2),
            count=150
        )
        
        # 区域C: 远距离区域
        points_keep_3 = self.create_random_points(
            x_range=(30, 40),
            y_range=(-5, 10),
            z_range=(0, 2),
            count=100
        )
        
        # 2. 创建应该被过滤的点云
        # 过滤区域1: [0-10, -5-5, 0-3]
        if self.animate:
            # 动画效果：点云移动穿过过滤区域
            offset = (self.count % 20) - 10
            points_filter_1 = self.create_grid_points(
                x_range=(2 + offset, 8 + offset),
                y_range=(-3, 3),
                z_range=(0.5, 2.5),
                density=8
            )
        else:
            points_filter_1 = self.create_grid_points(
                x_range=(2, 8),
                y_range=(-3, 3),
                z_range=(0.5, 2.5),
                density=8
            )
        
        # 过滤区域2: [20-30, 0-10, 0-3]
        points_filter_2 = self.create_grid_points(
            x_range=(22, 28),
            y_range=(2, 8),
            z_range=(0.5, 2.5),
            density=6
        )
        
        # 3. 边界测试点（刚好在过滤区域边缘）
        boundary_points = [
            # 区域1边界
            [0.0, 0.0, 1.0],   # 刚好在min边界
            [10.0, 0.0, 1.0],  # 刚好在max边界
            [-0.1, 0.0, 1.0],  # 刚好外面
            [10.1, 0.0, 1.0],  # 刚好外面
            # 区域2边界
            [20.0, 5.0, 1.0],
            [30.0, 5.0, 1.0],
            [19.9, 5.0, 1.0],
            [30.1, 5.0, 1.0],
        ]
        
        # 合并所有点
        all_points = (points_keep_1 + points_keep_2 + points_keep_3 + 
                      points_filter_1 + points_filter_2 + boundary_points)
        
        # 添加噪声
        if self.add_noise:
            noise_level = 0.02  # 2cm噪声
            for point in all_points:
                point[0] += np.random.normal(0, noise_level)
                point[1] += np.random.normal(0, noise_level)
                point[2] += np.random.normal(0, noise_level)
        
        # 统计信息
        total_points = len(all_points)
        keep_points = len(points_keep_1) + len(points_keep_2) + len(points_keep_3)
        filter_points = len(points_filter_1) + len(points_filter_2)
        boundary_count = len(boundary_points)
        
        self.get_logger().info(f'Total Points: {total_points}')
        self.get_logger().info(f'  - Should KEEP: {keep_points} (outside filter regions)')
        self.get_logger().info(f'  - Should FILTER: {filter_points} (inside filter regions)')
        self.get_logger().info(f'  - Boundary Test: {boundary_count} (edge cases)')
        self.get_logger().info(f'Expected filtered output: ~{keep_points} points')
        
        # 创建PointCloud2消息
        msg = self.create_pointcloud2_msg(all_points)
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Published successfully!')
        self.get_logger().info(f'{"="*60}\n')
        
        self.count += 1

    def create_pointcloud2_msg(self, points):
        """创建PointCloud2消息"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        
        # 定义点云字段 (XYZ + 强度)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.point_step = 16  # 4 * 4 bytes (x, y, z, intensity)
        msg.row_step = msg.point_step * msg.width
        
        # 打包点云数据
        buffer = []
        for i, point in enumerate(points):
            # 根据位置设置不同的强度值，便于可视化
            if 0 <= point[0] <= 10 and -5 <= point[1] <= 5:  # 过滤区域1
                intensity = 255.0  # 红色（应被过滤）
            elif 20 <= point[0] <= 30 and 0 <= point[1] <= 10:  # 过滤区域2
                intensity = 200.0  # 橙色（应被过滤）
            else:
                intensity = 100.0  # 蓝色（应保留）
            
            buffer.append(struct.pack('ffff', point[0], point[1], point[2], intensity))
        
        msg.data = b''.join(buffer)
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = FilterTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
