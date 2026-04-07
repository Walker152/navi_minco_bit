#!/usr/bin/env python3
"""
监控depth_cluster效果的脚本
订阅输入点云、地面点云和障碍物聚类结果，显示统计信息
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from collections import deque
import time

class DepthClusterMonitor(Node):
    def __init__(self):
        super().__init__('depth_cluster_monitor')
        
        # 订阅三个话题
        self.raw_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.raw_callback,
            10
        )
        
        self.ground_sub = self.create_subscription(
            PointCloud2,
            '/cloud_baselink',
            self.ground_callback,
            10
        )
        
        self.cluster_sub = self.create_subscription(
            PointCloud2,
            '/cloud_filter_baselink',
            self.cluster_callback,
            10
        )
        
        # 统计数据
        self.raw_count = 0
        self.ground_count = 0
        self.cluster_count = 0
        
        self.raw_history = deque(maxlen=10)
        self.ground_history = deque(maxlen=10)
        self.cluster_history = deque(maxlen=10)
        
        # 定时打印统计信息
        self.timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('=== Depth Cluster Monitor Started ===')
        self.get_logger().info('Monitoring topics:')
        self.get_logger().info('  Input:    /cloud_registered')
        self.get_logger().info('  Ground:   /cloud_baselink (绿色)')
        self.get_logger().info('  Clusters: /cloud_filter_baselink (彩色)')
        self.get_logger().info('=' * 50 + '\n')

    def raw_callback(self, msg):
        """处理原始点云"""
        point_count = msg.width * msg.height
        self.raw_count = point_count
        self.raw_history.append(point_count)

    def ground_callback(self, msg):
        """处理地面点云"""
        point_count = msg.width * msg.height
        self.ground_count = point_count
        self.ground_history.append(point_count)

    def cluster_callback(self, msg):
        """处理聚类结果点云"""
        point_count = msg.width * msg.height
        self.cluster_count = point_count
        self.cluster_history.append(point_count)

    def print_statistics(self):
        """打印统计信息"""
        if not self.raw_history or not self.ground_history or not self.cluster_history:
            self.get_logger().warn('Waiting for point cloud data...')
            return
        
        # 计算平均值
        avg_raw = sum(self.raw_history) / len(self.raw_history)
        avg_ground = sum(self.ground_history) / len(self.ground_history)
        avg_cluster = sum(self.cluster_history) / len(self.cluster_history)
        
        # 计算百分比
        if avg_raw > 0:
            ground_ratio = (avg_ground / avg_raw) * 100
            cluster_ratio = (avg_cluster / avg_raw) * 100
            other_ratio = 100 - ground_ratio - cluster_ratio
        else:
            ground_ratio = cluster_ratio = other_ratio = 0
        
        # 打印信息
        print('\n' + '='*70)
        print(f'  Depth Cluster Statistics  ')
        print('='*70)
        print(f'Current Frame:')
        print(f'  Input Points:     {self.raw_count:6d}')
        print(f'  Ground Points:    {self.ground_count:6d} (绿色 - 地面)')
        print(f'  Cluster Points:   {self.cluster_count:6d} (彩色 - 障碍物)')
        print(f'  Other/Filtered:   {self.raw_count - self.ground_count - self.cluster_count:6d}')
        print(f'-'*70)
        print(f'Average (last {len(self.raw_history)} frames):')
        print(f'  Input Points:     {avg_raw:8.1f}')
        print(f'  Ground Points:    {avg_ground:8.1f}  ({ground_ratio:5.1f}%)')
        print(f'  Cluster Points:   {avg_cluster:8.1f}  ({cluster_ratio:5.1f}%)')
        print(f'  Other/Filtered:   {avg_raw - avg_ground - avg_cluster:8.1f}  ({other_ratio:5.1f}%)')
        print('='*70)
        
        # 性能分析
        print('Analysis:')
        if ground_ratio > 70:
            print(f'  ⚠️  High ground ratio ({ground_ratio:.1f}%)')
            print(f'     → Scene is mostly flat ground')
        elif ground_ratio < 20:
            print(f'  ⚠️  Low ground ratio ({ground_ratio:.1f}%)')
            print(f'     → Check ground detection parameters')
        else:
            print(f'  ✓ Ground detection: {ground_ratio:.1f}% (normal)')
        
        if cluster_ratio > 60:
            print(f'  ℹ️  High obstacle ratio ({cluster_ratio:.1f}%)')
            print(f'     → Scene has many obstacles')
        elif cluster_ratio < 5:
            print(f'  ℹ️  Low obstacle ratio ({cluster_ratio:.1f}%)')
            print(f'     → Scene is mostly empty or ground')
        else:
            print(f'  ✓ Obstacle clustering: {cluster_ratio:.1f}% (normal)')
        
        if other_ratio > 30:
            print(f'  ⚠️  Many unclassified points ({other_ratio:.1f}%)')
            print(f'     → May need to adjust clustering parameters')
        
        print('='*70 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = DepthClusterMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nShutting down monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
