#!/usr/bin/env python3
"""
监控点云过滤效果的脚本
订阅原始点云和过滤后点云，对比统计信息
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
from collections import deque
import time

class FilterMonitor(Node):
    def __init__(self):
        super().__init__('filter_monitor')
        
        # 订阅原始点云和过滤后的点云
        self.raw_sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.raw_callback,
            10
        )
        
        self.filtered_sub = self.create_subscription(
            PointCloud2,
            '/cloud_filtered',
            self.filtered_callback,
            10
        )
        
        # 统计数据
        self.raw_count = 0
        self.filtered_count = 0
        self.raw_history = deque(maxlen=10)
        self.filtered_history = deque(maxlen=10)
        self.last_print_time = time.time()
        
        # 定时打印统计信息
        self.timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('=== Filter Effect Monitor Started ===')
        self.get_logger().info('Monitoring topics:')
        self.get_logger().info('  Input:  /cloud_registered')
        self.get_logger().info('  Output: /cloud_filtered')
        self.get_logger().info('======================================\n')

    def raw_callback(self, msg):
        """处理原始点云"""
        point_count = msg.width * msg.height
        self.raw_count = point_count
        self.raw_history.append(point_count)

    def filtered_callback(self, msg):
        """处理过滤后的点云"""
        point_count = msg.width * msg.height
        self.filtered_count = point_count
        self.filtered_history.append(point_count)

    def print_statistics(self):
        """打印统计信息"""
        if not self.raw_history or not self.filtered_history:
            self.get_logger().warn('Waiting for point cloud data...')
            return
        
        # 计算平均值
        avg_raw = sum(self.raw_history) / len(self.raw_history)
        avg_filtered = sum(self.filtered_history) / len(self.filtered_history)
        avg_removed = avg_raw - avg_filtered
        
        if avg_raw > 0:
            filter_rate = (avg_removed / avg_raw) * 100
        else:
            filter_rate = 0
        
        # 打印信息
        print('\n' + '='*70)
        print(f'  Point Cloud Filter Statistics  ')
        print('='*70)
        print(f'Current Frame:')
        print(f'  Input Points:    {self.raw_count:6d}')
        print(f'  Filtered Points: {self.filtered_count:6d}')
        print(f'  Removed Points:  {self.raw_count - self.filtered_count:6d}')
        print(f'-'*70)
        print(f'Average (last {len(self.raw_history)} frames):')
        print(f'  Input Points:    {avg_raw:6.1f}')
        print(f'  Filtered Points: {avg_filtered:6.1f}')
        print(f'  Removed Points:  {avg_removed:6.1f}  ({filter_rate:.1f}%)')
        print('='*70)
        
        # 性能提示
        if filter_rate > 50:
            print(f'⚠️  Warning: High filter rate ({filter_rate:.1f}%)')
            print(f'   Check if filter regions are configured correctly')
        elif filter_rate < 5:
            print(f'ℹ️  Info: Low filter rate ({filter_rate:.1f}%)')
            print(f'   Most points are outside filter regions')
        else:
            print(f'✓ Filter working normally ({filter_rate:.1f}% filtered)')
        
        print('='*70 + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = FilterMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\nShutting down monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
