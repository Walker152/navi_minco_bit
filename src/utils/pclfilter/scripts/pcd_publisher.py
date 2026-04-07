#!/usr/bin/env python3
"""
从 PCD 文件读取点云并发布
用于测试地面检测算法
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import os
import subprocess

class PCDPublisher(Node):
    def __init__(self, pcd_file):
        super().__init__('pcd_publisher')
        
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        
        # 参数
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('pcd_file', pcd_file)
        
        rate = self.get_parameter('publish_rate').value
        pcd_path = self.get_parameter('pcd_file').value
        
        self.timer = self.create_timer(1.0/rate, self.publish_pointcloud)
        self.count = 0
        self.pcd_file = pcd_path
        
        self.get_logger().info(f'=== PCD Publisher Started ===')
        self.get_logger().info(f'PCD file: {pcd_path}')
        self.get_logger().info(f'Publish rate: {rate} Hz')
        self.get_logger().info(f'Publishing to: /cloud_registered')
        self.get_logger().info('=' * 50)

    def read_pcd(self, filename):
        """读取 PCD 文件"""
        try:
            # 使用 pcl_pcd2ply 或直接 Python 读取
            import subprocess
            import numpy as np
            
            # 用 PCL 工具转换为 ASCII 并读取
            result = subprocess.run(
                ['python3', '-c', f"""
import struct
import numpy as np

with open('{filename}', 'rb') as f:
    # 读取 PCD 头
    lines = []
    while True:
        line = f.readline().decode('utf-8').strip()
        if line.startswith('DATA'):
            break
        lines.append(line)
    
    # 解析头信息
    points = []
    for line in lines:
        if line.startswith('POINTS'):
            num_points = int(line.split()[1])
        if line.startswith('WIDTH'):
            width = int(line.split()[1])
        if line.startswith('FIELDS'):
            fields = line.split()[1:]
    
    # 读取数据
    fmt = f'{num_points}f'
    num_floats = num_points * len(fields)
    data = struct.unpack(fmt, f.read(num_floats * 4))
    
    # 重新组织数据
    points = []
    for i in range(num_points):
        x = data[i * len(fields)]
        y = data[i * len(fields) + 1]
        z = data[i * len(fields) + 2]
        points.append([x, y, z])
    
    import json
    print(json.dumps(points))
"""],
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                import json
                points = json.loads(result.stdout)
                self.get_logger().info(f'Successfully read {len(points)} points from {filename}')
                return points
            else:
                raise Exception(f"Failed to read PCD: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f'Error reading PCD: {e}')
            return []

    def publish_pointcloud(self):
        """发布点云"""
        try:
            # 读取 PCD 文件
            points = self.read_pcd(self.pcd_file)
            
            if not points:
                self.get_logger().warn('No points to publish')
                return
            
            self.get_logger().info(f'Publishing frame #{self.count}: {len(points)} points')
            
            # 创建 PointCloud2 消息
            msg = PointCloud2()
            msg.header.frame_id = "camera_init"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.height = 1
            msg.width = len(points)
            msg.is_dense = False
            msg.is_bigendian = False
            
            # 设置字段
            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            
            msg.point_step = 12  # 3 * 4 bytes
            msg.row_step = msg.point_step * len(points)
            
            # 填充数据
            data = b''
            for x, y, z in points:
                data += struct.pack('f', float(x))
                data += struct.pack('f', float(y))
                data += struct.pack('f', float(z))
            
            msg.data = data
            
            # 发布
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published successfully!')
            
            self.count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error publishing: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    # 查找 PCD 文件
    pcd_path = '/home/rm/2025-sentry-navi/src/perception/icp_relocalization/maps/rmuc.pcd'
    
    if not os.path.exists(pcd_path):
        print(f"PCD file not found: {pcd_path}")
        return
    
    node = PCDPublisher(pcd_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
