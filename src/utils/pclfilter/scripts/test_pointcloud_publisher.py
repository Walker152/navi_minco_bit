#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class TestPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('test_pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)
        self.get_logger().info('Test PointCloud Publisher started')

    def publish_pointcloud(self):
        # 创建测试点云数据
        # 包含一些在过滤区域内的点和区域外的点
        points = []
        
        # 区域外的点（应该保留）
        for i in range(50):
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(-5, -1)  # y < 0, 在机器人后方
            z = np.random.uniform(-1, 1)
            points.append([x, y, z])
        
        # 区域内的点（根据cube.yaml配置，应该被过滤）
        # cube: min: [-0.3, 0.3, -0.3], max: [0.3, 1.2, 0.3]
        for i in range(30):
            x = np.random.uniform(-0.2, 0.2)  # 在-0.3到0.3之间
            y = np.random.uniform(0.4, 1.1)   # 在0.3到1.2之间
            z = np.random.uniform(-0.2, 0.2)  # 在-0.3到0.3之间
            points.append([x, y, z])
        
        # 更多区域外的点
        for i in range(50):
            x = np.random.uniform(-5, 5)
            y = np.random.uniform(2, 10)  # y > 1.2, 在机器人前方
            z = np.random.uniform(-1, 1)
            points.append([x, y, z])
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_init'
        
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False
        
        # 定义点云字段
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.point_step = 12  # 3 * 4 bytes
        msg.row_step = msg.point_step * msg.width
        
        # 打包点云数据
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = b''.join(buffer)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {len(points)} points (should filter ~30 points in cube region)')

def main(args=None):
    rclpy.init(args=args)
    node = TestPointCloudPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
