#!/usr/bin/env python3
"""
测试depth_cluster功能的演示脚本
发布包含地面和多个障碍物的点云数据
用于测试地面提取和障碍物聚类效果
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np
from std_msgs.msg import Header

class DepthClusterTestPublisher(Node):
    def __init__(self):
        super().__init__('depth_cluster_test_publisher')
        
        # 发布原始点云
        self.publisher_ = self.create_publisher(PointCloud2, '/cloud_registered', 10)
        
        # 参数
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('add_noise', True)
        self.declare_parameter('scene_type', 'complex')  # simple, complex, dynamic, slope, steep
        
        rate = self.get_parameter('publish_rate').value
        self.add_noise = self.get_parameter('add_noise').value
        self.scene_type = self.get_parameter('scene_type').value
        
        self.timer = self.create_timer(1.0/rate, self.publish_pointcloud)
        self.count = 0
        
        self.get_logger().info('=== Depth Cluster Test Publisher Started ===')
        self.get_logger().info('Publishing to: /cloud_registered')
        self.get_logger().info(f'Rate: {rate} Hz')
        self.get_logger().info(f'Scene Type: {self.scene_type}')
        self.get_logger().info(f'Add Noise: {self.add_noise}')
        self.get_logger().info('Expected outputs:')
        self.get_logger().info('  - /cloud_baselink (地面点云 - 绿色)')
        self.get_logger().info('  - /cloud_filter_baselink (障碍物聚类 - 彩色)')
        self.get_logger().info('=' * 50)

    def create_ground_plane(self, x_range, y_range, z=-0.2, density=50, slope_x=0.0, slope_y=0.0):
        """创建地面平面点云（可以有斜坡）
        
        Args:
            x_range: X方向范围
            y_range: Y方向范围
            z: 基准高度
            density: 点云密度
            slope_x: X方向斜率（弧度），正值为向前上坡
            slope_y: Y方向斜率（弧度），正值为向左上坡
        """
        points = []
        x_min, x_max = x_range
        y_min, y_max = y_range
        
        x_points = np.linspace(x_min, x_max, density)
        y_points = np.linspace(y_min, y_max, density)
        
        for x in x_points:
            for y in y_points:
                # 计算斜坡高度：z = z0 + x*tan(slope_x) + y*tan(slope_y)
                z_slope = z + x * np.tan(slope_x) + y * np.tan(slope_y)
                # 添加一些轻微起伏
                z_offset = 0.02 * np.sin(x * 0.5) * np.cos(y * 0.5)
                points.append([x, y, z_slope + z_offset])
        
        return points
    
    def create_ramp(self, start_pos, end_pos, width, density=20):
        """创建坡道（斜坡）
        
        Args:
            start_pos: 起点 [x, y, z]
            end_pos: 终点 [x, y, z]
            width: 坡道宽度
            density: 点云密度
        """
        points = []
        x1, y1, z1 = start_pos
        x2, y2, z2 = end_pos
        
        # 沿坡道长度方向的点
        length_points = np.linspace(0, 1, density)
        # 沿宽度方向的点
        width_points = np.linspace(-width/2, width/2, max(2, density//3))
        
        # 坡道方向向量
        dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
        length = np.sqrt(dx**2 + dy**2)
        
        # 垂直于坡道的方向（用于宽度）
        if length > 0:
            perp_x = -dy / length
            perp_y = dx / length
        else:
            perp_x, perp_y = 0, 1
        
        for t in length_points:
            # 沿坡道中心线的点
            center_x = x1 + t * dx
            center_y = y1 + t * dy
            center_z = z1 + t * dz
            
            for w in width_points:
                # 沿宽度方向偏移
                px = center_x + w * perp_x
                py = center_y + w * perp_y
                pz = center_z
                points.append([px, py, pz])
        
        return points

    def create_box_obstacle(self, center, size, density=10):
        """创建长方体障碍物"""
        points = []
        cx, cy, cz = center
        sx, sy, sz = size
        
        # 生成长方体表面点
        x_points = np.linspace(cx - sx/2, cx + sx/2, density)
        y_points = np.linspace(cy - sy/2, cy + sy/2, density)
        z_points = np.linspace(cz, cz + sz, max(2, density//2))
        
        # 前后面
        for x in x_points:
            for z in z_points:
                points.append([x, cy - sy/2, z])
                points.append([x, cy + sy/2, z])
        
        # 左右面
        for y in y_points:
            for z in z_points:
                points.append([cx - sx/2, y, z])
                points.append([cx + sx/2, y, z])
        
        # 顶面
        for x in x_points:
            for y in y_points:
                points.append([x, y, cz + sz])
        
        return points

    def create_cylinder_obstacle(self, center, radius, height, density=20):
        """创建圆柱体障碍物"""
        points = []
        cx, cy, cz = center
        
        # 圆周采样
        angles = np.linspace(0, 2 * np.pi, density)
        z_points = np.linspace(cz, cz + height, max(2, density//2))
        
        for angle in angles:
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            for z in z_points:
                points.append([x, y, z])
        
        # 顶面
        for angle in angles:
            for r in np.linspace(0, radius, density//4):
                x = cx + r * np.cos(angle)
                y = cy + r * np.sin(angle)
                points.append([x, y, cz + height])
        
        return points

    def create_scattered_points(self, x_range, y_range, z_range, count=100):
        """创建散点（模拟噪声或远处物体）"""
        points = []
        for _ in range(count):
            x = np.random.uniform(*x_range)
            y = np.random.uniform(*y_range)
            z = np.random.uniform(*z_range)
            points.append([x, y, z])
        return points

    def publish_pointcloud(self):
        """发布点云"""
        points = []
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'Publishing Scene #{self.count} - Type: {self.scene_type}')
        
        # 1. 地面点云（应该被检测为地面）
        if self.scene_type == 'simple':
            # 简单平地（用于基准测试）
            ground_points = self.create_ground_plane(
                x_range=(-12, 12),
                y_range=(-12, 12),
                z=0.0,
                density=50
            )
            self.get_logger().info(f'Ground points: {len(ground_points)} (flat plane)')
            
        elif self.scene_type == 'slope':
            # 斜坡和高低地形（用于测试斜坡过滤）
            # 平地段（后方）
            ground_flat = self.create_ground_plane(
                x_range=(-10, 0),
                y_range=(-12, 12),
                z=0.0,
                density=45,
                slope_x=0.0,
                slope_y=0.0
            )
            
            # 上坡段（中间）- 15度上坡
            ground_uphill = self.create_ground_plane(
                x_range=(0, 7),
                y_range=(-12, 12),
                z=0.0,
                density=40,
                slope_x=np.radians(15),  # 15度上坡（在阈值内）
                slope_y=0.0
            )
            
            # 下坡段（前方）- 10度下坡
            start_height = 7 * np.tan(np.radians(15))
            ground_downhill = self.create_ground_plane(
                x_range=(7, 13),
                y_range=(-12, 12),
                z=start_height,
                density=40,
                slope_x=np.radians(-10),  # 10度下坡（在阈值内）
                slope_y=0.0
            )
            
            # 横向缓坡（左侧） - 15度横向坡
            ground_ramp_left = self.create_ground_plane(
                x_range=(-8, -2),
                y_range=(-12, 12),
                z=0.3,
                density=40,
                slope_x=0.0,
                slope_y=np.radians(15)  # 15度横向坡（在阈值内）
            )
            
            # 横向缓坡（右侧） - 12度横向坡
            ground_ramp_right = self.create_ground_plane(
                x_range=(2, 8),
                y_range=(-12, 12),
                z=0.3,
                slope_x=0.0,
                slope_y=np.radians(-12)  # 12度横向坡（在阈值内）
            )
            
            ground_points = ground_flat + ground_uphill + ground_downhill + ground_ramp_left + ground_ramp_right
            self.get_logger().info(f'Ground points: {len(ground_points)} (slope terrain)')
            self.get_logger().info(f'  - Flat: {len(ground_flat)}, Uphill: {len(ground_uphill)}, Downhill: {len(ground_downhill)}')
            self.get_logger().info(f'  - Ramp L: {len(ground_ramp_left)}, Ramp R: {len(ground_ramp_right)}')
        
        elif self.scene_type == 'steep':
            # 陡峭坡地测试（测试>30度的拒绝）
            # 平地段
            ground_flat = self.create_ground_plane(
                x_range=(-10, -2),
                y_range=(-12, 12),
                z=0.0,
                density=45,
                slope_x=0.0,
                slope_y=0.0
            )
            
            # 陡峭上坡 - 35度（应该被拒绝）
            ground_steep_up = self.create_ground_plane(
                x_range=(-2, 4),
                y_range=(-12, 12),
                z=0.0,
                density=40,
                slope_x=np.radians(35),  # 35度（超过30度阈值，应被拒绝）
                slope_y=0.0
            )
            
            # 陡峭下坡 - 28度（刚好在阈值内）
            start_height = 6 * np.tan(np.radians(35))
            ground_steep_down = self.create_ground_plane(
                x_range=(4, 10),
                y_range=(-12, 12),
                z=start_height,
                density=40,
                slope_x=np.radians(-28),  # 28度（在阈值内）
                slope_y=0.0
            )
            
            # 非常陡峭 - 50度（肯定被拒绝）
            ground_very_steep = self.create_ground_plane(
                x_range=(10, 13),
                y_range=(-12, 12),
                z=0.0,
                density=30,
                slope_x=np.radians(50),  # 50度（明显超过）
                slope_y=0.0
            )
            
            ground_points = ground_flat + ground_steep_up + ground_steep_down + ground_very_steep
            self.get_logger().info(f'Ground points: {len(ground_points)} (steep terrain test)')
            self.get_logger().info(f'  - Flat: {len(ground_flat)}, Steep 35°: {len(ground_steep_up)}, Steep -28°: {len(ground_steep_down)}, Steep 50°: {len(ground_very_steep)}')
            self.get_logger().info(f'  ⚠ Expected: Flat accepted, 35° rejected, 28° accepted, 50° rejected')
        else:
            # complex/dynamic 场景 - 默认混合地形
            ground_flat = self.create_ground_plane(
                x_range=(-12, 5),
                y_range=(-12, 12),
                z=0.0,
                density=50
            )
            
            ground_uphill = self.create_ground_plane(
                x_range=(5, 12),
                y_range=(-12, 12),
                z=0.0,
                density=45,
                slope_x=np.radians(12),  # 12度上坡
                slope_y=0.0
            )
            
            ground_points = ground_flat + ground_uphill
            self.get_logger().info(f'Ground points: {len(ground_points)} (mixed terrain)')
        
        # 2. 根据场景类型创建障碍物
        obstacle_count = 0
        obstacles_points = []
        
        # 简化：仅在complex和dynamic场景中添加小障碍物（不是柱子）
        if self.scene_type in ['complex', 'dynamic']:
            # 小障碍物1：前方小盒子
            small_box1 = self.create_box_obstacle(
                center=[3.0, 2.0, 0.2],
                size=[0.4, 0.4, 0.6],
                density=6
            )
            obstacles_points.extend(small_box1)
            obstacle_count += 1
            
            # 小障碍物2：侧面小盒子
            small_box2 = self.create_box_obstacle(
                center=[7.0, -2.5, 0.3],
                size=[0.5, 0.5, 0.7],
                density=6
            )
            obstacles_points.extend(small_box2)
            obstacle_count += 1
            
            self.get_logger().info(f'Obstacle points: {len(obstacles_points)} ({obstacle_count} small boxes)')
        else:
            self.get_logger().info(f'Obstacle points: 0 (pure ground testing)')
        
        points = obstacles_points.copy()
        
        # 3. 添加最小化的散点噪声
        
        # 合并地面和障碍物
        all_points = ground_points + obstacles_points
        
        # 添加最小化的噪声
        if self.add_noise:
            noise_level = 0.01  # 1cm
            for point in all_points:
                point[0] += np.random.normal(0, noise_level)
                point[1] += np.random.normal(0, noise_level)
                point[2] += np.random.normal(0, noise_level * 0.5)
        
        # 统计信息
        total_points = len(all_points)
        self.get_logger().info(f'Total points: {total_points}')
        self.get_logger().info(f'  - Ground: {len(ground_points)} (应显示为绿色)')
        self.get_logger().info(f'  - Obstacles: {len(obstacles_points)} ({obstacle_count} 个小盒子)')
        self.get_logger().info(f'Expected: Ground detected + minimal obstacles')
        
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
        
        # 定义点云字段 (仅XYZ，depth_cluster使用PointXYZ)
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
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = DepthClusterTestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
