#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class TestPointCloudPublisher(Node):
    def __init__(self):
        super().__init__('test_pointcloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/gicp_map', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)
        self.get_logger().info('Test PointCloud Publisher started - publishing to /gicp_map')

    def publish_pointcloud(self):
        # 创建高密度复杂测试点云 - 真实场景模拟
        points = []
        
        # 1. 主平地区域 (高密度) - 坡度 ~0度
        for i in range(500):
            x = np.random.uniform(-3, 3)
            y = np.random.uniform(-3, -0.5)  # 机器人后方的平地
            z = -0.5 + np.random.normal(0, 0.015)  # 轻微噪声模拟真实地面
            points.append([x, y, z])
        
        # 2. 凹凸不平的草地区域（模拟自然地形）
        for i in range(400):
            x = np.random.uniform(-4, -2)
            y = np.random.uniform(-2, 2)
            # 使用正弦波创建起伏地形
            z = -0.4 + 0.1 * np.sin(x * 2) + 0.08 * np.cos(y * 1.5) + np.random.normal(0, 0.02)
            points.append([x, y, z])
        
        # 3. 轻微上坡 (~5度) - 密集采样
        for i in range(400):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(-0.5, 1.5)
            slope_angle = 5
            z = -0.5 + (y + 0.5) * np.tan(np.radians(slope_angle)) + np.random.normal(0, 0.02)
            points.append([x, y, z])
        
        # 4. 中等斜坡 (~15度) - 密集采样
        for i in range(400):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(1.5, 3)
            slope_angle = 15
            base_z = -0.5 + 2 * np.tan(np.radians(5))
            z = base_z + (y - 1.5) * np.tan(np.radians(slope_angle)) + np.random.normal(0, 0.02)
            points.append([x, y, z])
        
        # 5. 较陡斜坡 (~25度 - 接近阈值)
        for i in range(350):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(3, 4)
            slope_angle = 25
            base_z = -0.5 + 2 * np.tan(np.radians(5)) + 1.5 * np.tan(np.radians(15))
            z = base_z + (y - 3) * np.tan(np.radians(slope_angle)) + np.random.normal(0, 0.02)
            points.append([x, y, z])
        
        # 6. 陡坡障碍物 (~40度 - 应该被识别为障碍物)
        for i in range(350):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(4, 5)
            slope_angle = 40
            base_z = -0.5 + 2 * np.tan(np.radians(5)) + 1.5 * np.tan(np.radians(15)) + np.tan(np.radians(25))
            z = base_z + (y - 4) * np.tan(np.radians(slope_angle)) + np.random.normal(0, 0.02)
            points.append([x, y, z])
        
        # 7. 楼梯结构 (多级台阶) - 每级更密集
        step_height = 0.15
        step_depth = 0.4
        for step in range(5):  # 5级台阶
            for i in range(200):  # 每级200点
                x = np.random.uniform(2, 4)
                y = np.random.uniform(step * step_depth, (step + 1) * step_depth)
                z = -0.3 + step * step_height + np.random.normal(0, 0.01)
                points.append([x, y, z])
            # 台阶立面（垂直部分）
            for i in range(100):
                x = np.random.uniform(2, 4)
                y = (step + 1) * step_depth + np.random.normal(0, 0.02)
                z = np.random.uniform(-0.3 + step * step_height, -0.3 + (step + 1) * step_height)
                points.append([x, y, z])
        
        # 8. 侧面平台（高于主平地）- 更密集
        for i in range(400):
            x = np.random.uniform(4, 6)
            y = np.random.uniform(-2, 3)
            z = 0.2 + np.random.normal(0, 0.015)
            points.append([x, y, z])
        
        # 9. 下凹区域（坑洼）- 更详细
        for i in range(300):
            x = np.random.uniform(-1, 1)
            y = np.random.uniform(-2, -1)
            # 创建碗状凹陷
            dist_from_center = np.sqrt((x + 0.5)**2 + (y + 1.5)**2)
            z = -0.5 - 0.3 * np.exp(-dist_from_center**2) + np.random.normal(0, 0.01)
            points.append([x, y, z])
        
        # 10. 垂直墙壁 (多个高度) - 更高密度
        for i in range(400):
            x = np.random.uniform(-1, 1)
            y = 6.0 + np.random.normal(0, 0.05)
            z = np.random.uniform(-0.5, 2.0)  # 高墙
            points.append([x, y, z])
        # 墙壁表面纹理
        for i in range(200):
            x = np.random.uniform(-1, 1)
            y = 6.0 + np.random.normal(0, 0.02)
            z = np.random.uniform(-0.5, 2.0)
            # 添加砖块纹理
            z += 0.02 * np.sin(z * 20) * np.cos(x * 15)
            points.append([x, y, z])
        
        # 11. 树干或柱子（垂直圆柱体）- 更详细
        for pole in range(3):  # 3根柱子
            pole_x = -3 + pole * 3
            pole_y = 0
            for i in range(150):  # 每根柱子150点
                theta = np.random.uniform(0, 2 * np.pi)
                radius = 0.15 + np.random.normal(0, 0.01)
                x = pole_x + radius * np.cos(theta)
                y = pole_y + radius * np.sin(theta)
                z = np.random.uniform(-0.5, 1.5)
                points.append([x, y, z])
            # 柱子顶部
            for i in range(50):
                theta = np.random.uniform(0, 2 * np.pi)
                radius = np.random.uniform(0, 0.15)
                x = pole_x + radius * np.cos(theta)
                y = pole_y + radius * np.sin(theta)
                z = 1.5 + np.random.normal(0, 0.02)
                points.append([x, y, z])
        
        # 12. 散落的石头/障碍物（小型凸起）- 更多细节
        for rock in range(20):  # 增加到20个石头
            rock_x = np.random.uniform(-5, 5)
            rock_y = np.random.uniform(-2, 4)
            rock_height = np.random.uniform(0.1, 0.4)
            rock_radius = np.random.uniform(0.2, 0.4)
            for i in range(100):  # 每个石头100点
                offset_x = np.random.normal(0, rock_radius * 0.5)
                offset_y = np.random.normal(0, rock_radius * 0.5)
                x = rock_x + offset_x
                y = rock_y + offset_y
                dist = np.sqrt(offset_x**2 + offset_y**2)
                z = -0.5 + rock_height * np.exp(-dist**2 / (rock_radius**2)) + np.random.normal(0, 0.01)
                points.append([x, y, z])
        
        # 13. 远处高密度区域（模拟远方地形）
        for i in range(500):
            x = np.random.uniform(-6, 6)
            y = np.random.uniform(5, 8)
            z = 0.5 + 0.3 * np.sin(x * 0.5) + 0.2 * np.cos(y * 0.3) + np.random.normal(0, 0.05)
            points.append([x, y, z])
        
        # 14. 侧面陡峭边缘 - 更密集
        for i in range(300):
            x = 6.5 + np.random.normal(0, 0.05)
            y = np.random.uniform(-2, 4)
            z = np.random.uniform(-0.5, 0.8)
            points.append([x, y, z])
        
        # 15. 地面上的密集细节点（模拟高分辨率扫描）
        for i in range(800):
            x = np.random.uniform(-1.5, 1.5)
            y = np.random.uniform(0.5, 2.5)
            # 添加微小的纹理细节
            z = -0.5 + 0.02 * np.sin(x * 10) * np.cos(y * 10) + np.random.normal(0, 0.005)
            points.append([x, y, z])
        
        # 16. 车辙/轮胎痕迹（模拟道路）
        for track in range(2):  # 两条车辙
            track_y_offset = -0.3 + track * 0.6
            for i in range(300):
                x = np.random.uniform(-2, 2)
                y = np.random.uniform(0, 3) + track_y_offset
                # 创建凹陷的车辙
                dist_from_track = abs(y - track_y_offset)
                z = -0.5 - 0.05 * np.exp(-dist_from_track**2 / 0.01) + np.random.normal(0, 0.01)
                points.append([x, y, z])
        
        # 17. 栅栏/围栏结构
        for post in range(6):  # 6根栅栏柱
            post_x = -5
            post_y = -2 + post * 1.0
            # 栅栏柱
            for i in range(80):
                x = post_x + np.random.normal(0, 0.02)
                y = post_y + np.random.normal(0, 0.02)
                z = np.random.uniform(-0.5, 0.8)
                points.append([x, y, z])
            # 横杆
            if post < 5:
                for i in range(100):
                    x = post_x + np.random.normal(0, 0.01)
                    y = np.random.uniform(post_y, post_y + 1.0)
                    z = 0.4 + np.random.normal(0, 0.01)
                    points.append([x, y, z])
        
        # 18. 斜坡上的小石子（散点细节）
        for i in range(500):
            x = np.random.uniform(-2, 2)
            y = np.random.uniform(1, 4)
            # 基础斜坡高度
            if y < 1.5:
                base_z = -0.5 + (y + 0.5) * np.tan(np.radians(5))
            elif y < 3:
                base_z = -0.5 + 2 * np.tan(np.radians(5)) + (y - 1.5) * np.tan(np.radians(15))
            else:
                base_z = -0.5 + 2 * np.tan(np.radians(5)) + 1.5 * np.tan(np.radians(15)) + (y - 3) * np.tan(np.radians(25))
            # 添加小石子突起
            z = base_z + np.random.uniform(0, 0.05) + np.random.normal(0, 0.01)
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
        self.get_logger().info(
            f'Published {len(points)} points - 高密度真实场景: '
            f'平地+斜坡+楼梯+墙壁+柱子+石头+栅栏+车辙+细节纹理'
        )

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
