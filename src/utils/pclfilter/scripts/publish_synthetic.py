#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2
#!/usr/bin/env python3
"""发布更复杂的合成点云：倾斜平面 + 障碍物 + 随机簇 + 高斯噪声

用法: python3 publish_synthetic.py <tilt_deg> [topic] [complexity]
complexity: 1 (简单平面) / 2 (平面+噪声+少量障碍) / 3 (更复杂：多障碍+随机簇)
"""
import sys
import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2


class SyntheticPublisher(Node):
    def __init__(self, tilt_deg=0.0, topic='/gicp_map', complexity=2, noise_std=0.01):
        super().__init__('synthetic_publisher')
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.timer = self.create_timer(0.5, self.publish)
        self.tilt_deg = float(tilt_deg)
        self.tilt_rad = math.radians(self.tilt_deg)
        self.topic = topic
        self.complexity = int(complexity)
        # 默认不添加随机噪声以保证每次发布稳定相同点云
        self.noise_std = float(noise_std)
        # 使用固定种子生成确定性点云
        self.rng = random.Random(0)
        self.get_logger().info(f'Preparing deterministic synthetic cloud: tilt={self.tilt_deg}°, complexity={self.complexity} to {topic}')
        # 预生成一次点云，后续 publish 只发布相同的数据
        self.cloud = self.build_cloud()
        self.get_logger().info(f'Prepared {len(self.cloud)} points (stable)')

    def generate_plane(self, size=8.0, step=0.05):
        points = []
        n = int(size / step)
        half = n / 2
        for i in range(n):
            x = (i - half) * step
            for j in range(n):
                y = (j - half) * step
                z = 0.0
                points.append([x, y, z])
        return points

    def add_obstacles(self, points, count=5, radius=0.3, height=0.5, density=200):
        for _ in range(count):
            cx = self.rng.uniform(-2.5, 2.5)
            cy = self.rng.uniform(-2.5, 2.5)
            for k in range(density):
                r = radius * math.sqrt(self.rng.random())
                ang = self.rng.random() * 2 * math.pi
                x = cx + r * math.cos(ang)
                y = cy + r * math.sin(ang)
                z = self.rng.random() * height
                points.append([x, y, z])
        return points

    def add_random_clusters(self, points, clusters=6):
        for _ in range(clusters):
            cx = self.rng.uniform(-4.0, 4.0)
            cy = self.rng.uniform(-4.0, 4.0)
            for k in range(self.rng.randint(30, 150)):
                x = cx + self.rng.gauss(0, 0.2)
                y = cy + self.rng.gauss(0, 0.2)
                z = self.rng.gauss(0.5, 0.3)
                points.append([x, y, z])
        return points

    def add_sloped_plane(self, points, center=(0,0), size=1.0, step=0.05, tilt_x=0.0, tilt_y=0.0):
        # 在指定中心处添加一个小斜坡（局部平面）：z = a*x + b*y
        cx, cy = center
        n = int(size / step)
        half = n / 2
        # tilt_x, tilt_y 表示每米的高度变化（斜率），可由角度近似为 tan(angle)
        for i in range(n):
            for j in range(n):
                x = cx + (i - half) * step
                y = cy + (j - half) * step
                z = tilt_x * (x - cx) + tilt_y * (y - cy)
                points.append([x, y, z])
        return points

    def rotate_pitch(self, pts):
        cr = math.cos(self.tilt_rad)
        sr = math.sin(self.tilt_rad)
        rotated = []
        for p in pts:
            x, y, z = p
            x_r = cr * x + sr * z
            z_r = -sr * x + cr * z
            if self.noise_std > 0.0:
                nx = self.rng.gauss(0, self.noise_std)
                ny = self.rng.gauss(0, self.noise_std)
                nz = self.rng.gauss(0, self.noise_std)
            else:
                nx = ny = nz = 0.0
            rotated.append((x_r + nx, y + ny, z_r + nz))
        return rotated

    def build_cloud(self):
        # 构建一次性确定性点云（不在 publish 时重新生成）
        pts = self.generate_plane(size=8.0, step=0.06)

        if self.complexity >= 2:
            pts = self.add_obstacles(pts, count=6, radius=0.3, height=0.8, density=220)
            pts = self.add_random_clusters(pts, clusters=4)
            # 基于传感器倾角生成几个不同斜度的斜坡（包含更陡的角度）
            base = self.tilt_deg
            ramps = [((-2.0, -1.5), max(10.0, base - 10.0), 0.0),
                     ((1.5, 0.5), base, 0.0),
                     ((0.0, 2.0), base + 10.0, 5.0)]
            for center, ax_deg, ay_deg in ramps:
                a = math.tan(math.radians(ax_deg))
                b = math.tan(math.radians(ay_deg))
                pts = self.add_sloped_plane(pts, center=center, size=1.2, step=0.04, tilt_x=a, tilt_y=b)

        if self.complexity >= 3:
            pts = self.add_obstacles(pts, count=8, radius=0.5, height=1.2, density=400)
            pts = self.add_random_clusters(pts, clusters=8)
            # 更大的更陡坡（超越传感器倾角）
            big_ramps = [((-3.0, 2.0), max(30.0, base + 15.0), 10.0),
                         ((2.5, -2.5), max(45.0, base + 30.0), 5.0)]
            for center, ax_deg, ay_deg in big_ramps:
                a = math.tan(math.radians(ax_deg))
                b = math.tan(math.radians(ay_deg))
                pts = self.add_sloped_plane(pts, center=center, size=2.0, step=0.06, tilt_x=a, tilt_y=b)

        rotated = self.rotate_pitch(pts)
        return rotated

    def publish(self):
        # 直接发布预生成的稳定点云
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        msg = pc2.create_cloud_xyz32(header, self.cloud)
        self.pub.publish(msg)
        self.get_logger().debug(f'Published {len(self.cloud)} stable synthetic points to {self.topic}')


def main():
    tilt = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    topic = sys.argv[2] if len(sys.argv) > 2 else '/gicp_map'
    complexity = int(sys.argv[3]) if len(sys.argv) > 3 else 3
    rclpy.init()
    node = SyntheticPublisher(tilt_deg=tilt, topic=topic, complexity=complexity)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
