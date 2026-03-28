#!/usr/bin/env python3
"""稳定的合成点云发布器：包含 30° 主坡和若干不同角度的小坡

发布话题：/cloud_registered
使用确定性生成，默认无噪声，便于测试和重复性验证
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class StableRampPublisher(Node):
    def __init__(self, topic='/cloud_registered', frame_id='map', rate_hz=1.0):
        super().__init__('stable_ramp_publisher')
        self.pub = self.create_publisher(PointCloud2, topic, 10)
        self.frame_id = frame_id
        self.rate = float(rate_hz)
        self.timer = self.create_timer(1.0 / self.rate, self.publish)

        # 点云参数
        self.plane_size = 8.0
        self.step = 0.06

        # 主坡角度（度）
        self.main_tilt_deg = 30.0

        # 在主坡上加入若干小坡（中心坐标, 角度deg_x, deg_y）
        self.small_ramps = [
            ((-1.5, -0.8), 12.0, 0.0),
            ((0.8, 0.5), -10.0, 0.0),
            ((2.0, -1.8), 20.0, 5.0),
            ((-2.2, 1.5), 15.0, -8.0),
        ]

        # 预生成稳定点云（后续发布只发送相同数据）
        self.cloud = self._build_stable_cloud()
        self.get_logger().info(f'Prepared stable cloud with {len(self.cloud)} points; publishing to {topic}')

    def _generate_plane(self, size, step):
        pts = []
        n = int(size / step)
        half = n / 2
        for i in range(n):
            x = (i - half) * step
            for j in range(n):
                y = (j - half) * step
                pts.append([x, y, 0.0])
        return pts

    def _add_sloped_patch(self, pts, center, size, step, angle_x_deg=0.0, angle_y_deg=0.0):
        cx, cy = center
        a = math.tan(math.radians(angle_x_deg))
        b = math.tan(math.radians(angle_y_deg))
        n = int(size / step)
        half = n / 2
        for i in range(n):
            for j in range(n):
                x = cx + (i - half) * step
                y = cy + (j - half) * step
                z = a * (x - cx) + b * (y - cy)
                pts.append([x, y, z])
        return pts

    def _apply_main_tilt(self, pts, tilt_deg):
        # 沿 x 方向按主坡角度提升 z
        t = math.tan(math.radians(tilt_deg))
        tilted = []
        for x, y, z in pts:
            z2 = z + t * x
            tilted.append((float(x), float(y), float(z2)))
        return tilted

    def _build_stable_cloud(self):
        pts = self._generate_plane(self.plane_size, self.step)

        # 在平面上添加若干局部斜坡（作为小坡的基础）
        for center, ang_x, ang_y in self.small_ramps:
            pts = self._add_sloped_patch(pts, center=center, size=1.2, step=0.04, angle_x_deg=ang_x, angle_y_deg=ang_y)

        # 对整个点云应用主坡倾角（30度）
        tilted = self._apply_main_tilt(pts, self.main_tilt_deg)
        return tilted

    def publish(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        cloud_msg = pc2.create_cloud_xyz32(header, self.cloud)
        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StableRampPublisher(topic='/cloud_registered', frame_id='map', rate_hz=1.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
