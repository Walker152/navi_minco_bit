#!/usr/bin/env python3
"""
发布一个模拟雷达倾斜安装的点云：
- 世界坐标系中，地面为水平面（z=0），并包含多个不同角度的斜坡。
- 然后应用绕X轴的旋转变换（角度可配置），模拟雷达倾斜安装。
- 最终发布到 `/gicp_map` 话题，用于测试倾斜补偿算法。
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class TiltedRampPublisher(Node):
    def __init__(self):
        super().__init__('tilted_ramp_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('tilt_angle_x', 30.0)
        self.declare_parameter('ramp_angles', [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
        self.declare_parameter('ramp_length', 4.0)
        self.declare_parameter('ramp_width', 1.5)
        self.declare_parameter('ramp_spacing', 2.5)
        self.declare_parameter('plane_length', 10.0)
        self.declare_parameter('plane_width', 6.0)

        p = self.get_parameter
        self.frame_id = p('frame_id').get_parameter_value().string_value
        self.publish_rate = p('publish_rate').get_parameter_value().double_value
        self.tilt_angle_x = p('tilt_angle_x').get_parameter_value().double_value
        # handle ramp_angles as list of doubles
        try:
            self.ramp_angles = p('ramp_angles').get_parameter_value().double_array_value
        except Exception:
            self.ramp_angles = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
        self.ramp_length = p('ramp_length').get_parameter_value().double_value
        self.ramp_width = p('ramp_width').get_parameter_value().double_value
        self.ramp_spacing = p('ramp_spacing').get_parameter_value().double_value
        self.plane_length = p('plane_length').get_parameter_value().double_value
        self.plane_width = p('plane_width').get_parameter_value().double_value

        self.pub = self.create_publisher(PointCloud2, '/gicp_map', 10)

        rad = math.radians(self.tilt_angle_x)
        self.R = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(rad), -math.sin(rad)],
            [0.0, math.sin(rad), math.cos(rad)],
        ])

        self.get_logger().info(f'Tilted Ramp Publisher started: tilt X={self.tilt_angle_x}°, ramps={self.ramp_angles}')
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._publish)

    def _generate_world_points(self):
        xs = np.linspace(-self.plane_length / 2.0, self.plane_length / 2.0, 240)
        ys = np.linspace(-self.plane_width / 2.0, self.plane_width / 2.0, 120)
        X, Y = np.meshgrid(xs, ys, indexing='xy')
        Z = np.zeros_like(X)

        center_start = -(len(self.ramp_angles) - 1) * self.ramp_spacing / 2.0
        for idx, angle in enumerate(self.ramp_angles):
            y_center = center_start + idx * self.ramp_spacing
            for iy, y in enumerate(ys):
                if abs(y - y_center) <= self.ramp_width / 2.0:
                    for ix, x in enumerate(xs):
                        if abs(x) <= self.ramp_length / 2.0:
                            z_ramp = math.tan(math.radians(angle)) * x
                            if z_ramp < 0:
                                z_ramp = 0.0
                            Z[iy, ix] = z_ramp

        points = []
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                points.append((float(x), float(y), float(Z[iy, ix])))
        return points

    def _apply_rotation(self, points):
        rotated = []
        for p in points:
            pv = np.array(p)
            pr = self.R.dot(pv)
            rotated.append((float(pr[0]), float(pr[1]), float(pr[2])))
        return rotated

    def _publish(self):
        world_points = self._generate_world_points()
        radar_points = self._apply_rotation(world_points)

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(header, radar_points)
        self.pub.publish(cloud)
        self.get_logger().info(f'Published {len(radar_points)} points (tilted)')


def main(args=None):
    rclpy.init(args=args)
    node = TiltedRampPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
