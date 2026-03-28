#!/usr/bin/env python3
"""Sequential ramp pointcloud publisher.

Publishes a flat plane and overlays ramps sequentially on `/gicp_map`.
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SequentialRampPublisher(Node):
    def __init__(self):
        super().__init__('ramp_pointcloud_publisher')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('stage_duration', 5.0)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.stage_duration = float(self.get_parameter('stage_duration').get_parameter_value().double_value)

        self.pub = self.create_publisher(PointCloud2, '/gicp_map', 10)

        # configuration for plane + multiple ramps
        self.ramp_angles = [10, 20, 30, 40, 50, 60]
        self.ramp_length = 4.0
        self.ramp_width = 1.5
        self.ramp_spacing = 2.5
        # plane size
        self.plane_length = 10.0
        self.plane_width = max(6.0, self.ramp_spacing * (len(self.ramp_angles) + 1))

        self.publish_count = 0
        self.get_logger().info(f'Publisher started: rate={self.publish_rate}Hz, plane {self.plane_length}x{self.plane_width}, ramps={self.ramp_angles}')
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1e-3), self._publish_plane_with_ramps)

    def _generate_plane_points(self, length=6.0, width=4.0, nx=60, ny=40):
        xs = np.linspace(-length / 2.0, length / 2.0, nx)
        ys = np.linspace(-width / 2.0, width / 2.0, ny)
        pts = []
        for x in xs:
            for y in ys:
                pts.append([float(x), float(y), 0.0])
        return pts

    def _generate_ramp(self, angle_deg, length=4.0, width=1.5, nx=80, ny=20, xc=0.0, yc=0.0):
        xs = np.linspace(xc - length / 2.0, xc + length / 2.0, nx)
        ys = np.linspace(yc - width / 2.0, yc + width / 2.0, ny)
        a = math.radians(angle_deg)
        pts = []
        for x in xs:
            for y in ys:
                z = math.tan(a) * (x - (xc - length / 2.0))
                if z < 0.0:
                    z = 0.0
                pts.append([float(x), float(y), float(z)])
        return pts

    def _publish_plane_with_ramps(self):
        now = time.time()
        stage = int((now // self.stage_duration)) % (len(self.ramp_angles) + 1)
        points = []
        points.extend(self._generate_plane_points(self.plane_length, self.plane_width, nx=60, ny=40))
        if stage > 0:
            angle = self.ramp_angles[stage - 1]
            yc = -self.plane_width / 2.0 + stage * self.ramp_spacing
            ramp_pts = self._generate_ramp(angle, length=self.ramp_length, width=self.ramp_width, nx=80, ny=24, xc=0.0, yc=yc)
            points.extend(ramp_pts)

        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        cloud = point_cloud2.create_cloud_xyz32(header, [(p[0], p[1], p[2]) for p in points])
        self.pub.publish(cloud)
        self.publish_count += 1
        if self.publish_count % 5 == 0:
            self.get_logger().info(f'Published {self.publish_count} clouds (last stage={stage})')


def main(args=None):
    rclpy.init(args=args)
    node = SequentialRampPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
