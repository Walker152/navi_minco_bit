#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def rodrigues_rotate(v, k, theta):
    # rotate vector v around axis k (unit) by theta (rad)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    dot = v[0] * k[0] + v[1] * k[1] + v[2] * k[2]
    cross = (
        k[1] * v[2] - k[2] * v[1],
        k[2] * v[0] - k[0] * v[2],
        k[0] * v[1] - k[1] * v[0],
    )
    return (
        v[0] * cos_t + cross[0] * sin_t + k[0] * dot * (1 - cos_t),
        v[1] * cos_t + cross[1] * sin_t + k[1] * dot * (1 - cos_t),
        v[2] * cos_t + cross[2] * sin_t + k[2] * dot * (1 - cos_t),
    )


class CloudPub(Node):
    def __init__(self):
        super().__init__('cloud_registered_pub')
        # use reliable + transient_local so viewers receive a stable latched copy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PointCloud2, 'cloud_registered', qos)
        # faster publish for smooth visualization
        self.timer = self.create_timer(0.2, self.timer_cb)

        # configuration
        self.big_slope_deg = 30.0
        self.area_x = (-8.0, 8.0)
        self.area_y = (-4.0, 4.0)
        # finer base grid for higher density
        self.res = 0.08
        # small patches on slope (centers)
        self.patch_centers = [(-3.0, -1.5), (-1.5, 1.2), (0.5, -0.5), (2.5, 0.7), (4.0, -1.0)]
        # relative angles in degrees between 0 and 60
        self.patch_rel_angles = [0.0, 12.0, 28.0, 44.0, 60.0]
        self.patch_size = 1.2
        # very fine patch resolution
        self.patch_res = 0.03
        # add small random clusters and vertical poles to increase complexity
        self.rock_clusters = [(-2.5, 0.5), (1.0, -1.2), (3.5, 1.5)]
        self.rock_points = 200
        self.pole_positions = [(-4.0, -2.0), (0.0, 2.5), (5.0, -0.5)]
        self.pole_height = 2.0
        # measurement noise sigma
        self.noise_sigma = 0.01

        # pre-generate cloud once
        self.cloud_msg = self.generate_cloud()

    def big_plane_z(self, x):
        return x * math.tan(math.radians(self.big_slope_deg))

    def generate_cloud(self):
        points = []

        # big plane (tilted around Y axis): z = x * tan(alpha)
        x0, x1 = self.area_x
        y0, y1 = self.area_y
        x = x0
        while x <= x1:
            y = y0
            while y <= y1:
                z = self.big_plane_z(x)
                # add small surface perturbation to make map less uniform
                zb = z + 0.01 * math.sin(0.5 * x) * math.cos(0.3 * y)
                points.append((x, y, zb))
                y += self.res
            x += self.res

        # big plane normal (not normalized): (-tan(alpha), 0, 1)
        tan_a = math.tan(math.radians(self.big_slope_deg))
        n_big = (-tan_a, 0.0, 1.0)
        # along-slope direction (in-plane): (1,0,tan_a)
        v_slope = (1.0, 0.0, tan_a)
        # normalize v_slope
        vs_len = math.sqrt(v_slope[0] ** 2 + v_slope[1] ** 2 + v_slope[2] ** 2)
        v_slope = (v_slope[0] / vs_len, v_slope[1] / vs_len, v_slope[2] / vs_len)

        # add small tilted patches on top of big plane
        for (center_x, center_y), rel_deg in zip(self.patch_centers, self.patch_rel_angles):
            rel_rad = math.radians(rel_deg)
            # rotate big normal around v_slope by rel_rad to get local normal
            # first normalize n_big
            nlen = math.sqrt(n_big[0] ** 2 + n_big[1] ** 2 + n_big[2] ** 2)
            n_big_u = (n_big[0] / nlen, n_big[1] / nlen, n_big[2] / nlen)
            n_local = rodrigues_rotate(n_big_u, v_slope, rel_rad)

            # compute center z on big plane
            center_z = self.big_plane_z(center_x)
            # for patch points, compute z by plane equation: n·(p - p0) = 0 => nz*(z - z0) = -nx*(x-x0)-ny*(y-y0)
            nx, ny, nz = n_local
            if abs(nz) < 1e-6:
                continue

            px = center_x - self.patch_size / 2.0
            while px <= center_x + self.patch_size / 2.0:
                py = center_y - self.patch_size / 2.0
                while py <= center_y + self.patch_size / 2.0:
                    dz = -(nx * (px - center_x) + ny * (py - center_y)) / nz
                    pz = center_z + dz
                    # small random micro-roughness
                    pz += random.uniform(-0.01, 0.01)
                    points.append((px, py, pz))
                    py += self.patch_res
                px += self.patch_res

        # add rock clusters (Gaussian blobs)
        for (cx, cy) in self.rock_clusters:
            for i in range(self.rock_points):
                rx = random.gauss(cx, 0.25)
                ry = random.gauss(cy, 0.25)
                # height slightly above plane
                rz = self.big_plane_z(rx) + abs(random.gauss(0.15, 0.05))
                points.append((rx, ry, rz))

        # add vertical poles
        for (pxc, pyc) in self.pole_positions:
            n_steps = int(self.pole_height / 0.05)
            for k in range(n_steps):
                z = k * 0.05 + 0.02
                # small ring around pole center
                for ang in range(0, 360, 45):
                    rad = math.radians(ang)
                    r = 0.02
                    points.append((pxc + r * math.cos(rad), pyc + r * math.sin(rad), z))

        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()
        # add gaussian noise to all points (measurement noise)
        noisy = []
        for (x, y, z) in points:
            nx = x + random.gauss(0.0, self.noise_sigma)
            ny = y + random.gauss(0.0, self.noise_sigma)
            nz = z + random.gauss(0.0, self.noise_sigma)
            noisy.append((nx, ny, nz))

        cloud = point_cloud2.create_cloud_xyz32(header, noisy)
        cloud.is_dense = True
        return cloud

    def timer_cb(self):
        # refresh header timestamp
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CloudPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
