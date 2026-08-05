#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# 注意：如果你的驱动输出的是 CustomMsg，请修改为：
# from livox_interfaces.msg import CustomMsg as PointCloudType
from sensor_msgs.msg import PointCloud2 as PointCloudType

class PtpSyncChecker(Node):
    def __init__(self):
        super().__init__('ptp_sync_checker')

        # ⚠️ 请将这里替换为你实际 launch 文件中的双雷达 topic 名称
        self.topic_left = '/livox/lidar_192_168_1_122'
        self.topic_right = '/livox/lidar_192_168_1_135'

        self.sub_left = self.create_subscription(PointCloudType, self.topic_left, self.left_cb, 10)
        self.sub_right = self.create_subscription(PointCloudType, self.topic_right, self.right_cb, 10)

        self.left_time = 0.0
        self.right_time = 0.0

        # 每 1 秒打印一次状态报告
        self.timer = self.create_timer(1.0, self.status_monitor)
        self.get_logger().info("🔍 PTP 双雷达时间戳同步校验节点已启动...")

    def left_cb(self, msg):
        # 提取 Header 中的时间戳并转换为秒
        self.left_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def right_cb(self, msg):
        self.right_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def status_monitor(self):
        # 获取当前工控机的 ROS 系统时间
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.left_time == 0 or self.right_time == 0:
            self.get_logger().warn("等待接收双雷达点云数据...")
            return

        # 计算传输与处理延迟 (当前时间 - 报文打戳时间)
        delay_left = (now - self.left_time) * 1000   # 转换为毫秒 ms
        delay_right = (now - self.right_time) * 1000 

        # 核心指标：两台雷达发出的点云，时间戳相差多少？
        diff_between_lidars = abs(self.left_time - self.right_time) * 1000 

        self.get_logger().info(
            f"\n"
            f"⏱️ [工控机系统时间] {now:.3f}\n"
            f"├─ 雷达 1 传输延迟: {delay_left:7.2f} ms\n"
            f"├─ 雷达 2 传输延迟: {delay_right:7.2f} ms\n"
            f"└─ 🔴 双雷达时间对齐误差: {diff_between_lidars:7.3f} ms"
        )

def main():
    rclpy.init()
    node = PtpSyncChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("检测到退出信号，关闭节点。")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()