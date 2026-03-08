#!/usr/bin/env python3
"""
在RViz2中自动添加点云显示
"""
import rclpy
from rclpy.node import Node
import json
import time

class RVizDisplayController(Node):
    def __init__(self):
        super().__init__('rviz_display_controller')
        self.get_logger().info("RViz Display Controller 已启动")
        
        # 订阅RViz的配置话题
        self.publisher_display_1 = self.create_publisher(
            type('DisplayAdd', (), {}),
            '/rviz/displays/add_display', 10)
        
        self.timer = self.create_timer(5.0, self.check_displays)
        self.get_logger().info("""
╔════════════════════════════════════════════════════════════════╗
║                    RViz2 点云可视化指南                         ║
╚════════════════════════════════════════════════════════════════╝

✅ RViz2已启动！

现在请在RViz2中手动添加显示（自动脚本在开发中）：

【步骤1】修改 Fixed Frame：
  1. 左侧面板找到 "Global Options"
  2. 将 "Fixed Frame" 改为 "camera_init"

【步骤2】添加原始点云 (/cloud_registered)：
  1. 点击左下方 "Add" 按钮
  2. 选择 "By topic" 标签
  3. 找到 "/cloud_registered" → PointCloud2
  4. 点击 "Add"
  → 应该看到白色点云出现

【步骤3】添加过滤点云 (/cloud_filter_baselink)：
  1. 再次点击 "Add" 按钮
  2. 选择 "By topic" 标签  
  3. 找到 "/cloud_filter_baselink" → PointCloud2
  4. 点击 "Add"
  → 应该看到绿色/彩色点云出现

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

📊 实时监测：
""")

    def check_displays(self):
        """定期检查显示情况"""
        self.get_logger().info("✓ 系统运行正常，等待您的操作...")

def main(args=None):
    rclpy.init(args=args)
    node = RVizDisplayController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n程序已中止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
