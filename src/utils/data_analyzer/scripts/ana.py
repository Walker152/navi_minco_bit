#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nav2 Performance Analyzer for Ubuntu 22.04 (ROS2 Humble)
--------------------------------------------------------
基于 PySide6 (Qt) 和 ROS2 实现的导航性能实时分析工具。
功能：
1. 实时监听 TF (map->base_link) 获取机器人位置。
2. 监听 /plan 获取全局路径，计算路径跟随误差 (Cross Track Error)。
3. 监听 /amcl_pose (或等效定位话题) 分析定位协方差/不确定度。
4. 实时动态曲线绘制与数据统计。

Author: Gemini
Updates: Removed scipy dependency to fix numpy version conflicts.
"""

import sys
import threading
import time
import numpy as np
# Removed scipy dependency to avoid ABI conflict errors
# from scipy.spatial import KDTree 

# UI Framework
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QPushButton, QLabel, QGroupBox, 
                               QSplitter, QFrame)
from PySide6.QtCore import QTimer, Signal, Slot, QObject, Qt
from PySide6.QtGui import QFont, QColor

# Plotting
import pyqtgraph as pg

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# 配置区
TOPIC_GLOBAL_PLAN = '/plan'
TOPIC_LOCALIZATION = '/amcl_pose' 
FRAME_MAP = 'map'
FRAME_ROBOT = 'base_link'

class ROSWorker(QObject):
    """
    ROS2 工作线程，用于处理所有 ROS 通讯，防止阻塞 GUI
    """
    data_updated = Signal(dict) # 信号：发送最新计算的数据给 GUI

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.running = True
        self.paused = False
        
        # 数据缓存
        self.current_path_points = None # 存储路径点 (Nx2 数组)
        self.latest_cov_trace = 0.0     # 定位协方差迹（不确定度）
        self.path_deviation = 0.0       # 路径偏差

    def start_ros(self):
        rclpy.init(args=None)
        self.node = Node('nav2_performance_analyzer')
        
        # 1. TF Listener (获取机器人真实位置)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # 2. Path Subscriber (全局规划)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.node.create_subscription(Path, TOPIC_GLOBAL_PLAN, self.path_callback, qos_profile)

        # 3. Localization Subscriber (定位不确定度)
        self.node.create_subscription(PoseWithCovarianceStamped, TOPIC_LOCALIZATION, self.pose_cov_callback, qos_profile)

        # 定时器用于周期性计算和发布数据 (20Hz)
        self.timer = self.node.create_timer(0.05, self.update_loop)

        # Spin in a separate thread context (handled by rclpy.spin)
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS Spin Error: {e}")
        finally:
            self.shutdown()

    def path_callback(self, msg):
        """处理接收到的全局路径，存储为 numpy 数组"""
        if not msg.poses:
            return
        
        # 提取路径点 (x, y) 并直接存储为 numpy 数组
        # 相比 KDTree，直接矩阵运算对于路径这种规模的数据（通常几百个点）足够快且无额外依赖
        self.current_path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def pose_cov_callback(self, msg):
        """处理定位消息，计算协方差矩阵的迹作为不确定度指标"""
        # Covariance 是一个 6x6 矩阵展开的 36 浮点数数组
        # 索引 0是xx, 7是yy, 35是theta_theta
        # 我们主要关注 XY 平面的定位置信度
        cov = msg.pose.covariance
        self.latest_cov_trace = cov[0] + cov[7]  # Var(X) + Var(Y)

    def update_loop(self):
        if self.paused:
            return

        try:
            # 获取 map -> base_link 的变换
            now = rclpy.time.Time()
            # 注意：在 ROS2 humble 中 lookup_transform 可能抛出异常，需做好捕获
            if not self.tf_buffer.can_transform(FRAME_MAP, FRAME_ROBOT, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)):
                 return

            trans = self.tf_buffer.lookup_transform(
                FRAME_MAP, 
                FRAME_ROBOT, 
                rclpy.time.Time()) # Get latest available

            rx = trans.transform.translation.x
            ry = trans.transform.translation.y

            # 计算路径偏差 (Cross Track Error)
            deviation = 0.0
            if self.current_path_points is not None and len(self.current_path_points) > 0:
                # 使用 numpy 广播机制计算当前点到所有路径点的距离
                # 形状: (N, 2) - (1, 2) -> (N, 2)
                diff = self.current_path_points - np.array([rx, ry])
                # 计算欧氏距离平方
                dists_sq = np.sum(diff**2, axis=1)
                # 取最小距离的平方根
                deviation = np.sqrt(np.min(dists_sq))
            
            # 发送数据到 GUI
            data = {
                'timestamp': time.time(),
                'deviation': deviation,
                'uncertainty': self.latest_cov_trace,
                'robot_x': rx,
                'robot_y': ry
            }
            self.data_updated.emit(data)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # TF 还没准备好，忽略
            pass
        except Exception as e:
            print(f"Update Loop Error: {e}")

    def shutdown(self):
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def set_paused(self, paused):
        self.paused = paused

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Nav2 Performance Analyzer")
        self.resize(1000, 700)
        
        # 样式设置 (Dark Theme)
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; color: #ffffff; }
            QLabel { color: #e0e0e0; font-size: 14px; }
            QGroupBox { border: 1px solid #555; margin-top: 10px; font-weight: bold; color: #aaa; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 5px; }
            QPushButton { background-color: #444; border: 1px solid #666; padding: 5px; color: white; border-radius: 4px; }
            QPushButton:hover { background-color: #555; }
            QPushButton:pressed { background-color: #333; }
        """)

        # Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- 顶部控制栏 ---
        top_bar = QHBoxLayout()
        
        self.btn_pause = QPushButton("⏸ Pause Analysis")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self.toggle_pause)
        
        self.btn_clear = QPushButton("🗑 Clear History")
        self.btn_clear.clicked.connect(self.clear_data)

        self.lbl_status = QLabel("Status: Waiting for ROS2...")
        self.lbl_status.setStyleSheet("color: #ffaa00;")

        top_bar.addWidget(self.btn_pause)
        top_bar.addWidget(self.btn_clear)
        top_bar.addStretch()
        top_bar.addWidget(self.lbl_status)
        main_layout.addLayout(top_bar)

        # --- 数据显示区 (数字) ---
        stats_layout = QHBoxLayout()
        self.lbl_dev_val = self.create_stat_card(stats_layout, "Path Deviation (m)", "0.000")
        self.lbl_unc_val = self.create_stat_card(stats_layout, "Loc Uncertainty (Cov)", "0.000")
        main_layout.addLayout(stats_layout)

        # --- 图表区 (PyQtGraph) ---
        splitter = QSplitter(Qt.Vertical)
        
        # 1. 路径偏差图
        self.plot_dev = pg.PlotWidget(title="Path Deviation (m)")
        self.plot_dev.showGrid(x=True, y=True)
        self.plot_dev.setLabel('left', 'Error (m)')
        self.plot_dev.setLabel('bottom', 'Time (s)')
        self.curve_dev = self.plot_dev.plot(pen=pg.mkPen('c', width=2)) # Cyan
        splitter.addWidget(self.plot_dev)

        # 2. 定位不确定度图
        self.plot_unc = pg.PlotWidget(title="Localization Uncertainty (Variance Trace)")
        self.plot_unc.showGrid(x=True, y=True)
        self.plot_unc.setLabel('left', 'Covariance')
        self.plot_unc.setLabel('bottom', 'Time (s)')
        self.curve_unc = self.plot_unc.plot(pen=pg.mkPen('m', width=2)) # Magenta
        splitter.addWidget(self.plot_unc)

        main_layout.addWidget(splitter)

        # --- 数据存储 ---
        self.time_data = []
        self.dev_data = []
        self.unc_data = []
        self.start_time = time.time()
        self.max_points = 500 # 窗口显示的最大点数

        # --- ROS 线程启动 ---
        self.ros_thread = threading.Thread(target=self.start_ros_worker, daemon=True)
        self.ros_worker = ROSWorker()
        self.ros_worker.data_updated.connect(self.update_gui)
        self.ros_thread.start()

    def create_stat_card(self, layout, title, default_val):
        group = QGroupBox(title)
        vbox = QVBoxLayout()
        val_label = QLabel(default_val)
        val_label.setAlignment(Qt.AlignCenter)
        val_label.setFont(QFont("Arial", 24, QFont.Bold))
        vbox.addWidget(val_label)
        group.setLayout(vbox)
        layout.addWidget(group)
        return val_label

    def start_ros_worker(self):
        self.ros_worker.start_ros()

    @Slot(dict)
    def update_gui(self, data):
        """接收 ROS 线程的数据并更新 UI"""
        t = data['timestamp'] - self.start_time
        dev = data['deviation']
        unc = data['uncertainty']

        # Update Labels
        self.lbl_dev_val.setText(f"{dev:.4f}")
        self.lbl_unc_val.setText(f"{unc:.4f}")
        self.lbl_status.setText(f"Status: Running | Robot: ({data['robot_x']:.2f}, {data['robot_y']:.2f})")
        self.lbl_status.setStyleSheet("color: #00ff00;")

        # Update Lists
        self.time_data.append(t)
        self.dev_data.append(dev)
        self.unc_data.append(unc)

        # Keep buffer size fixed
        if len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.dev_data.pop(0)
            self.unc_data.pop(0)

        # Update Plots
        self.curve_dev.setData(self.time_data, self.dev_data)
        self.curve_unc.setData(self.time_data, self.unc_data)

    def toggle_pause(self):
        is_paused = self.btn_pause.isChecked()
        self.ros_worker.set_paused(is_paused)
        if is_paused:
            self.btn_pause.setText("▶ Resume Analysis")
            self.lbl_status.setText("Status: Paused")
            self.lbl_status.setStyleSheet("color: #ffff00;")
        else:
            self.btn_pause.setText("⏸ Pause Analysis")

    def clear_data(self):
        self.time_data = []
        self.dev_data = []
        self.unc_data = []
        self.start_time = time.time() # Reset time zero
        self.curve_dev.setData([], [])
        self.curve_unc.setData([], [])

    def closeEvent(self, event):
        self.ros_worker.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())