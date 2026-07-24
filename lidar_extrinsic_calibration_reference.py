#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达外参标定工具

用途：
    对安装在旋转云台上的激光雷达进行外参标定，计算：
    1. 旋转半径 (Radius): 雷达中心到云台旋转轴的水平距离
    2. 安装倾角 (Pitch Bias): 雷达相对于水平面的俯仰角

原理：
    - 底盘静止，云台带动雷达旋转时，雷达在 XY 平面上画圆
    - 通过最小二乘法拟合圆，得到旋转半径
    - 通过分析姿态四元数，得到平均 Pitch 角作为安装倾角

依赖：
    pip install rosbags numpy scipy matplotlib

作者: Jinbo Liu
日期: 2026-01-24
"""

import argparse
import sys
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# ROS 2 Bag 读取
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


@dataclass
class OdometryData:
    """里程计数据容器"""
    timestamps: List[float] = field(default_factory=list)
    positions_x: List[float] = field(default_factory=list)
    positions_y: List[float] = field(default_factory=list)
    positions_z: List[float] = field(default_factory=list)
    orientations: List[Tuple[float, float, float, float]] = field(default_factory=list)  # (x, y, z, w)

    def to_numpy(self):
        """转换为 NumPy 数组"""
        return {
            'timestamps': np.array(self.timestamps),
            'x': np.array(self.positions_x),
            'y': np.array(self.positions_y),
            'z': np.array(self.positions_z),
            'orientations': np.array(self.orientations)
        }


class LidarExtrinsicCalibrator:
    """激光雷达外参标定器"""

    def __init__(self, bag_path: str, topic_name: str = "/aft_mapped_to_init"):
        """
        初始化标定器

        Args:
            bag_path: ROS 2 bag 文件路径
            topic_name: 里程计话题名称
        """
        self.bag_path = Path(bag_path)
        self.topic_name = topic_name
        self.odom_data = OdometryData()
        self.typestore = get_typestore(Stores.ROS2_HUMBLE)

        # 标定结果
        self.circle_center: Optional[Tuple[float, float]] = None
        self.radius: Optional[float] = None
        self.dx: Optional[float] = None
        self.dy: Optional[float] = None
        self.pitch_mean: Optional[float] = None
        self.pitch_std: Optional[float] = None
        self.roll_mean: Optional[float] = None
        self.rmse: Optional[float] = None

    def read_bag(self) -> bool:
        """
        读取 ROS 2 bag 文件中的里程计数据

        Returns:
            是否成功读取数据
        """
        if not self.bag_path.exists():
            print(f"[错误] Bag 文件不存在: {self.bag_path}")
            return False

        print(f"[信息] 正在读取 Bag 文件: {self.bag_path}")

        try:
            with Reader(self.bag_path) as reader:
                # 检查话题是否存在
                topic_found = False
                for connection in reader.connections:
                    if connection.topic == self.topic_name:
                        topic_found = True
                        print(f"[信息] 找到话题: {self.topic_name}, 消息类型: {connection.msgtype}")
                        break

                if not topic_found:
                    print(f"[错误] 未找到话题: {self.topic_name}")
                    print("[信息] 可用话题列表:")
                    for conn in reader.connections:
                        print(f"       - {conn.topic} ({conn.msgtype})")
                    return False

                # 读取消息
                msg_count = 0
                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == self.topic_name:
                        msg = self.typestore.deserialize_cdr(rawdata, connection.msgtype)

                        # 提取位置
                        self.odom_data.timestamps.append(timestamp * 1e-9)  # 纳秒转秒
                        self.odom_data.positions_x.append(msg.pose.pose.position.x)
                        self.odom_data.positions_y.append(msg.pose.pose.position.y)
                        self.odom_data.positions_z.append(msg.pose.pose.position.z)

                        # 提取姿态四元数 (x, y, z, w)
                        quat = (
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w
                        )
                        self.odom_data.orientations.append(quat)
                        msg_count += 1

                print(f"[信息] 成功读取 {msg_count} 条里程计消息")
                return msg_count > 0

        except Exception as e:
            print(f"[错误] 读取 Bag 文件失败: {e}")
            return False

    @staticmethod
    def circle_residuals(params: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        圆拟合的残差函数

        Args:
            params: [x_c, y_c, R] 圆心坐标和半径
            x, y: 数据点坐标

        Returns:
            残差数组
        """
        x_c, y_c, R = params
        return np.sqrt((x - x_c)**2 + (y - y_c)**2) - R

    def fit_circle(self, x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float, float]:
        """
        使用最小二乘法拟合圆

        Args:
            x, y: 数据点坐标数组

        Returns:
            (x_c, y_c, R, rmse): 圆心坐标、半径、均方根误差
        """
        # 初始估计：圆心为数据点的均值，半径为到均值的平均距离
        x_mean, y_mean = np.mean(x), np.mean(y)
        R_init = np.mean(np.sqrt((x - x_mean)**2 + (y - y_mean)**2))
        initial_params = [x_mean, y_mean, R_init]

        # 最小二乘优化
        result = least_squares(
            self.circle_residuals,
            initial_params,
            args=(x, y),
            method='lm'  # Levenberg-Marquardt 算法
        )

        x_c, y_c, R = result.x

        # 计算 RMSE
        residuals = self.circle_residuals(result.x, x, y)
        rmse = np.sqrt(np.mean(residuals**2))

        return x_c, y_c, abs(R), rmse

    def compute_euler_angles(self, orientations: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        将四元数转换为欧拉角 (Roll, Pitch, Yaw)

        Args:
            orientations: 四元数数组 (N, 4)，格式为 (x, y, z, w)

        Returns:
            (roll, pitch, yaw): 欧拉角数组，单位为弧度
        """
        # scipy 的 Rotation 使用 (x, y, z, w) 格式
        rotations = Rotation.from_quat(orientations)

        # 转换为欧拉角，使用 'xyz' 内旋顺序（即 Roll-Pitch-Yaw）
        euler_angles = rotations.as_euler('xyz', degrees=False)

        roll = euler_angles[:, 0]
        pitch = euler_angles[:, 1]
        yaw = euler_angles[:, 2]

        return roll, pitch, yaw

    def calibrate(self) -> bool:
        """
        执行标定

        Returns:
            是否标定成功
        """
        data = self.odom_data.to_numpy()

        if len(data['x']) < 10:
            print("[错误] 数据点太少，无法进行标定（至少需要 10 个点）")
            return False

        print("\n" + "=" * 60)
        print("开始标定...")
        print("=" * 60)

        # ==================== 1. 圆拟合 ====================
        print("\n[步骤 1] 圆拟合 (XY 平面)")
        x_c, y_c, R, rmse = self.fit_circle(data['x'], data['y'])

        self.circle_center = (x_c, y_c)
        self.radius = R
        self.rmse = rmse

        print(f"  - 圆心坐标: ({x_c:.6f}, {y_c:.6f}) [米]")
        print(f"  - 拟合半径: {R:.6f} [米] = {R * 1000:.3f} [毫米]")
        print(f"  - 拟合 RMSE: {rmse:.6f} [米] = {rmse * 1000:.3f} [毫米]")

        # ==================== 1.5 & 2. 倾角分析与局部 XY 偏移量 ====================
        print("\n[步骤 1.5 & 2] 分析欧拉角并计算纯水平 X, Y 偏移量 (dx, dy)")

        # 1. 提前转换为欧拉角，提取核心的 yaw 
        roll, pitch, yaw = self.compute_euler_angles(data['orientations'])

        # 2. 构造所有的圆向向量
        dx_world = data['x'] - x_c
        dy_world = data['y'] - y_c

        # 3. 纯二维逆向旋转 (去 Yaw) 
        # [dx_local] = [ cos(yaw)   sin(yaw)] * [dx_world]
        # [dy_local]   [-sin(yaw)   cos(yaw)]   [dy_world]
        dx_local = dx_world * np.cos(yaw) + dy_world * np.sin(yaw)
        dy_local = -dx_world * np.sin(yaw) + dy_world * np.cos(yaw)

        # 4. 求均值
        self.dx = float(np.mean(dx_local))
        self.dy = float(np.mean(dy_local))
        dx_std = float(np.std(dx_local))
        dy_std = float(np.std(dy_local))

        self.pitch_mean = np.mean(pitch)
        self.pitch_std = np.std(pitch)
        self.roll_mean = np.mean(roll)

        print(f"  - 雷达局部 X 偏移 (dx): {self.dx:.6f} m")
        print(f"  - 雷达局部 Y 偏移 (dy): {self.dy:.6f} m")
        print(f"  - 稳定性检验: std_x={dx_std*1000:.2f}mm, std_y={dy_std*1000:.2f}mm")

        calculated_R = np.hypot(self.dx, self.dy)
        print(f"  - 校验吻合度: 二维模长 {calculated_R:.6f}m vs 拟合半径 {R:.6f}m")

        print(f"  - 平均 Pitch: {self.pitch_mean:.6f} [弧度] = {np.degrees(self.pitch_mean):.4f} [度]")
        print(f"  - Pitch 标准差: {self.pitch_std:.6f} [弧度] = {np.degrees(self.pitch_std):.4f} [度]")
        print(f"  - 平均 Roll: {self.roll_mean:.6f} [弧度] = {np.degrees(self.roll_mean):.4f} [度]")

        # ==================== 3. 输出总结 ====================
        print("\n" + "=" * 60)
        print("标定结果总结")
        print("=" * 60)
        print(f"\n  【雷达偏心安装平移 (XY Offset)】")
        print(f"     dx = {self.dx:.6f} m = {self.dx * 1000:.3f} mm")
        print(f"     dy = {self.dy:.6f} m = {self.dy * 1000:.3f} mm")
        print(f"     校验模长 = {np.hypot(self.dx, self.dy) * 1000:.3f} mm vs 拟合半径 = {self.radius * 1000:.3f} mm")
        print(f"     (用于填入 t_lidar_joint 的 x, y 分量)")

        print(f"\n  【安装倾角 (Pitch Bias)】")
        print(f"     Pitch = {self.pitch_mean:.6f} rad = {np.degrees(self.pitch_mean):.4f} deg")
        print(f"     (用于填入 angle_y 参数)")

        print(f"\n  【拟合质量】")
        print(f"     RMSE = {self.rmse * 1000:.3f} mm")
        if self.rmse < 0.005:
            print("     ✓ 拟合质量优秀 (RMSE < 5mm)")
        elif self.rmse < 0.01:
            print("     ○ 拟合质量良好 (RMSE < 10mm)")
        else:
            print("     △ 拟合质量一般，建议检查数据")

        print("\n" + "=" * 60)

        return True

    def visualize(self, save_path: Optional[str] = None):
        """
        可视化标定结果

        Args:
            save_path: 保存图片的路径，None 则显示
        """
        if self.radius is None:
            print("[错误] 请先执行 calibrate() 进行标定")
            return

        data = self.odom_data.to_numpy()
        roll, pitch, yaw = self.compute_euler_angles(data['orientations'])

        # 创建图形
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('LiDAR Extrinsic Calibration Result', fontsize=14, fontweight='bold')

        # ==================== 图 1: XY 平面圆拟合 ====================
        ax1 = axes[0]

        # 绘制原始轨迹散点
        scatter = ax1.scatter(data['x'], data['y'], c=data['timestamps'] - data['timestamps'][0],
                              cmap='viridis', s=10, alpha=0.7, label='Raw Trajectory')
        plt.colorbar(scatter, ax=ax1, label='Time [s]')

        # 绘制拟合圆
        theta = np.linspace(0, 2 * np.pi, 100)
        x_circle = self.circle_center[0] + self.radius * np.cos(theta)
        y_circle = self.circle_center[1] + self.radius * np.sin(theta)
        ax1.plot(x_circle, y_circle, 'r-', linewidth=2, label=f'Fitted Circle (R={self.radius*1000:.2f}mm)')

        # 绘制圆心
        ax1.plot(self.circle_center[0], self.circle_center[1], 'r+', markersize=15, 
                 markeredgewidth=2, label='Center')

        # 绘制半径线
        ax1.plot([self.circle_center[0], data['x'][0]], 
                 [self.circle_center[1], data['y'][0]], 
                 'r--', linewidth=1, alpha=0.7)

        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.set_title(f'XY Trajectory & Circle Fitting\nRadius R = {self.radius*1000:.3f} mm, RMSE = {self.rmse*1000:.3f} mm')
        ax1.legend(loc='upper right')
        ax1.axis('equal')
        ax1.grid(True, alpha=0.3)

        # ==================== 图 2: 姿态角变化 ====================
        ax2 = axes[1]

        time_rel = data['timestamps'] - data['timestamps'][0]

        # 绘制 Roll 和 Pitch 曲线
        ax2.plot(time_rel, np.degrees(roll), 'b-', linewidth=1, alpha=0.7, label='Roll')
        ax2.plot(time_rel, np.degrees(pitch), 'g-', linewidth=1, alpha=0.7, label='Pitch')

        # 绘制平均 Pitch 线
        ax2.axhline(y=np.degrees(self.pitch_mean), color='r', linestyle='--', linewidth=2,
                    label=f'Mean Pitch = {np.degrees(self.pitch_mean):.4f}°')

        # 绘制 Pitch ± 1σ 范围
        ax2.fill_between(time_rel, 
                         np.degrees(self.pitch_mean - self.pitch_std),
                         np.degrees(self.pitch_mean + self.pitch_std),
                         color='red', alpha=0.1, label='Pitch ±1σ')

        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Angle [deg]')
        ax2.set_title(f'Attitude Changes\nMean Pitch = {np.degrees(self.pitch_mean):.4f}° ± {np.degrees(self.pitch_std):.4f}°')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"[信息] 图片已保存至: {save_path}")
        else:
            plt.show()

    def export_cpp_code(self):
        """
        输出可直接使用的 C++ 代码片段
        """
        if self.radius is None:
            print("[错误] 请先执行 calibrate() 进行标定")
            return

        print("\n" + "=" * 60)
        print("C++ 代码片段 (可直接复制到 LIVMapper.cpp)")
        print("=" * 60)
        print(f"""
// ========== 标定后的外参 ==========
// 标定时间: {import_datetime()}
// 拟合 RMSE: {self.rmse * 1000:.3f} mm

// base_link → joint_link: 绕 Y 轴旋转 (Pitch)
double angle_y = {self.pitch_mean:.6f};  // {np.degrees(self.pitch_mean):.4f} 度

// joint_link → lidar_link: XY 联合平移外参
Eigen::Vector3d t_lidar_joint({self.dx:.6f}, {self.dy:.6f}, 0.0);  // dx: {self.dx * 1000:.3f} mm, dy: {self.dy * 1000:.3f} mm
""")


def import_datetime():
    """获取当前时间字符串"""
    from datetime import datetime
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='激光雷达外参标定工具 - 计算旋转半径和安装倾角',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python lidar_extrinsic_calibration.py /path/to/rosbag
  python lidar_extrinsic_calibration.py /path/to/rosbag -t /custom/odom_topic
  python lidar_extrinsic_calibration.py /path/to/rosbag -o result.png
        """
    )
    parser.add_argument('bag_path', type=str, help='ROS 2 bag 文件路径')
    parser.add_argument('-t', '--topic', type=str, default='/aft_mapped_to_init',
                        help='里程计话题名称 (默认: /aft_mapped_to_init)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='保存可视化图片的路径 (默认: 显示窗口)')
    parser.add_argument('--no-plot', action='store_true',
                        help='不显示可视化图形')

    args = parser.parse_args()

    # 创建标定器
    calibrator = LidarExtrinsicCalibrator(args.bag_path, args.topic)

    # 读取数据
    if not calibrator.read_bag():
        sys.exit(1)

    # 执行标定
    if not calibrator.calibrate():
        sys.exit(1)

    # 输出 C++ 代码
    calibrator.export_cpp_code()

    # 可视化
    if not args.no_plot:
        calibrator.visualize(args.output)


if __name__ == "__main__":
    main()