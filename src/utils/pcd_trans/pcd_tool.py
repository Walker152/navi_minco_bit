#!/usr/bin/env python3
"""
点云处理工具 (Point Cloud Processing Tool)

功能:
1. 点云变换: 支持平移 (x, y, z) 和旋转 (roll, pitch, yaw)
2. 格式转换: 支持 ASCII, Binary, Binary Compressed 之间的转换
3. 信息查看: 显示点云的基本信息

依赖:
- numpy
- open3d (强烈推荐，用于支持所有 PCD 格式)

如果未安装 open3d，本工具仅支持 ASCII 格式的 PCD 文件。

安装依赖:
  pip3 install open3d numpy
  # 或者在国内网络环境下:
  pip3 install open3d numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
"""

import argparse
import numpy as np
import sys
import os
from pathlib import Path

# 尝试导入 Open3D
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    将欧拉角转换为旋转矩阵 (XYZ顺序: R = Rz * Ry * Rx)
    """
    # Roll (X轴)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Pitch (Y轴)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Yaw (Z轴)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    return Rz @ Ry @ Rx

class PointCloudProcessor:
    def __init__(self):
        self.points = None
        self.has_open3d = HAS_OPEN3D
        self.pcd_o3d = None  # Open3D 点云对象

    def load(self, filepath):
        """加载 PCD 文件"""
        print(f"[INFO] Loading: {filepath}")
        if not os.path.exists(filepath):
            print(f"[ERROR] File not found: {filepath}")
            return False

        if self.has_open3d:
            try:
                self.pcd_o3d = o3d.io.read_point_cloud(filepath)
                if self.pcd_o3d.is_empty():
                    print("[WARNING] Loaded point cloud is empty or format not supported by Open3D.")
                    # 尝试回退到手动解析 (仅限 ASCII)
                    return self._load_ascii_fallback(filepath)
                
                self.points = np.asarray(self.pcd_o3d.points)
                print(f"[SUCCESS] Loaded {len(self.points)} points using Open3D.")
                return True
            except Exception as e:
                print(f"[ERROR] Open3D failed to load file: {e}")
                return False
        else:
            return self._load_ascii_fallback(filepath)

    def _load_ascii_fallback(self, filepath):
        """手动解析 ASCII PCD 文件 (无 Open3D 时使用)"""
        print("[INFO] Open3D not available. Attempting to load as ASCII PCD...")
        try:
            header = {}
            points = []
            with open(filepath, 'rb') as f:
                while True:
                    line = f.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        break
                    
                    if line.startswith('DATA'):
                        header['DATA'] = line.split()[1]
                        break
                    
                    parts = line.split(maxsplit=1)
                    if len(parts) == 2:
                        header[parts[0]] = parts[1]

                if header.get('DATA') != 'ascii':
                    print(f"[ERROR] Unsupported format '{header.get('DATA')}' without Open3D.")
                    print("Please install Open3D to handle binary/compressed files:")
                    print("  pip3 install open3d")
                    return False

                # 读取数据
                for line in f:
                    line = line.decode('utf-8', errors='ignore').strip()
                    if line:
                        vals = list(map(float, line.split()))
                        if len(vals) >= 3:
                            points.append(vals[:3])
            
            self.points = np.array(points)
            print(f"[SUCCESS] Loaded {len(self.points)} points (ASCII mode).")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to load ASCII file: {e}")
            return False

    def transform(self, tx, ty, tz, roll, pitch, yaw):
        """应用平移和旋转"""
        if self.points is None:
            return

        print(f"[INFO] Applying Transform:")
        print(f"  Translation: [{tx}, {ty}, {tz}]")
        print(f"  Rotation (deg): [{np.rad2deg(roll)}, {np.rad2deg(pitch)}, {np.rad2deg(yaw)}]")

        # 1. 旋转
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        self.points = (R @ self.points.T).T

        # 2. 平移
        translation = np.array([tx, ty, tz])
        self.points += translation

        # 如果使用 Open3D，同步更新 Open3D 对象
        if self.has_open3d and self.pcd_o3d is not None:
            self.pcd_o3d.points = o3d.utility.Vector3dVector(self.points)

    def save(self, filepath, ascii_format=False):
        """保存 PCD 文件"""
        print(f"[INFO] Saving to: {filepath}")
        
        if self.has_open3d and self.pcd_o3d is not None:
            # 使用 Open3D 保存
            # write_point_cloud 默认保存为 binary compressed，除非指定 write_ascii=True
            # 或者文件名以 .xyz 结尾等
            success = o3d.io.write_point_cloud(filepath, self.pcd_o3d, write_ascii=ascii_format)
            if success:
                fmt = "ASCII" if ascii_format else "Binary/Compressed"
                print(f"[SUCCESS] Saved using Open3D ({fmt}).")
            else:
                print("[ERROR] Open3D failed to save file.")
        else:
            # 手动保存为 ASCII
            self._save_ascii_fallback(filepath)

    def _save_ascii_fallback(self, filepath):
        """手动保存为 ASCII PCD"""
        try:
            with open(filepath, 'w') as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {len(self.points)}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(self.points)}\n")
                f.write("DATA ascii\n")
                for p in self.points:
                    f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            print("[SUCCESS] Saved as ASCII PCD (Fallback).")
        except Exception as e:
            print(f"[ERROR] Failed to save file: {e}")

def main():
    # ==================== 配置区域 (请在此处修改参数) ====================
    
    # 输入文件路径
    INPUT_PCD = "src/navigation/navi2_bringup/maps/pcd/2026rmuc.pcd"
    
    # 输出文件路径
    OUTPUT_PCD = "src/perception/small_gicp/PCD/2026rmuc_trans.pcd"
    
    # 平移参数 (单位: 米)
    TX = -3.2
    TY = -10.2
    TZ = 0.0
    
    # 旋转参数 (单位: 度)
    ROLL = 0.0
    PITCH = 0.0
    YAW = 0.0
    
    # 格式选项
    # True: 强制保存为 ASCII 格式 (方便阅读，兼容性好)
    # False: 如果有 Open3D，保存为 Binary Compressed (体积小，加载快)
    SAVE_AS_ASCII = True
    
    # ==================================================================

    print("=" * 60)
    print("PCD 点云处理工具")
    print("=" * 60)

    # 检查 Open3D
    if not HAS_OPEN3D:
        print("!"*60)
        print("警告: 未检测到 Open3D 库。")
        print("本工具将仅支持 ASCII 格式的 PCD 文件。")
        print("如果您的输入文件是 Binary 或 Binary Compressed 格式，将会加载失败。")
        print("建议安装 Open3D: pip3 install open3d")
        print("!"*60)

    processor = PointCloudProcessor()
    
    # 1. 加载
    if not processor.load(INPUT_PCD):
        sys.exit(1)

    # 2. 变换
    # 将角度转换为弧度
    r_rad = np.deg2rad(ROLL)
    p_rad = np.deg2rad(PITCH)
    y_rad = np.deg2rad(YAW)
    
    if any([TX, TY, TZ, ROLL, PITCH, YAW]):
        processor.transform(TX, TY, TZ, r_rad, p_rad, y_rad)
    else:
        print("[INFO] No transformation parameters provided. Only performing format conversion/copy.")

    # 3. 保存
    # 如果没有 Open3D，只能存 ASCII。如果有 Open3D，根据配置决定
    save_ascii = SAVE_AS_ASCII or (not HAS_OPEN3D)
    processor.save(OUTPUT_PCD, ascii_format=save_ascii)

if __name__ == "__main__":
    main()
