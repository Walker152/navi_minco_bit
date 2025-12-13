#!/usr/bin/env python3
"""
PCD 点云变换工具
功能：对 PCD 文件进行旋转、平移和裁剪操作
"""

import numpy as np
import sys
from pathlib import Path

# ==================== 配置参数 ====================

# 输入输出文件
INPUT_PCD = "input.pcd"
OUTPUT_PCD = "output.pcd"

# 平移参数 (单位: 米)
TRANSLATION_X = 0.0
TRANSLATION_Y = 0.0
TRANSLATION_Z = 0.0

# 旋转参数 (单位: 度)
ROTATION_ROLL = 0.0   # 绕X轴旋转
ROTATION_PITCH = 0.0  # 绕Y轴旋转
ROTATION_YAW = 0.0    # 绕Z轴旋转

# 裁剪参数 (单位: 米)
ENABLE_CROP = False   # 是否启用裁剪
CROP_X_MIN = -100.0
CROP_X_MAX = 100.0
CROP_Y_MIN = -100.0
CROP_Y_MAX = 100.0
CROP_Z_MIN = -100.0
CROP_Z_MAX = 100.0

# 是否显示统计信息
SHOW_STATS = True

# ==================================================


def read_pcd(filename):
    """
    读取 PCD 文件
    返回: points (Nx3 numpy array), header (字典)
    """
    print(f"Reading PCD file: {filename}")
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    header = {}
    header_end = 0
    
    # 解析头部
    for i, line in enumerate(lines):
        line = line.strip()
        if line.startswith('DATA'):
            header_end = i + 1
            header['DATA'] = line.split()[1]
            break
        
        if line:
            parts = line.split(maxsplit=1)
            if len(parts) == 2:
                header[parts[0]] = parts[1]
    
    # 获取点的数量
    num_points = int(header.get('POINTS', 0))
    
    # 解析数据
    if header['DATA'] == 'ascii':
        data_lines = lines[header_end:]
        points = []
        for line in data_lines:
            line = line.strip()
            if line:
                values = list(map(float, line.split()))
                if len(values) >= 3:
                    points.append(values[:3])  # 只取 x, y, z
        points = np.array(points)
    else:
        print("Error: Only ASCII format is supported")
        sys.exit(1)
    
    print(f"Loaded {len(points)} points")
    return points, header


def write_pcd(filename, points, header=None):
    """
    写入 PCD 文件（ASCII 格式）
    """
    print(f"Writing PCD file: {filename}")
    
    with open(filename, 'w') as f:
        # 写入头部
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        # 写入数据
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    
    print(f"Saved {len(points)} points")


def rotation_matrix_from_rpy(roll, pitch, yaw):
    """
    从欧拉角（弧度）构建旋转矩阵
    顺序: ZYX (Yaw-Pitch-Roll)
    """
    # Roll (X轴旋转)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Pitch (Y轴旋转)
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Yaw (Z轴旋转)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # 组合旋转: R = Rz * Ry * Rx
    R = Rz @ Ry @ Rx
    return R


def transform_points(points, translation, rotation_rpy):
    """
    对点云进行旋转和平移变换
    
    Args:
        points: Nx3 点云数组
        translation: [tx, ty, tz] 平移向量
        rotation_rpy: [roll, pitch, yaw] 欧拉角（弧度）
    
    Returns:
        变换后的点云
    """
    print("\nApplying transformation...")
    print(f"  Translation: [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}] m")
    print(f"  Rotation (RPY): [{rotation_rpy[0]:.3f}, {rotation_rpy[1]:.3f}, {rotation_rpy[2]:.3f}] rad")
    print(f"  Rotation (Deg): [{np.rad2deg(rotation_rpy[0]):.1f}, {np.rad2deg(rotation_rpy[1]):.1f}, {np.rad2deg(rotation_rpy[2]):.1f}] deg")
    
    # 构建旋转矩阵
    R = rotation_matrix_from_rpy(*rotation_rpy)
    
    # 先旋转，后平移
    transformed = (R @ points.T).T + translation
    
    return transformed


def crop_points(points, x_range, y_range, z_range):
    """
    裁剪点云
    
    Args:
        points: Nx3 点云数组
        x_range: [xmin, xmax]
        y_range: [ymin, ymax]
        z_range: [zmin, zmax]
    
    Returns:
        裁剪后的点云
    """
    print("\nApplying crop...")
    print(f"  X range: [{x_range[0]:.2f}, {x_range[1]:.2f}] m")
    print(f"  Y range: [{y_range[0]:.2f}, {y_range[1]:.2f}] m")
    print(f"  Z range: [{z_range[0]:.2f}, {z_range[1]:.2f}] m")
    
    mask = (
        (points[:, 0] >= x_range[0]) & (points[:, 0] <= x_range[1]) &
        (points[:, 1] >= y_range[0]) & (points[:, 1] <= y_range[1]) &
        (points[:, 2] >= z_range[0]) & (points[:, 2] <= z_range[1])
    )
    
    cropped = points[mask]
    print(f"  Kept {len(cropped)}/{len(points)} points ({100*len(cropped)/len(points):.1f}%)")
    
    return cropped


def print_point_cloud_stats(points, label="Point cloud"):
    """打印点云统计信息"""
    print(f"\n{label} statistics:")
    print(f"  Number of points: {len(points)}")
    print(f"  X range: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y range: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    print(f"  Centroid: [{points[:, 0].mean():.2f}, {points[:, 1].mean():.2f}, {points[:, 2].mean():.2f}]")


def main():
    print("=" * 60)
    print("PCD 点云变换工具")
    print("=" * 60)
    
    # 检查输入文件
    if not Path(INPUT_PCD).exists():
        print(f"Error: Input file '{INPUT_PCD}' does not exist")
        sys.exit(1)
    
    # 读取点云
    points, header = read_pcd(INPUT_PCD)
    
    if SHOW_STATS:
        print_point_cloud_stats(points, "Original point cloud")
    
    # 转换旋转角度为弧度
    rotation_rad = np.deg2rad([ROTATION_ROLL, ROTATION_PITCH, ROTATION_YAW])
    translation = np.array([TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z])
    
    # 应用变换
    has_transform = np.any(translation != 0) or np.any(rotation_rad != 0)
    
    if has_transform:
        points = transform_points(points, translation, rotation_rad)
        if SHOW_STATS:
            print_point_cloud_stats(points, "Transformed point cloud")
    else:
        print("\nNo transformation applied (all parameters are zero)")
    
    # 应用裁剪
    if ENABLE_CROP:
        x_range = [CROP_X_MIN, CROP_X_MAX]
        y_range = [CROP_Y_MIN, CROP_Y_MAX]
        z_range = [CROP_Z_MIN, CROP_Z_MAX]
        points = crop_points(points, x_range, y_range, z_range)
        if SHOW_STATS:
            print_point_cloud_stats(points, "Cropped point cloud")
    else:
        print("\nCrop disabled")
    
    # 保存结果
    write_pcd(OUTPUT_PCD, points, header)
    print(f"\n{'=' * 60}")
    print("✓ Successfully processed point cloud!")
    print(f"  Input:  {INPUT_PCD}")
    print(f"  Output: {OUTPUT_PCD}")
    print("=" * 60)


if __name__ == '__main__':
    main()
