#!/usr/bin/env python3
"""
参数调试工具
用于快速测试不同的参数组合并比较结果
Parameter Debug Tool - Test different parameter combinations and compare results
"""

import subprocess
import time
import re
from pathlib import Path

class DebugParams:
    def __init__(self):
        self.workspace = Path("/home/rm/pclfilter")
        self.log_file = Path("/tmp/depth_cluster_debug.log")
        self.test_results = []
    
    def kill_processes(self):
        """杀死所有相关进程"""
        print("🧹 清理进程...")
        subprocess.run("killall -9 depth_cluster_node rviz2 2>/dev/null || true", 
                      shell=True, capture_output=True)
        time.sleep(1)
    
    def start_node(self, radius: float, min_neighbors: int, max_angle: float, verbose: bool = True):
        """启动节点并设置参数"""
        print(f"\n🚀 启动节点: radius={radius}, neighbors={min_neighbors}, max_angle={max_angle}°")
        
        # 启动 TF 发布器
        subprocess.run(
            "source /opt/ros/humble/setup.bash && "
            "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init > /dev/null 2>&1 &",
            shell=True, cwd=self.workspace
        )
        time.sleep(1)
        
        # 清空日志
        self.log_file.write_text("")
        
        # 启动节点
        cmd = f"""
        cd {self.workspace} && \
        source /opt/ros/humble/setup.bash && \
        source install/setup.bash && \
        ros2 run pclfilter depth_cluster_node \
          --ros-args \
          -p normal_estimation_radius:={radius} \
          -p min_neighbors:={min_neighbors} \
          -p max_slope_angle_degrees:={max_angle} \
          -p verbose:={str(verbose).lower()} \
          -p print_interval:=1 \
          > {self.log_file} 2>&1 &
        """
        subprocess.run(cmd, shell=True)
        time.sleep(3)
    
    def read_stats(self) -> dict:
        """从日志读取统计数据"""
        try:
            with open(self.log_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            stats = {
                'ground_count': 0,
                'ground_percent': 0.0,
                'input_points': 0,
            }
            
            # 提取数据
            ground_match = re.search(r'✅ 接受为地面[^:]*: (\d+)\s*\(([\d.]+)%\)', content)
            if ground_match:
                stats['ground_count'] = int(ground_match.group(1))
                stats['ground_percent'] = float(ground_match.group(2))
            
            input_match = re.search(r'Input points[^:]*: (\d+)', content)
            if input_match:
                stats['input_points'] = int(input_match.group(1))
            
            return stats
        except:
            return {}
    
    def test_combination(self, radius: float, min_neighbors: int, max_angle: float):
        """测试一个参数组合"""
        self.kill_processes()
        self.start_node(radius, min_neighbors, max_angle)
        time.sleep(2)
        
        stats = self.read_stats()
        if stats:
            result = {
                'radius': radius,
                'min_neighbors': min_neighbors,
                'max_angle': max_angle,
                'ground_percent': stats.get('ground_percent', 0),
                'ground_count': stats.get('ground_count', 0),
            }
            self.test_results.append(result)
            print(f"✅ 检测率: {stats.get('ground_percent', 0):.1f}% ({stats.get('ground_count', 0)} 点)")
            return result
        else:
            print("⚠️  无法读取数据")
            return None
    
    def print_results(self):
        """打印结果对比"""
        if not self.test_results:
            print("❌ 无测试结果")
            return
        
        print("\n" + "="*80)
        print("📊 测试结果对比")
        print("="*80)
        print(f"{'半径(m)':<10} {'最小邻居':<12} {'最大角度(°)':<12} {'检测率(%)':<12} {'点数':<10}")
        print("-"*80)
        
        for r in self.test_results:
            print(f"{r['radius']:<10.2f} {r['min_neighbors']:<12} {r['max_angle']:<12.1f} "
                  f"{r['ground_percent']:<12.1f} {r['ground_count']:<10}")
        
        print("="*80)
        
        # 推荐最佳参数
        if len(self.test_results) > 0:
            best = max(self.test_results, key=lambda x: 
                      abs(x['ground_percent'] - 60))  # 接近60%为理想
            print(f"\n💡 推荐参数: radius={best['radius']}, neighbors={best['min_neighbors']}, "
                  f"max_angle={best['max_angle']}° (检测率: {best['ground_percent']:.1f}%)")


def main():
    print("🔧 PCL Filter 参数调试工具")
    print("="*80)
    
    debug = DebugParams()
    
    # 预设的测试方案
    print("\n选择测试方案:")
    print("1. 快速测试 (3 个参数组合)")
    print("2. 完整测试 (9 个参数组合)")
    print("3. 自定义测试")
    print("4. 显示当前配置")
    
    choice = input("\n请选择 (1-4): ").strip()
    
    if choice == "1":
        # 快速测试
        params = [
            (0.3, 5, 25),  # 保守
            (0.5, 5, 30),  # 默认
            (0.8, 5, 35),  # 宽松
        ]
        for radius, neighbors, angle in params:
            debug.test_combination(radius, neighbors, angle)
            time.sleep(1)
    
    elif choice == "2":
        # 完整测试
        params = [
            (0.3, 5, 20), (0.3, 5, 30), (0.3, 5, 40),
            (0.5, 5, 20), (0.5, 5, 30), (0.5, 5, 40),
            (0.8, 5, 20), (0.8, 5, 30), (0.8, 5, 40),
        ]
        for radius, neighbors, angle in params:
            debug.test_combination(radius, neighbors, angle)
            time.sleep(1)
    
    elif choice == "3":
        # 自定义
        try:
            radius = float(input("搜索半径 (0.3-1.0): "))
            neighbors = int(input("最小邻居数 (3-10): "))
            angle = float(input("最大斜坡角度 (15-50): "))
            debug.test_combination(radius, neighbors, angle)
        except ValueError:
            print("❌ 输入无效")
            return
    
    elif choice == "4":
        print("\n📋 当前配置文件: config/debug_params.yaml")
        yaml_file = Path("/home/rm/pclfilter/config/debug_params.yaml")
        if yaml_file.exists():
            print(yaml_file.read_text()[:500] + "...")
        return
    
    else:
        print("❌ 无效选择")
        return
    
    # 打印结果
    debug.print_results()
    
    # 清理
    print("\n🧹 清理进程...")
    debug.kill_processes()
    print("✅ 完成！")


if __name__ == "__main__":
    main()
