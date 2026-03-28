#!/usr/bin/env python3
"""
综合启动脚本：启动点云发布、过滤和RViz2可视化
"""
import subprocess
import time
import sys
import os

def run_command(cmd, name, background=True):
    """运行命令"""
    print(f"\n[启动] {name}")
    print(f"命令: {cmd}\n")
    
    if background:
        subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)
        time.sleep(1)
    else:
        subprocess.run(cmd, shell=True)

def main():
    workspace_path = "/home/rm/sentinel-up-gimbal/src/pclfilter"
    source_cmd = f"cd {workspace_path} && source /opt/ros/humble/setup.bash && source install/local_setup.bash"
    
    print("=" * 60)
    print("pclfilter 综合启动系统")
    print("=" * 60)
    
    # 1. 启动过滤节点（保留运行核心功能）
    cmd1 = f"{source_cmd} && ./install/pclfilter/lib/pclfilter/clear_node"
    run_command(cmd1, "点云过滤节点 (clear_node)", background=True)

    # 2. 启动RViz2
    time.sleep(2)
    cmd2 = f"source /opt/ros/humble/setup.bash && rviz2"
    run_command(cmd2, "RViz2可视化工具", background=False)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n程序已中止")
        sys.exit(0)
