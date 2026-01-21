#!/usr/bin/env python3
"""
自动在RViz2中添加点云显示的脚本
通过修改RViz2配置文件
"""
import os
import yaml
import json

def create_rviz_config():
    """创建一个基础的RViz2配置文件"""
    
    config = {
        "Panels": [
            {
                "Class": "rviz_common/Displays",
                "Name": "Displays",
                "Property Tree Widget": {
                    "Expanded": [
                        "/Global Options1",
                        "/Raw PointCloud1",
                        "/Filtered PointCloud1"
                    ],
                    "Splitter Ratio": 0.5
                },
                "Tree Height": 600
            },
            {
                "Class": "rviz_common/Views",
                "Name": "Views",
                "Property Tree Widget": {
                    "Expanded": ["/Current View1"],
                    "Splitter Ratio": 0.5
                }
            }
        ],
        "Visualization Manager": {
            "Class": "",
            "Displays": [
                {
                    "Class": "rviz_common/PointCloud2",
                    "Name": "Raw PointCloud",
                    "Topic": "/cloud_registered",
                    "Color": "255; 255; 255",
                    "Size (m)": 0.05,
                    "Style": "Points",
                    "Value": True,
                    "Enabled": True
                },
                {
                    "Class": "rviz_common/PointCloud2",
                    "Name": "Filtered PointCloud",
                    "Topic": "/cloud_filter_baselink",
                    "Color": "0; 255; 0",
                    "Size (m)": 0.05,
                    "Style": "Points",
                    "Value": True,
                    "Enabled": True
                }
            ],
            "Global Options": {
                "Background Color": "50; 50; 50",
                "Fixed Frame": "camera_init",
                "Frame Rate": 30
            },
            "Views": {
                "Current": {
                    "Class": "rviz_common/Orbit",
                    "Distance": 10,
                    "Name": "Current View",
                    "Pitch": 0.785398,
                    "Target Frame": "<Fixed Frame>",
                    "Yaw": 0.785398
                }
            }
        },
        "Window Geometry": {
            "Height": 1028,
            "Width": 1855
        }
    }
    
    return config

def save_config(filepath, config):
    """保存配置为YAML文件"""
    with open(filepath, 'w') as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"✅ 配置已保存到: {filepath}")

def main():
    config_path = "/home/rm/sentinel-up-gimbal/src/pclfilter/config/auto_pointcloud.rviz"
    
    print("=" * 60)
    print("RViz2 自动配置生成器")
    print("=" * 60)
    
    config = create_rviz_config()
    
    # 先尝试保存为YAML
    # save_config(config_path, config)
    
    # 由于YAML格式问题，我们输出Python字典
    print("\n生成的配置对象：")
    print(json.dumps(config, indent=2, ensure_ascii=False))
    
    print(f"\n💡 由于RViz2配置格式复杂，建议手动操作：")
    print("""
    【方法】在RViz2中手动添加显示：
    
    1️⃣ 设置Fixed Frame为 'camera_init'
       左侧 → Global Options → Fixed Frame
    
    2️⃣ 添加原始点云
       左下 Add → By topic → /cloud_registered → PointCloud2
    
    3️⃣ 添加过滤点云  
       左下 Add → By topic → /cloud_filter_baselink → PointCloud2
    
    4️⃣ 调整大小
       在左侧 Displays 中展开点云显示，调整 Size (m) = 0.05
    """)

if __name__ == '__main__':
    main()
