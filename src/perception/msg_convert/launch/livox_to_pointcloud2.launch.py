#!/usr/bin/env python3
"""
Launch file for Livox to PointCloud2 converter
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('msg_convert')
    config_file = os.path.join(pkg_share, 'config', 'livox_converter.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration yaml file'
    )
    
    # Create the converter node
    converter_node = Node(
        package='msg_convert',
        executable='livox_to_pointcloud2',
        name='livox_to_pointcloud2',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # 可选的重映射配置
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        converter_node,
    ])
