#!/usr/bin/env python3
"""
测试depth_cluster功能的Launch文件
同时启动：
1. 测试点云发布器（发布包含地面和障碍物的点云）
2. depth_cluster节点（进行地面提取和障碍物聚类）
3. RViz2可视化（可选）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包路径
    pclfilter_dir = FindPackageShare('pclfilter')
    rviz_config = PathJoinSubstitution([pclfilter_dir, 'config', 'depth_cluster_test.rviz'])
    
    # Launch参数
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2 for visualization'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='2.0',
        description='Point cloud publishing rate (Hz)'
    )
    
    add_noise_arg = DeclareLaunchArgument(
        'add_noise',
        default_value='true',
        description='Add noise to point cloud'
    )
    
    scene_type_arg = DeclareLaunchArgument(
        'scene_type',
        default_value='complex',
        description='Scene type: simple, complex, dynamic, or slope'
    )
    
    # 测试点云发布器
    test_publisher = Node(
        package='pclfilter',
        executable='test_depth_cluster_demo.py',
        name='depth_cluster_test_publisher',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'add_noise': LaunchConfiguration('add_noise'),
            'scene_type': LaunchConfiguration('scene_type'),
        }],
        emulate_tty=True,
    )
    
    # depth_cluster节点
    depth_cluster_node = Node(
        package='pclfilter',
        executable='depth_cluster_node',
        name='depth_cluster_node',
        output='screen',
        emulate_tty=True,
    )
    
    # RViz2可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    
    # 静态TF发布器 (camera_init -> map)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
    )
    
    return LaunchDescription([
        # 参数声明
        use_rviz_arg,
        publish_rate_arg,
        add_noise_arg,
        scene_type_arg,
        
        # 节点
        static_tf,
        test_publisher,
        depth_cluster_node,
        rviz_node,
    ])
