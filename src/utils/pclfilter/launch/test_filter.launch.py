#!/usr/bin/env python3
"""
测试点云过滤效果的Launch文件
同时启动：
1. 测试点云发布器
2. 点云过滤节点 (clear_node)
3. RViz2可视化（可选）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包路径
    pclfilter_dir = FindPackageShare('pclfilter')
    
    # 配置文件路径
    cube_yaml = PathJoinSubstitution([pclfilter_dir, 'config', 'cube.yaml'])
    basemap_yaml = PathJoinSubstitution([pclfilter_dir, 'config', 'basemap.yaml'])
    rviz_config = PathJoinSubstitution([pclfilter_dir, 'config', 'simple_rviz2.rviz'])
    
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
    
    animate_arg = DeclareLaunchArgument(
        'animate',
        default_value='false',
        description='Animate point cloud movement'
    )
    
    # 测试点云发布器
    test_publisher = Node(
        package='pclfilter',
        executable='test_filter_demo.py',
        name='test_filter_publisher',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'add_noise': LaunchConfiguration('add_noise'),
            'animate': LaunchConfiguration('animate'),
        }],
        emulate_tty=True,
    )
    
    # 点云过滤节点
    clear_node = Node(
        package='pclfilter',
        executable='clear_node',
        name='clear_node',
        output='screen',
        parameters=[
            {'cube_file': cube_yaml},
            {'polygons_file': basemap_yaml},
        ],
        remappings=[
            ('/cloud_registered', '/cloud_registered'),
            ('/cloud_filter_baselink', '/cloud_filtered'),
        ],
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
        animate_arg,
        
        # 节点
        static_tf,
        test_publisher,
        clear_node,
        rviz_node,
    ])
