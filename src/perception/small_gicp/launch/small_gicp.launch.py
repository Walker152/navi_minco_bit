from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 获取功能包路径
    pkg_share = FindPackageShare('small_gicpapp').find('small_gicpapp')
    
    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'small_gicp.yaml')
    
    # PCD文件路径
    pcd_file_path = os.path.join(pkg_share, 'PCD', 'scans.pcd')
    
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the YAML config file'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/stdpc',
        description='Input point cloud topic'
    )
    
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value=pcd_file_path,
        description='Path to the reference PCD file'
    )
    
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [
        ("/tf", "tf"), 
        ("/tf_static", "tf_static"),
        ("registered_scan", LaunchConfiguration('input_topic')),  # 重映射输入点云话题
    ]

    node = Node(
        package="small_gicpapp",
        executable="small_gicpapp",
        namespace="",
        output="screen",
        remappings=remappings,
        parameters=[
            LaunchConfiguration('config_file'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        input_topic_arg,
        pcd_file_arg,
        node
    ])
