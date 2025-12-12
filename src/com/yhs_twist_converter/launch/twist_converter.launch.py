#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 声明启动参数
    default_gear_arg = DeclareLaunchArgument(
        'default_gear',
        default_value='8',
        description='Default gear for the chassis (0=reverse, 1=neutral, 2=forward)'
    )
    
    max_linear_x_arg = DeclareLaunchArgument(
        'max_linear_x',
        default_value='2.0',
        description='Maximum linear velocity in x direction (m/s)'
    )
    
    max_linear_y_arg = DeclareLaunchArgument(
        'max_linear_y',
        default_value='2.0',
        description='Maximum linear velocity in y direction (m/s)'
    )
    
    max_angular_z_arg = DeclareLaunchArgument(
        'max_angular_z',
        default_value='2.0',
        description='Maximum angular velocity in z direction (rad/s)'
    )

    # 创建转换节点
    twist_to_ctrl_cmd_node = Node(
        package='yhs_twist_converter',
        executable='twist_to_ctrl_cmd_node',
        name='twist_to_ctrl_cmd_node',
        output='screen',
        parameters=[{
            'default_gear': LaunchConfiguration('default_gear'),
            'max_linear_x': LaunchConfiguration('max_linear_x'),
            'max_linear_y': LaunchConfiguration('max_linear_y'),
            'max_angular_z': LaunchConfiguration('max_angular_z'),
        }],
        remappings=[
            # 如果需要重映射话题名称，可以在这里添加
            # ('cmd_vel', 'teleop/cmd_vel'),  # 示例：重映射输入话题
            # ('ctrl_cmd', 'robot/ctrl_cmd'), # 示例：重映射输出话题
        ]
    )

    return LaunchDescription([
        default_gear_arg,
        max_linear_x_arg,
        max_linear_y_arg,
        max_angular_z_arg,
        twist_to_ctrl_cmd_node
    ])