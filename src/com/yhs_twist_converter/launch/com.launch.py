#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # yhs_can_control package share dir (用于 params_file 默认)
    can_share_dir = get_package_share_directory('yhs_can_control')
    default_params_file = os.path.join(can_share_dir, 'params', 'cfg.yaml')

    # Declare launch arguments
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS2 parameters file to use for yhs_can_control.'
    )

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

    # LaunchConfigurations
    params_file = LaunchConfiguration('params_file')
    default_gear = LaunchConfiguration('default_gear')
    max_linear_x = LaunchConfiguration('max_linear_x')
    max_linear_y = LaunchConfiguration('max_linear_y')
    max_angular_z = LaunchConfiguration('max_angular_z')

    # Nodes
    yhs_can_control_node = Node(
        package='yhs_can_control',
        executable='yhs_can_control_node',
        name='yhs_can_control_node',
        output='screen',
        parameters=[params_file]
    )

    twist_to_ctrl_cmd_node = Node(
        package='yhs_twist_converter',
        executable='twist_to_ctrl_cmd_node',
        name='twist_to_ctrl_cmd_node',
        output='screen',
        parameters=[{
            'default_gear': default_gear,
            'max_linear_x': max_linear_x,
            'max_linear_y': max_linear_y,
            'max_angular_z': max_angular_z,
        }]
    )

    return LaunchDescription([
        params_declare,
        default_gear_arg,
        max_linear_x_arg,
        max_linear_y_arg,
        max_angular_z_arg,
        yhs_can_control_node,
        twist_to_ctrl_cmd_node
    ])