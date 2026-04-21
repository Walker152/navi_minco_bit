#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('msg_convert')
    default_config = os.path.join(pkg_share, 'config', 'cloud_registered_crop_filter.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the crop filter config yaml file'
    )

    filter_node = Node(
        package='msg_convert',
        executable='cloud_registered_crop_filter',
        name='cloud_registered_crop_filter',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        config_file_arg,
        filter_node,
    ])
