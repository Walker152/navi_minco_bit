import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('icp_relocalization')

    default_config_path = os.path.join(pkg_share, 'config', 'gicp_relocalization.yaml')
    declare_gicp_config_cmd = DeclareLaunchArgument(
        'gicp_config',
        default_value=default_config_path,
        description='Full path to the GICP relocalization config file.'
    )

    ld = LaunchDescription()

    ld.add_action(declare_gicp_config_cmd)

    # GICP重定位节点
    gicp_node = Node(
        package='icp_relocalization',
        executable='gicp_node',
        name='gicp_relocalization_node',
        output='screen',
        parameters=[LaunchConfiguration('gicp_config')],
    )

    ld.add_action(gicp_node)
    
    return ld
