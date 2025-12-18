import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import launch.actions
import launch.conditions

def generate_launch_description():
    pkg_share = get_package_share_directory('icp_relocalization')
    msg_convert_pkg_share = get_package_share_directory('msg_convert')

    default_config_path = os.path.join(pkg_share, 'config', 'gicp_relocalization.yaml')
    declare_gicp_config_cmd = DeclareLaunchArgument(
        'gicp_config',
        default_value=default_config_path,
        description='Full path to the GICP relocalization config file.'
    )

    ld = LaunchDescription()

    ld.add_action(declare_gicp_config_cmd)

    # 包含 msg_convert 的 launch 文件
    msg_convert_launch_path = os.path.join(msg_convert_pkg_share, 'launch', 'livox_to_pointcloud2.launch.py')
    msg_convert_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(msg_convert_launch_path),
        launch_arguments={}.items(),
    )

    ld.add_action(msg_convert_node)

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
