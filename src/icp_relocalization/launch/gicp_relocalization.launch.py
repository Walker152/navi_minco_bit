from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    declare_target_pcd_file_cmd = DeclareLaunchArgument(
        'target_pcd_file',
        default_value='src/icp_relocalization/maps/rmuc.pcd',
        description='Path to the target PCD map file.'
    )

    ld = LaunchDescription()
    ld.add_action(declare_target_pcd_file_cmd)

    # GICP重定位节点
    gicp_node = Node(
        package='icp_relocalization',
        executable='gicp_node',
        name='gicp_relocalization_node',
        output='screen',
        parameters=[
            # 文件路径
            {'target_pcd_file': LaunchConfiguration('target_pcd_file')},
            
            # GICP核心参数
            {'gicp.target_voxel_leaf_size': 0.5},
            {'gicp.source_voxel_leaf_size': 0.3},
            {'gicp.max_correspondence_distance': 1.0}, # 稍微放宽一点，旧的0.1可能太严格
            {'gicp.max_iterations': 75},
            {'gicp.transformation_epsilon': 0.01},
            {'gicp.euclidean_fitness_epsilon': 0.01},

            # 坐标系和频率
            {'base_frame': 'base_link'},
            {'map_frame': 'map'},
            {'icp_frequency': 1.0}, # 1 Hz

            # 漂移检测阈值
            {'drift_threshold_m': 1.0},
            {'drift_threshold_rad': 0.5}
        ],
    )

    ld.add_action(gicp_node)
    
    return ld
