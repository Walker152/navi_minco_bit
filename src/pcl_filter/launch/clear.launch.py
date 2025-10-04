import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('pcl_filter') 
    
    # 构建参数文件的绝对路径
    param_file_cube = os.path.join(pkg_share_dir, 'config', 'cube.yaml') 
    # 构建参数文件路径
    
    # 定义要启动的节点
    clear_node = Node(
        package='pcl_filter',
        executable='use_clear',  # 对应CMakeLists.txt中add_executable定义的名字
        name='clear_it',
        output='screen',
        # 重映射规则：旧话题名 -> 新话题名
        remappings=[
            ('/odom_topic', '/a'),
            ('/cloud_registered', '/cloud_registered'), # 请确认此重映射是否必要
            ('/cloud_filter_baselink', '/cloud_filter_baselink'), # 请确认此重映射是否必要
        ],
        # 加载参数文件
        parameters=[param_file_cube]
        # 你也可以选择直接在此定义参数（字典形式），而不是从文件加载：
        # parameters=[{'some_parameter_name': 'some_value'}]
    )

    # 创建启动描述并添加节点
    ld = LaunchDescription([clear_node])
    
    return ld