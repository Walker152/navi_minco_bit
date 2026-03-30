import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取与拼接默认路径
    DreamChaser_dir = get_package_share_directory(
        'navi2')
    # 官方包
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        DreamChaser_dir,'config', 'our.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='False')
    
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        # 'map', default=os.path.join(DreamChaser_dir, 'maps', '2026/rmuc2026.yaml'))
        # 'map', default=os.path.join(DreamChaser_dir, 'maps', '2026/rmul2026.yaml'))
        # 'map', default=os.path.join(DreamChaser_dir, 'maps', '2026/hotel.yaml'))
        #  'map', default=os.path.join(DreamChaser_dir, 'maps', '2026/room.yaml'))
        'map', default=os.path.join(DreamChaser_dir, 'maps', 'first_floor/first_floor.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(DreamChaser_dir, 'params', 'sentry1.yaml'))

    return launch.LaunchDescription([
        # 静态TF: map -> camera_init
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_to_camera_init',
            # arguments=['1.73', '6.4', '0', '0.0', '0', '0.0', 'map', 'camera_init'],
            # arguments=['5', '7', '0', '0', '0', '0', 'map', 'camera_init'],
            # arguments=['6.8', '3.67', '0', '0', '0', '0', 'map', 'camera_init'],
            arguments=['12', '4.5', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen'),

        # # 静态TF: camera_init -> body
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_camera_init_to_body',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'body'],
        #     output='screen'),

        # # 静态TF: body -> base_link
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_body_to_base_link',
        #     arguments=['0', '0.15', '0', '0', '0', '0', 'body', 'base_link'],
        #     output='screen'),
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),           
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])