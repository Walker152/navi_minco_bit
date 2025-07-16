import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 自己的功能包
    sentry_nav2_dir = get_package_share_directory("navigate")
    # 系统自带导航体验包
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    # 使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml_path = LaunchConfiguration(
        "map", default=os.path.join(sentry_nav2_dir, "map", "rmuc_2025.yaml")
    )

    nav2_param_path = LaunchConfiguration(
        "params_file",
        default=os.path.join(sentry_nav2_dir, "config", "nav2_params.yaml"),
    )

    # rviz_config_dir = os.path.join(fishbot_navigation2_dir, "config", "navigation.rviz")
    rviz_config_dir = os.path.join(sentry_nav2_dir, "rviz", "nav2_default_view.rviz")


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=map_yaml_path,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_param_path,
                description="Full path to param file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_dir, "/launch", "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_yaml_path,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_param_path,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
