from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

bt_manager_share = FindPackageShare('bt_manager').find('bt_manager')
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bt_debug_logs',
            default_value='true',
            description='Enable behavior-tree transition debug logs'),
        DeclareLaunchArgument(
            'bt_debug_log_to_file',
            default_value='false',
            description='Enable behavior-tree transition logs to file'),
        DeclareLaunchArgument(
            'bt_debug_log_file',
            default_value=bt_manager_share + '/logs/bt_transition.log',
            description='Behavior-tree transition log file path'),
        DeclareLaunchArgument(
            'publish_area_markers',
            default_value='true',
            description='Whether to publish area markers for visualization'),
        Node(
            package='bt_manager',
            executable='bt_manager_node',
            name='bt_manager_node',
            output='screen',
            parameters=[{
                'bt_debug_logs': ParameterValue(LaunchConfiguration('bt_debug_logs'), value_type=bool),
                'bt_debug_log_to_file': ParameterValue(
                    LaunchConfiguration('bt_debug_log_to_file'), value_type=bool),
                'bt_debug_log_file': LaunchConfiguration('bt_debug_log_file'),
                'publish_area_markers': ParameterValue(LaunchConfiguration('publish_area_markers'), value_type=bool),
            }]),
    ])
