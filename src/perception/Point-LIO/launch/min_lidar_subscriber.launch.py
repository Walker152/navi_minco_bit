from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lid_topic = LaunchConfiguration("lid_topic")
    msg_type = LaunchConfiguration("msg_type")
    qos_mode = LaunchConfiguration("qos_mode")
    qos_depth = LaunchConfiguration("qos_depth")
    print_period = LaunchConfiguration("print_period")

    min_subscriber = Node(
        package="point_lio",
        executable="min_lidar_subscriber",
        name="min_lidar_subscriber",
        output="screen",
        parameters=[
            {
                "lid_topic": lid_topic,
                "msg_type": msg_type,
                "qos_mode": qos_mode,
                "qos_depth": ParameterValue(qos_depth, value_type=int),
                "print_period": ParameterValue(print_period, value_type=float),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("lid_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("msg_type", default_value="pointcloud2"),
            DeclareLaunchArgument("qos_mode", default_value="sensor"),
            DeclareLaunchArgument("qos_depth", default_value="5"),
            DeclareLaunchArgument("print_period", default_value="1.0"),
            min_subscriber,
        ]
    )
