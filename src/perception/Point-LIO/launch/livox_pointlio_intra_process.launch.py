from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    driver_params_file = LaunchConfiguration("driver_params_file")
    pointlio_params_file = LaunchConfiguration("pointlio_params_file")
    use_intra_process = LaunchConfiguration("use_intra_process")
    container_name = LaunchConfiguration("container_name")
    log_level = LaunchConfiguration("log_level")
    pointlio_imu_topic = LaunchConfiguration("pointlio_imu_topic")
    intra_process_value = ParameterValue(use_intra_process, value_type=bool)

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        composable_node_descriptions=[
            ComposableNode(
                package="livox_ros_driver2",
                plugin="livox_ros::DriverNode",
                name="livox_driver_node",
                parameters=[ParameterFile(driver_params_file, allow_substs=True)],
                extra_arguments=[{"use_intra_process_comms": intra_process_value}],
            ),
            ComposableNode(
                package="point_lio",
                plugin="point_lio::LaserMappingNode",
                name="laserMapping",
                parameters=[
                    pointlio_params_file,
                    {"common.imu_topic": pointlio_imu_topic},
                ],
                extra_arguments=[{"use_intra_process_comms": intra_process_value}],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "driver_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("livox_ros_driver2"),
                        "config",
                        "mixed_MID360_component.yaml",
                    ]
                ),
                description="ROS2 parameter yaml for livox_ros_driver2 DriverNode.",
            ),
            DeclareLaunchArgument(
                "pointlio_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("point_lio"), "config", "mid360.yaml"]
                ),
                description="Point-LIO parameter yaml.",
            ),
            DeclareLaunchArgument("use_intra_process", default_value="true"),
            DeclareLaunchArgument(
                "container_name",
                default_value="livox_pointlio_container",
            ),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "pointlio_imu_topic",
                default_value="livox/imu_192_168_1_135",
            ),
            container,
        ]
    )
