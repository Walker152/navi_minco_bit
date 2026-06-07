import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'
enable_internal_lidar_merge = True
enable_merge_debug = False
merge_front_ip = '192.168.1.135'
merge_back_ip = '192.168.1.122'
merge_output_topic = 'livox/lidar'
merge_frame_id = 'livox_frame'
merge_max_interval_ms = 5.0
merge_extrinsic_back_to_front = [0.0, 0.4, 0.0, -0.35453, 0.0, 0.0]

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
# However, if you want to keep the relative path logic from your example, 
# be aware it relies on file location.

livox_ros_driver2_dir = get_package_share_directory('livox_ros_driver2')
user_config_path = os.path.join(livox_ros_driver2_dir, 'config', 'mixed_MID360_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"enable_internal_lidar_merge": enable_internal_lidar_merge},
    {"enable_merge_debug": enable_merge_debug},
    {"merge_front_ip": merge_front_ip},
    {"merge_back_ip": merge_back_ip},
    {"merge_output_topic": merge_output_topic},
    {"merge_frame_id": merge_frame_id},
    {"merge_max_interval_ms": merge_max_interval_ms},
    {"merge_extrinsic_back_to_front": merge_extrinsic_back_to_front}
]

def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
