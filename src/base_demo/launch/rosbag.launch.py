from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
import os
import datetime

def generate_launch_description():
    # ==================== 参数声明 ====================
    enable_recording = DeclareLaunchArgument(
        'enable_recording', 
        default_value='true',
        description='是否启用录制 (true/false)',
        choices=['true', 'false']
    )
    
    output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(os.getcwd(), 'bag_files'),
        description='录制文件保存目录'
    )
    
    topics = DeclareLaunchArgument(
        'topics',
        default_value='/chatter /number',
        description='要录制的话题列表，空格分隔'
    )
    
    duration = DeclareLaunchArgument(
        'duration',
        default_value='0',
        description='录制持续时间(秒)，0表示无限录制'
    )
    
    max_size = DeclareLaunchArgument(
        'max_size',
        default_value='0',
        description='单个文件最大尺寸(MB)，0表示无限制'
    )
    
    storage_format = DeclareLaunchArgument(
        'storage_format',
        default_value='sqlite3',
        description='存储格式 (sqlite3/mcap)',
        choices=['sqlite3', 'mcap']
    )
    
    # ==================== 动态生成文件名 ====================
    # 获取当前时间戳
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # ==================== 录制命令构建 ====================
    # 基本命令
    cmd = ['ros2', 'bag', 'record', '-a']  # -a 表示录制所有话题
    
    # 条件配置
    cmd.extend(['-s', LaunchConfiguration('storage_format')])
    cmd.extend(['-o', os.path.join(LaunchConfiguration('output_dir'), timestamp)])
    
    # 有选择地录制特定话题
    cmd.extend(['-e', LaunchConfiguration('topics')])
    
    # 限制条件
    cmd.extend(['--max-bag-size', PythonExpression(['str(', LaunchConfiguration('max_size'), '*1048576)'])])
    cmd.extend(['-d', LaunchConfiguration('duration')])
    
    # 压缩设置（如果需要）
    cmd.extend(['--compression-mode', 'file'])
    cmd.extend(['--compression-format', 'zstd'])
    
    # ==================== 录制进程 ====================
    record_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_recording'))
    )
    
    # ==================== 事件处理 ====================
    # 录制启动时的日志
    start_log = LogInfo(msg='录制已启动！保存目录: ' + 
                      PythonExpression(['"', LaunchConfiguration('output_dir'), '/" + "', timestamp, '"']))
    
    # 录制结束时的日志
    exit_log = LogInfo(msg='录制已完成！保存目录: ' + 
                     PythonExpression(['"', LaunchConfiguration('output_dir'), '/" + "', timestamp, '"']))
    
    # 注册事件
    start_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=record_process,
            on_start=[start_log]
        )
    )
    
    exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=record_process,
            on_exit=[exit_log]
        )
    )
    
    # ==================== 返回Launch描述 ====================
    return LaunchDescription([
        enable_recording,
        output_dir,
        topics,
        duration,
        max_size,
        storage_format,
        record_process,
        start_handler,
        exit_handler
    ])