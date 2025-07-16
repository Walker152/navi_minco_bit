# #!/bin/bash

# # 终止所有相关进程
# pkill -9 -f 'gazebo'
# pkill -9 -f 'gzserver'
# pkill -9 -f 'gzclient'
# pkill -9 -f 'ros2 master'
# pkill -9 -f 'rosmaster'
# pkill -9 -f 'spawn_entity'

# # 释放端口
# for port in 11345 11311; do
#   lsof -i :$port | awk 'NR!=1 {print $2}' | xargs kill -9 2>/dev/null
# done

# # 删除残留文件
# rm -rf ~/.ros/log/* /tmp/gazebo* /tmp/ros*
# # rm -rf ~/.gazebo/* ~/.ros/log/* /tmp/gazebo* /tmp/ros*

# echo "Gazebo和相关进程已强制清除"

#!/bin/bash

# 添加错误处理
set -e  # 遇到错误立即退出
trap 'echo "发生错误，清理过程中断"' ERR

# 定义清理函数
cleanup_processes() {
    local processes=('gazebo' 'gzserver' 'gzclient' 'ros2 master' 'rosmaster' 'spawn_entity')
    
    for proc in "${processes[@]}"; do
        if pgrep -f "$proc" > /dev/null; then
            echo "正在终止 $proc..."
            pkill -9 -f "$proc" || true
        fi
    done
}

cleanup_ports() {
    local ports=(11345 11311)
    
    for port in "${ports[@]}"; do
        if lsof -i ":$port" >/dev/null 2>&1; then
            echo "正在释放端口 $port..."
            lsof -i ":$port" | awk 'NR!=1 {print $2}' | xargs -r kill -9 2>/dev/null || true
        fi
    done
}

cleanup_files() {
    local paths=(
        "$HOME/.ros/log"
        "/tmp/gazebo*"
        "/tmp/ros*"
    )
    
    for path in "${paths[@]}"; do
        if ls $path >/dev/null 2>&1; then
            echo "正在删除 $path..."
            rm -rf $path || true
        fi
    done
}

# 主执行流程
echo "开始清理进程..."
cleanup_processes

echo "开始清理端口..."
cleanup_ports

echo "开始清理文件..."
cleanup_files

echo "清理完成！"