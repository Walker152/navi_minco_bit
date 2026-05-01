#!/bin/bash

# ==========================================
# 加固版 PTP 同步脚本 (解决环境变量与挂起问题)
# ==========================================

INTERFACE="enp86s0"

# 1. 权限校验
if [ "$EUID" -ne 0 ]; then
  echo "❌ 权限错误: 必须使用 sudo 运行！"
  exit 1
fi

# 2. 自动寻找命令的绝对路径 (解决 sudo 环境变量丢失问题)
PTP4L_BIN=$(which ptp4l || echo "/usr/sbin/ptp4l")
PHC2SYS_BIN=$(which phc2sys || echo "/usr/sbin/phc2sys")

if [ ! -x "$PTP4L_BIN" ]; then
    echo "❌ 致命错误: 找不到 ptp4l 可执行文件，请确认 linuxptp 已安装。"
    exit 1
fi

# 3. 强力清理旧进程
killall -9 ptp4l phc2sys 2>/dev/null
sleep 1

# 4. 初始化日志文件 (确保即使无输出也会创建空文件)
PTP4L_LOG="/tmp/ptp4l_${INTERFACE}.log"
PHC2SYS_LOG="/tmp/phc2sys_${INTERFACE}.log"
touch $PTP4L_LOG $PHC2SYS_LOG
chmod 666 $PTP4L_LOG $PHC2SYS_LOG

echo "========================================"
echo "🚀 正在启动硬件级 PTP Master 广播服务"
echo "🌐 网卡接口: $INTERFACE"
echo "========================================"

# 5. 去除 nohup，直接使用后台符 &，并将日志实时刷入文件
# 使用 stdbuf -oL 强制按行刷新输出缓冲区，防止日志被系统吞掉
stdbuf -oL $PTP4L_BIN -i $INTERFACE -m -l 6 > $PTP4L_LOG 2>&1 &
PTP4L_PID=$!
echo "✅ ptp4l 已启动 (PID: $PTP4L_PID) | 日志: $PTP4L_LOG"

stdbuf -oL $PHC2SYS_BIN -c $INTERFACE -s CLOCK_REALTIME -O 0 > $PHC2SYS_LOG 2>&1 &
PHC2SYS_PID=$!
echo "✅ phc2sys 已启动 (PID: $PHC2SYS_PID) | 日志: $PHC2SYS_LOG"

echo "========================================"
echo "💡 提示: 脚本已挂起，按下 Ctrl+C 停止服务。"
echo "========================================"

# 6. 守护与退出机制
trap "echo -e '\n🛑 正在终止 PTP 进程...'; kill -9 $PTP4L_PID $PHC2SYS_PID 2>/dev/null; echo '退出成功。'; exit 0" SIGINT SIGTERM

wait