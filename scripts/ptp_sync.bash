#!/bin/bash

# Mid-360 硬件级 PTP 时间同步启动脚本

INTERFACE="enp86s0"

if [ "$EUID" -ne 0 ]; then
  echo "❌ 权限错误: 请使用 sudo 运行此脚本。"
  exit 1
fi

echo "🧹 正在清理系统中的旧 PTP 进程..."
killall -q ptp4l phc2sys
sleep 1

PTP4L_LOG="../tmp/ptp4l_${INTERFACE}.log"
PHC2SYS_LOG="../tmp/phc2sys_${INTERFACE}.log"

echo "========================================"
echo "🚀 正在启动硬件级 PTP Master 广播服务"
echo "🌐 网卡接口: $INTERFACE"
echo "📄 ptp4l 日志: $PTP4L_LOG"
echo "📄 phc2sys 日志: $PHC2SYS_LOG"
echo "========================================"

nohup ptp4l -i $INTERFACE -m -l 6 > $PTP4L_LOG 2>&1 &
PTP4L_PID=$!
echo "✅ [1/2] ptp4l 已拉起，进程 PID: $PTP4L_PID"

nohup phc2sys -c $INTERFACE -s CLOCK_REALTIME -O 0 > $PHC2SYS_LOG 2>&1 &
PHC2SYS_PID=$!
echo "✅ [2/2] phc2sys 已拉起，进程 PID: $PHC2SYS_PID"

echo "========================================"
echo "💡 提示:"
echo "实时查看雷达同步状态: tail -f $PTP4L_LOG"
echo "按下 Ctrl+C 停止服务并退出。"
echo "========================================"

trap "echo -e '\n🛑 捕获到退出信号，正在安全终止 PTP 进程...'; kill $PTP4L_PID $PHC2SYS_PID; echo '已退出。'; exit 0" SIGINT SIGTERM

wait