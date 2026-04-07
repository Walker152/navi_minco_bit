std_msgs/Header header

uint16 self_health        # 机器人血量
uint16 bullets_remaining  # 剩余发弹量
uint16 cooling_value      # 机器人每秒冷却值
uint16 heat_limit         # 机器人热量上限
uint16 current_heat       # 机器人当前热量
geometry_msgs/Point sentry_pos         # 哨兵位置
float32 speed_monitor_angle   # 测速模块朝向，单位为度，正北为0度
uint32 sentry_info_1      # 哨兵信息1，未解码，需接收后根据协议解码
uint16 sentry_info_2      # 哨兵信息2，未解码，需接收后根据协议解码