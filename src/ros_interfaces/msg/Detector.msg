std_msgs/Header header
bool is_detect                # 是否被哨兵 or Radar 检测到

# 机器人的基本状态信息
uint16 remain_hp              # 剩余血量
uint16 max_hp                 # 最大血量
uint16 armor_id               # 装甲板ID

# 机器人位置信息
geometry_msgs/Point position  # 位置坐标
