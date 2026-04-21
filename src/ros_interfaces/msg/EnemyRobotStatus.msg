# 机器人装甲板ID常量定义
uint8 HERO = 1           # 英雄机器人
uint8 ENGINEER = 2       # 工程机器人
uint8 INFANTRY_3 = 3     # 3号步兵机器人
uint8 INFANTRY_4 = 4     # 4号步兵机器人

# 机器人的基本状态信息
uint8 robot_id             # 机器人ID，蓝方=红方+100
uint16 robot_hp            # 机器人血量
uint16 allowed_projectile  # 允许发射的弹丸数量，单位发

# 机器人位置信息
geometry_msgs/Pose position  # 位置坐标