std_msgs/Header header

uint16 game_time_remaining  # 比赛剩余时间，单位s
uint16 coin_remaining       # 己方剩余金币数量
uint32 event_code           # 场地事件代码，未解码，需接收后根据协议解码
uint8 game_status           # 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中
float32 manual_point_x      # 手动指定的目标点x坐标，单位m，坐标系minimap
float32 manual_point_y      # 手动指定的目标点y坐标，单位m，坐标系minimap
uint8 manual_key            # 手动指定的目标点快捷
uint16 enemy_outpost_hp     # 敌方前哨站血量
uint16 enemy_base_hp        # 敌方基地血量
