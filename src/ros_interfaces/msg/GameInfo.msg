std_msgs/Header header

uint16 game_time_remaining  # 比赛剩余时间，单位s
uint16 coin_remaining       # 己方剩余金币数量
uint32 event_code           # 场地事件代码，未解码，需接收后根据协议解码
uint8 game_status           # 0:未开始比赛 1:准备阶段 2:15s裁判系统自检 3:5s倒计时 4:比赛中 5:比赛结算中