std_msgs/Header header

ros_interfaces/EnemyRobotStatus[6] detectors  # 雷达检测到的机器人信息列表，最多6个机器人
uint16 enemy_coin_left              # 敌方剩余金币数量
uint16 enemy_coin_accumulated       # 敌方累计获得金币数量
bool is_enemy_outpost_sensed        # 敌方前哨站是否被雷达识别到