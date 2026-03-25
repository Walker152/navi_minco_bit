std_msgs/Header header

uint16 self_health
uint16 num_shoot
uint16 own_outpost_health        # 自方前哨站血量

bool enemy_outpost_destroyed
bool buff_active                        # 是否处于能量机关增益状态                        # 比赛状态, True为比赛进行中

float32 gimbal_yaw                      # 云台偏航角

ros_interfaces/Detector enemy_detected   # 敌方检测信息

uint8 game_status
uint8 current_stance
uint8 lifter_pos_now

uint16 hero_health
uint16 infantry3_health