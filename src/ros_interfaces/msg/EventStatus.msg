std_msgs/Header header

bool own_outpost_destroyed              # 自方前哨站是否被摧毁
bool buff_active                        # 是否处于能量机关增益状态
bool game_status                        # 比赛状态, True为比赛进行中

float32 self_health                     # 自身血量
float32 gimbal_yaw                      # 云台偏航角

int16 enemy_outpost_health              # 敌方前哨站血量 

ros_interfaces/Detector enemy_detected   # 敌方检测信息

int32 position                          # 新增的姿态字段