std_msgs/Header header

bool is_get                      # 视觉是否瞄准到敌人
geometry_msgs/Point armor_pos    # 瞄准到的装甲板位置，具体坐标系询问视觉
uint8 armor_num                  # 不做红蓝方区分
float32 yaw_encoder              # 编码器的yaw轴角度 逆时针为正
float32 yaw_imu                  # imu的yaw轴角度 逆时针为正
uint8 lifter_current_pos         # 0 -- kTop 1 -- kBottom 2 -- kMiddle
bool is_transformable            # 是否能够进行变形（不止变形中不能变形，升降卡住后也无法进行变形）
float32 transform_state          # 变形状态，0-1，0%为未变形，100%为完全变形，过渡状态根据实际情况变化
uint8 capacitor_capacity         # 电容容量百分比