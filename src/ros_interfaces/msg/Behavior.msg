uint8 desired_stance
uint8 STANCE_ATTACK=1
uint8 STANCE_DEFEND=2
uint8 STANCE_MOVE=3
uint8 STANCE_ENHANCED_ATTACK = 4
uint8 STANCE_ENHANCED_DEFEND = 5
uint8 STANCE_ENHANCED_MOVE = 6

uint8 control_mode
uint8 CONTROL_AUTO=0
uint8 CONTROL_MANUAL=1

bool use_gyro_mode
float32 gyro_vel

uint8 desire_lifter_pos
uint8 LIFTER_TOP=0
uint8 LIFTER_BOTTOM=1
uint8 LIFTER_MIDDLE=2

uint16 ammo_purchase_request

uint8 revive_request
uint8 remote_revive_request
uint8 remote_ammo_request
uint8 remote_health_request

bool use_limited_scan
float32 scan_yaw_min
float32 scan_yaw_max

uint8 pitch_mode
uint8 PITCH_TOP=0
uint8 PITCH_BOTTOM=1

bool not_aim_enemy
bool use_capacitor

bool tunnel_escape_active
float32 tunnel_escape_vx
float32 tunnel_escape_vy
float32 tunnel_escape_wz

bool tunnel_prepare_active
bool tunnel_ready

bool tunnel_align_active
float32 tunnel_align_angle_deg
