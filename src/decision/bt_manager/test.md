任务目标：云台战术区域化巡检功能开发 (Gimbal Scan Strategy by Area)1. 需求背景当前哨兵云台（Gimbal）的巡检策略需要细化。我们需要根据哨兵所处的战术模式（TacticalMode）以及当前所在的物理区域（Nav Zones），动态下发不同的云台扫描参数 {scan_yaw_min, scan_yaw_max}。 如果哨兵位于特定防守或高地等区域内，需启用偏航角范围限制（use_limited_scan = true），否则默认进行全范围环视（use_limited_scan = false）。2. 核心数据结构修改 (area.hpp)请在 area.hpp 中新增区域类型枚举，并将原有的 tactical_gimbal_map 替换为两级 unordered_map：// 云台单区域扫描配置
struct GimbalPatrolConfig {
  float scan_yaw_min;
  float scan_yaw_max;
};

// 巡检区域枚举
enum class PatrolZoneType {
  ENEMY_DEFENSE,
  OWN_DEFENSE,
  HIGHLAND
};

// 云台巡检区域映射表 (TacticalMode -> (PatrolZoneType -> GimbalPatrolConfig))
inline std::unordered_map<TacticalMode, std::unordered_map<PatrolZoneType, GimbalPatrolConfig>> tactical_area_gimbal_map = {
  {TacticalMode::OFFENSIVE, {
    {PatrolZoneType::ENEMY_DEFENSE, {-30.0f, 30.0f}},   // 敌方防区
    {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}}, // 己方防区
    {PatrolZoneType::HIGHLAND,      {-90.0f, 90.0f}}    // 高地区域
  }},
  {TacticalMode::DEFENSIVE, {
    {PatrolZoneType::ENEMY_DEFENSE, {-180.0f, 180.0f}}, // 敌方防区
    {PatrolZoneType::OWN_DEFENSE,   {90.0f, 180.0f}},   // 己方防区
    {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}}  // 高地区域
  }},
  {TacticalMode::BALANCED, {
    {PatrolZoneType::ENEMY_DEFENSE, {-90.0f, 90.0f}},   // 敌方防区
    {PatrolZoneType::OWN_DEFENSE,   {-180.0f, 180.0f}}, // 己方防区
    {PatrolZoneType::HIGHLAND,      {-180.0f, 180.0f}}  // 高地区域
  }}
};
3. 行为树节点实现 (gimbal_action.cpp / 相关 Action 头文件)在负责执行云台巡检的 Action 节点（如 SetGimbalPoseByAreaAction）的 tick() 方法中，实现以下业务逻辑：获取基础数据:// 注意黑板中的 key 是 "tactical_mode" 而不是 "current_tactical_mode"
auto mode = blackboard->get<TacticalMode>("tactical_mode"); 
auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");
区域判定 (Contain 判断):Point2D current_pt{current_pose.position.x, current_pose.position.y};
bool is_in_zone = false;
PatrolZoneType target_zone;

if (Sentry_BT::enemy_defense_zone.contains(current_pt)) { 
    target_zone = PatrolZoneType::ENEMY_DEFENSE; 
    is_in_zone = true;
} else if (Sentry_BT::own_defense_zone.contains(current_pt)) { 
    target_zone = PatrolZoneType::OWN_DEFENSE; 
    is_in_zone = true;
} else if (Sentry_BT::highland_zone.contains(current_pt)) { 
    target_zone = PatrolZoneType::HIGHLAND; 
    is_in_zone = true;
}
查表与黑板写入:if (is_in_zone) {
    // 命中区域：使用配置参数，启用 yaw 限制
    auto config = Sentry_BT::tactical_area_gimbal_map[mode][target_zone];
    blackboard->set("scan_yaw_min_deg", config.scan_yaw_min);
    blackboard->set("scan_yaw_max_deg", config.scan_yaw_max);
    blackboard->set("use_limited_scan", true);
} else {
    // 默认处理：全范围环视，不限制 yaw
    blackboard->set("scan_yaw_min_deg", -180.0f);
    blackboard->set("scan_yaw_max_deg", 180.0f);
    blackboard->set("use_limited_scan", false);
}
4. 接口与数据流检查清单 (Checklist)开发完成后，请严格按照以下清单检查数据流，确保 use_limited_scan 能够正确贯穿从黑板到下位机的全链路：[ ] 黑板初始化 (blackboard.hpp):在 Blackboard::Blackboard() 的 // --- Gimbal Tree --- 注释块下，确保初始化了 use_limited_scan 变量：blackboard_->set("use_limited_scan", false); // <--- 必须新增
blackboard_->set("scan_yaw_min_deg", -180.0f);
// ...
[ ] ROS Msg 定义检查:前往 ros_interfaces/msg/Behavior.msg，检查是否包含 bool use_limited_scan。如果没有，必须添加并重新编译消息包 colcon build --packages-select ros_interfaces。[ ] 接口层转发 (ros_interface.cpp):在 ros_interface.cpp 的 10Hz timer_ 回调函数中，读取并赋值 use_limited_scan：// 读取
const auto yaw_max_deg = blackboard_->get<float>("scan_yaw_max_deg");
const auto use_limited_scan = blackboard_->get<bool>("use_limited_scan"); // <--- 新增读取

// ... 赋值给 msg
ros_interfaces::msg::Behavior behavior_msg;
behavior_msg.use_limited_scan = use_limited_scan; // <--- 新增赋值
behavior_msg.scan_yaw_min = yaw_min_deg;
// ...
[ ] Communication / 下位机解包逻辑检查:检查与下位机通信的 communication 节点或 CAN/串口驱动代码。确认在订阅 /sentry/behaivor_send (Behavior.msg) 并打包数据帧发送给下位机时，将 use_limited_scan 包含在内。确认下位机（STM32 等）固件中的云台控制逻辑：当 use_limited_scan == false 时执行无死区的 360° 环视；当 use_limited_scan == true 时，云台运动严格限制在 yaw_min 和 yaw_max 之间。