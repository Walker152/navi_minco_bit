#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
using namespace color_text;
namespace Sentry_BT
{
  // ------------------- CheckRetreatCondition -------------------
  CheckRetreatCondition::CheckRetreatCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckRetreatCondition::providedPorts()
  {
    return {
        BT::InputPort<float>("health_threshold"),
        BT::InputPort<float>("recovery_threshold"),
    };
  }

  BT::NodeStatus CheckRetreatCondition::tick()
  { 
    std::cout << BLUE << "---------- CheckRetreatCondition ----------" << RESET << std::endl;
    auto blackboard = config().blackboard;
    // 从XML获取健康阈值和恢复阈值
    auto health_threshold_ = getInput<float>("health_threshold");
    auto recovery_threshold_ = getInput<float>("recovery_threshold");
    if(!health_threshold_ || !recovery_threshold_)
    {
      throw BT::RuntimeError("missing required input [health_threshold] or [recovery_threshold]");
    }
    float health_threshold = health_threshold_.value();
    float recovery_threshold = recovery_threshold_.value();
    std::cout << WHITE << "health_threshold:" << health_threshold << "%" << RESET << std::endl;
    auto health = blackboard->get<float>("health");
    auto current_mode = blackboard->get<int>("current_mode");
    std::cout << WHITE << "Current mode: " << mode_names[current_mode] << ", Health: " << health << "%" << RESET << std::endl;
    // 如果已经在撤退模式，检查是否应该继续撤退
    if(current_mode == Sentry_BT::NavMode::RETREAT)
    {
      // 只有当血量恢复到安全水平才退出撤退
      if(health >= recovery_threshold)
      {
        blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
        return BT::NodeStatus::FAILURE;  // 退出撤退
      }
      return BT::NodeStatus::SUCCESS;  // 继续撤退
    }
    // 如果不在撤退模式，检查是否需要进入撤退
    else if(health < health_threshold)
    {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::RETREAT);
      return BT::NodeStatus::SUCCESS;  // 进入撤退
    }

    return BT::NodeStatus::FAILURE;  // 不需要撤退
  }

  // ------------------- CheckTargetLocked -------------------
  CheckTargetLocked::CheckTargetLocked(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckTargetLocked::providedPorts()
  {
    return {
        BT::InputPort<bool>("target_valid"),
    };
  }

  BT::NodeStatus CheckTargetLocked::tick()
  {
    std::cout << BLUE << "---------- CheckTargetLocked ----------" << RESET << std::endl;
    auto blackboard = config().blackboard;

    bool target_valid = false;
    try {
      target_valid = blackboard->get<bool>("target_valid");
    } catch(...) {
      return BT::NodeStatus::FAILURE;
    }

    std::cout << WHITE << "Target valid: " << (target_valid ? "true" : "false") << RESET << std::endl;


    //检查目标是否处于可追击范围
    try {
      target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    } catch(...) {
      return BT::NodeStatus::FAILURE;
    }

    //rmuc
    // Sentry_BT::Area_Square highland_area = {{6.7, 2.0}, {13.0, -1.8}};
    // Sentry_BT::Area_Square enemy_outpost_area = {{8.5, 4.5}, {11.5, 2.8}};
    // Sentry_BT::Area_Square own_outpost_area = {{8.5, -2.7}, {11.5, -4.2}};  //待修改

    //rmul
    Sentry_BT::Area_Square attack_area = {{7.0, 7.0}, {0.0, 0.0}};
    
    if(!attack_area.contains({target_pose.position.x, target_pose.position.y}))
    {
      std::cout << "Target out of effective area: (" << target_pose.position.x << ", " << target_pose.position.y << ")"
              << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    // 使用静态变量记录最后一次看到敌人的时间
    static std::chrono::time_point<std::chrono::system_clock> last_seen_time = std::chrono::system_clock::now();

    if(target_valid)
    {
      last_seen_time = std::chrono::system_clock::now();
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
      return BT::NodeStatus::SUCCESS;
    }
    else {
      auto now = std::chrono::system_clock::now();
      double lost_duration = std::chrono::duration<double>(now - last_seen_time).count();

      // 容忍 1.0 秒内的视觉丢失
      if(lost_duration < 1.0)
      {
        std::cout << YELLOW << "[Debounce] Target visually lost, but keeping lock for " 
                  << (1.0 - lost_duration) << "s" << RESET << std::endl;
        // 保持追击模式，返回假成功
        blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
        return BT::NodeStatus::SUCCESS; 
      }
    }
    return BT::NodeStatus::FAILURE;
  }

  // ------------------- CheckOutpostRemained -------------------
  CheckOutpostRemained::CheckOutpostRemained(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckOutpostRemained::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckOutpostRemained::tick()
  {
    std::cout << BLUE << "---------- CheckOutpostRemained ----------" << RESET << std::endl;
    auto blackboard = config().blackboard;

    auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
    std::cout << WHITE << "Enemy outpost destroyed: " << (enemy_outpost_destroyed ? "true" : "false") << RESET << std::endl;
    // 如果前哨站还在，切换到响应模式
    if(!enemy_outpost_destroyed)
    {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::RESPONSE);
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  // --------------------- CheckInStairsZone ----------------------
CheckInStairsZone::CheckInStairsZone(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
  // 构造函数：初始化节点，不需要复杂操作
}

BT::PortsList CheckInStairsZone::providedPorts()
{
  return {}; // 不需要输入端口，直接从黑板读取位置
}

BT::NodeStatus CheckInStairsZone::tick()
{
  // 1. 获取黑板对象
  auto blackboard = config().blackboard;
  std::cout << BLUE << "---------- CheckInStairsZone ----------" << RESET << std::endl;
  // 2. 从黑板读取ros接口并获取当前位置
  std::shared_ptr<ros_interface> ros_iface;
  geometry_msgs::msg::Pose current_pose;
  ros_iface = blackboard->get<std::shared_ptr<ros_interface>>("ros_interface");
  if(!ros_iface){
    return BT::NodeStatus::FAILURE;
  }
  current_pose = ros_iface->getCurrentPose();
  
  // 3. 提取坐标
  double x = current_pose.position.x;
  double y = current_pose.position.y;

  // 4. 定义台阶区域（根据实际场地调整）
   bool in_stairs_zone = (x > 3.2 && x < 6.0 && y > -6.4 && y < -5.3);
  std::cout<<"Current Position: ("<<x<<", "<<y<<")"<<std::endl;
  // 5. 判断是否在台阶相关区域
  if (in_stairs_zone) {
    return BT::NodeStatus::SUCCESS; // 在台阶区域，可以执行撤离
  }
  
  return BT::NodeStatus::FAILURE; // 不在台阶区域
}
}  // namespace Sentry_BT