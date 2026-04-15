#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
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
    auto blackboard = config().blackboard;
    auto health_threshold_ = getInput<float>("health_threshold");
    auto recovery_threshold_ = getInput<float>("recovery_threshold");
    if(!health_threshold_ || !recovery_threshold_)
    {
      throw BT::RuntimeError("missing required input [health_threshold] or [recovery_threshold]");
    }
    float health_threshold = health_threshold_.value();
    float recovery_threshold = recovery_threshold_.value();
    auto health = blackboard->get<float>("health");
    auto current_mode = blackboard->get<int>("current_mode");

    BT::NodeStatus result = BT::NodeStatus::FAILURE;
    if(current_mode == Sentry_BT::NavMode::RETREAT)
    {
      if(health >= recovery_threshold)
      {
        blackboard->set<int>("current_mode", Sentry_BT::NavMode::PATROL);
        result = BT::NodeStatus::FAILURE;
      }
      else
      {
        result = BT::NodeStatus::SUCCESS;
      }
    }
    else if(health < health_threshold)
    {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::RETREAT);
      result = BT::NodeStatus::SUCCESS;
    }

    static BT::NodeStatus last_result = BT::NodeStatus::IDLE;
    if(result != last_result)
    {
      std::cout << WHITE << "CheckRetreatCondition => "
                << (result == BT::NodeStatus::SUCCESS ? "RETREAT_ACTIVE" : "RETREAT_INACTIVE")
                << ", health=" << health
                << ", threshold=" << health_threshold
                << ", recovery=" << recovery_threshold << RESET << std::endl;
      last_result = result;
    }

    return result;
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
    auto blackboard = config().blackboard;

    static bool last_condition_met = false;
    static int tick_count = 0;
    static std::chrono::time_point<std::chrono::system_clock> last_seen_time =
        std::chrono::system_clock::now();

    bool target_valid = false;
    try {
      target_valid = blackboard->get<bool>("target_valid");
    } catch(...) {
      if(last_condition_met)
      {
        std::cout << YELLOW << "CheckTargetLocked => UNLOCKED (target_valid unavailable)" << RESET
                  << std::endl;
        last_condition_met = false;
      }
      return BT::NodeStatus::FAILURE;
    }

    try {
      target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    } catch(...) {
      if(last_condition_met)
      {
        std::cout << YELLOW << "CheckTargetLocked => UNLOCKED (target_pose unavailable)" << RESET
                  << std::endl;
        last_condition_met = false;
      }
      return BT::NodeStatus::FAILURE;
    }

    //rmuc
    // Sentry_BT::Area_Square highland_area = {{6.7, 2.0}, {13.0, -1.8}};
    // Sentry_BT::Area_Square enemy_outpost_area = {{8.5, 4.5}, {11.5, 2.8}};
    // Sentry_BT::Area_Square own_outpost_area = {{8.5, -2.7}, {11.5, -4.2}};  //待修改

    //rmul
    Sentry_BT::Area_Square attack_area = {{7.8, 7.4}, {4.5, 2.0}};
    
    const bool in_attack_area = attack_area.contains({target_pose.position.x, target_pose.position.y});
    bool condition_met = false;

    if(in_attack_area && target_valid)
    {
      last_seen_time = std::chrono::system_clock::now();
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
      condition_met = true;
      tick_count = 0;
    }
    else if(in_attack_area)
    {
      auto now = std::chrono::system_clock::now();
      double lost_duration = std::chrono::duration<double>(now - last_seen_time).count();

      // 容忍 1.0 秒内的视觉丢失
      if(lost_duration < 1.0)
      {
        tick_count++;
        if(tick_count % 5 == 0)
        {
          std::cout << YELLOW << "[Debounce] Target visually lost, keeping lock for "
                    << (1.0 - lost_duration) << "s" << RESET << std::endl;
        }
        blackboard->set<int>("current_mode", Sentry_BT::NavMode::TRACING);
        condition_met = true;
      }
      else
      {
        tick_count = 0;
      }
    }
    else
    {
      tick_count = 0;
    }

    if(condition_met != last_condition_met)
    {
      if(condition_met)
      {
        std::cout << GREEN << "CheckTargetLocked => LOCKED"
                  << ", target_xy=(" << target_pose.position.x << ", " << target_pose.position.y
                  << ")" << RESET << std::endl;
      }
      else
      {
        std::cout << YELLOW << "CheckTargetLocked => UNLOCKED"
                  << (in_attack_area ? "" : " (out of attack area)") << RESET << std::endl;
      }
      last_condition_met = condition_met;
    }

    return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
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
    auto blackboard = config().blackboard;

    auto enemy_outpost_destroyed = blackboard->get<bool>("enemy_outpost_destroyed");
    const bool outpost_remained = !enemy_outpost_destroyed;

    static bool last_outpost_remained = !outpost_remained;
    if(outpost_remained != last_outpost_remained)
    {
      std::cout << BLUE << "CheckOutpostRemained => "
                << (outpost_remained ? "OUTPOST_REMAINED" : "OUTPOST_DESTROYED")
                << RESET << std::endl;
      last_outpost_remained = outpost_remained;
    }

    // 如果前哨站还在，切换到响应模式
    if(outpost_remained)
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
  auto blackboard = config().blackboard;

  std::shared_ptr<ros_interface> ros_iface;
  geometry_msgs::msg::Pose current_pose;
  ros_iface = blackboard->get<std::shared_ptr<ros_interface>>("ros_interface");
  if(!ros_iface){
    return BT::NodeStatus::FAILURE;
  }
  current_pose = ros_iface->getCurrentPose();

  double x = current_pose.position.x;
  double y = current_pose.position.y;

  bool in_stairs_zone = (x > 3.2 && x < 6.0 && y > -6.4 && y < -5.3);

  static bool last_in_stairs_zone = false;
  if(in_stairs_zone != last_in_stairs_zone)
  {
    std::cout << WHITE << "CheckInStairsZone => "
              << (in_stairs_zone ? "IN_ZONE" : "OUT_OF_ZONE")
              << ", pos=(" << x << ", " << y << ")" << RESET << std::endl;
    last_in_stairs_zone = in_stairs_zone;
  }

  return in_stairs_zone ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// --------------------- CheckWillThroughTunnel ----------------------
CheckWillThroughTunnel::CheckWillThroughTunnel(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
{
  // 构造函数：初始化节点，不需要复杂操作 
}

BT::PortsList CheckWillThroughTunnel::providedPorts()
{
  return {}; // 不需要输入端口，直接从黑板读取信息
}

BT::NodeStatus CheckWillThroughTunnel::tick()
{
  auto blackboard = config().blackboard;
  bool will_through_tunnel = false;
  auto lifter_current_pos = blackboard->get<int>("lifter_current_pos");
  will_through_tunnel = blackboard->get<bool>("through_tunnel");
  static bool last_state_ = !will_through_tunnel;
  if (last_state_ != will_through_tunnel)
  {
    std::cout << WHITE << "CheckWillThroughTunnel => "
            << (will_through_tunnel ? "WILL_THROUGH_TUNNEL" : "WILL_NOT_THROUGH_TUNNEL")
            << RESET << std::endl;
    last_state_ = will_through_tunnel;
  }
  
  if (will_through_tunnel) {
    if (lifter_current_pos == 0) {
      blackboard->set<int>("desired_lifter_pos", 1); // 设置目标升降位置为 1(bottom)，准备过隧道
    }
  }
  else {
    if (lifter_current_pos != 0) {
      blackboard->set<int>("desired_lifter_pos", 0); // 设置目标升降位置为 0(top)，准备不通过隧道
    }
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace Sentry_BT