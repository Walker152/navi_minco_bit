#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/blackboard.hpp"
#include <iostream>
#include <string>

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
    std::cout << "---------- CheckRetreatCondition ----------" << std::endl;
    auto blackboard = config().blackboard;
    //
    auto my_position = blackboard->get<int>("my_position");
    std::cout << "当前姿态：" << my_position  << std::endl;
    //
    // 从XML获取健康阈值和恢复阈值
    auto health_threshold_ = getInput<float>("health_threshold");
    auto recovery_threshold_ = getInput<float>("recovery_threshold");
    if(!health_threshold_ || !recovery_threshold_)
    {
      throw BT::RuntimeError("missing required input [health_threshold] or [recovery_threshold]");
    }
    float health_threshold = health_threshold_.value();
    float recovery_threshold = recovery_threshold_.value();
    std::cout << "health_threshold" << health_threshold << std::endl;
    auto health = blackboard->get<float>("health");
    auto current_mode = blackboard->get<int>("current_mode");
    std::cout << "Current mode: " << current_mode << ", Health: " << health << std::endl;
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
    std::cout << "---------- CheckTargetLocked ----------" << std::endl;
    auto blackboard = config().blackboard;

    auto target_armor_id = blackboard->get<int>("target_armor_id");
    auto target_pose = blackboard->get<geometry_msgs::msg::Pose>("target_pose");
    auto target_valid = blackboard->get<bool>("target_valid");
    std::cout << "Target armor ID: " << target_armor_id << std::endl;
    std::cout << "Target position: (" << target_pose.position.x << ", " << target_pose.position.y << ", "
              << target_pose.position.z << ")" << std::endl;

    // 检查目标是否有效
    if(!target_valid)
    {
      return BT::NodeStatus::FAILURE;
    }
    if(target_armor_id == Sentry_BT::RobotID::Engineer)
    {
      return BT::NodeStatus::FAILURE;
    }

    Sentry_BT::Area_Square highland_area = {{6.7, 2.0}, {13.0, -1.8}};
    Sentry_BT::Area_Square enemy_outpost_area = {{8.5, 4.5}, {11.5, 2.8}};
    Sentry_BT::Area_Square own_outpost_area = {{8.5, -2.7}, {11.5, -4.2}};  //待修改

    if(highland_area.contains({target_pose.position.x, target_pose.position.y}) ||
       enemy_outpost_area.contains({target_pose.position.x, target_pose.position.y}) ||
       own_outpost_area.contains({target_pose.position.x, target_pose.position.y}))
    {
      blackboard->set<bool>("target_valid", false);
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::ATTACK);
      return BT::NodeStatus::SUCCESS;
    }
    std::cout << "Target out of effective area: (" << target_pose.position.x << ", " << target_pose.position.y << ")"
              << std::endl;
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
    std::cout << "---------- CheckOutpostRemained ----------" << std::endl;
    auto blackboard = config().blackboard;

    auto enemy_outpost_health = blackboard->get<int>("enemy_outpost_health");
    std::cout << "Enemy outpost health: " << enemy_outpost_health << std::endl;
    // 如果前哨站还在，切换到响应模式
    if(enemy_outpost_health > 0)
    {
      blackboard->set<int>("current_mode", Sentry_BT::NavMode::RESPONSE);
      std::cout << "Enemy outpost health: " << enemy_outpost_health << std::endl;
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  // ------------------- CheckNavStatus -------------------
  CheckNavStatus::CheckNavStatus(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckNavStatus::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckNavStatus::tick()
  {
    std::cout << "---------- CheckNavStatus ----------" << std::endl;
    auto blackboard = config().blackboard;
    // 从黑板获取导航状态
    auto nav_status = blackboard->get<int>("nav_status");

    std::cout << "Current navigation status: " << current_nav_status[nav_status] << std::endl;
    // 只有当导航空闲或失败时，才允许选择新的巡逻点
    if(nav_status == Sentry_BT::NavStatus::IDLE || nav_status == Sentry_BT::NavStatus::FAILURE)
    {
      return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
  }

  // ------------------- CheckIfRetreating -------------------
  CheckIfRetreating::CheckIfRetreating(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckIfRetreating::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckIfRetreating::tick()
  {
    std::cout << "---------- CheckIfRetreating ----------" << std::endl;
    auto blackboard = config().blackboard;

    // 从黑板获取当前模式
    auto current_mode = blackboard->get<int>("current_mode");

    std::cout << "Current mode: " << mode_names[current_mode] << std::endl;

    // 检查当前模式是否为撤退模式
    return (current_mode == Sentry_BT::NavMode::RETREAT) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

    // ------------------- CheckMPCondition -------------------
  CheckMPCondition::CheckMPCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckMPCondition::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckMPCondition::tick()
  {
    std::cout << "---------- CheckMPCondition ----------" << std::endl;
    auto blackboard = config().blackboard;
    auto enemy_outpost_health = blackboard->get<int>("enemy_outpost_health");
    auto current_mode = blackboard->get<int>("current_mode");
    auto nav_status = blackboard->get<int>("nav_status");
    auto outpost_msg = blackboard->get<bool>("outpost_msg");
    auto my_position = blackboard->get<int>("my_position");
    if(((enemy_outpost_health > 0) && (current_mode == Sentry_BT::NavMode::RESPONSE) && (outpost_msg == false)) || 
       ((current_mode == Sentry_BT::NavMode::RETREAT) && (nav_status == Sentry_BT::NavStatus::RUNNING)) ||
       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::RUNNING))     
      )
    {
      blackboard->set("want_position", 1); 
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

// ------------------- CheckAPCondition -------------------
  CheckAPCondition::CheckAPCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }
  
  BT::PortsList CheckAPCondition::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckAPCondition::tick()
  {
    std::cout << "---------- CheckAPCondition ----------" << std::endl;
    auto blackboard = config().blackboard;
    auto my_position = blackboard->get<int>("my_position");
    auto enemy_outpost_health = blackboard->get<int>("enemy_outpost_health");
    auto current_mode = blackboard->get<int>("current_mode");
    auto nav_status = blackboard->get<int>("nav_status");
    auto outpost_msg = blackboard->get<bool>("outpost_msg");
    auto current_health = blackboard->get<float>("health");
    if(((enemy_outpost_health > 0) && (current_mode == Sentry_BT::NavMode::RESPONSE) && (outpost_msg == true)) ||
       (current_mode == Sentry_BT::NavMode::ATTACK) ||
       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::IDLE)) ||
       ((current_mode == Sentry_BT::NavMode::PATROL) && (nav_status == Sentry_BT::NavStatus::SUCCESS))
      )
    {
      blackboard->set("want_position", 2); // attack
      std::cout << "now want_position = 2" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;                  
  }

// ------------------- CheckDPCondition -------------------
  CheckDPCondition::CheckDPCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckDPCondition::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckDPCondition::tick()
  {
    std::cout << "---------- CheckDPCondition ----------" << std::endl;
    auto blackboard = config().blackboard;

    auto my_position = blackboard->get<int>("my_position");     
    auto current_mode = blackboard->get<int>("current_mode");
    auto nav_status = blackboard->get<int>("nav_status");
    auto current_health = blackboard->get<float>("health");
    if(((current_mode == Sentry_BT::NavStatus::FAILURE) && (current_health <= 50.0f))
      )
    {
      blackboard->set("want_position", 3); // defend   
      std::cout << "now want_position = 3" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
 
// ------------------- CheckWhetherChange -------------------
  CheckWhetherChange::CheckWhetherChange(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {
  }

  BT::PortsList CheckWhetherChange::providedPorts()
  {
    return {};
  }

  BT::NodeStatus CheckWhetherChange::tick()
  {
    std::cout << "---------- CheckWhetherChange ----------" << std::endl;
    auto blackboard = config().blackboard;

    auto my_position = blackboard->get<int>("my_position");
    auto want_position = blackboard->get<int>("want_position");
    if(want_position != my_position)
    {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
}  // namespace Sentry_BT