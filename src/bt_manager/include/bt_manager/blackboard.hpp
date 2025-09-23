#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <map>
#include <memory>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace Sentry_BT
{

class Blackboard
{
public:
  Blackboard();
  ~Blackboard() = default;
  
  // 初始化黑板数据
  void initialize();
  
  // 更新方法
  void updateHealth(float health);
  void updateOutpostStatus(bool destroyed);
  void updateBonusStatus(bool active);
  void updateTargetInfo(bool locked, const geometry_msgs::msg::Point & pose, const int& id);
  
  // 获取方法
  float getHealth() const;
  geometry_msgs::msg::Pose getPosition() const;
  bool isOutpostDestroyed() const;
  bool isBonusActive() const;
  bool isTargetLocked() const;
  geometry_msgs::msg::Point getTargetPosition() const;
  std::string getTargetId() const;
  
  // 策略状态
  void setCurrentMode(const std::string & mode);
  std::string getCurrentMode() const;
  
  // 导航目标
  void setNavigationGoal(const geometry_msgs::msg::Pose & goal);
  geometry_msgs::msg::Pose getNavigationGoal() const;
  
  // 获取BT黑板的共享指针
  BT::Blackboard::Ptr getBTBlackboard();
  
private:
  // 内部数据存储
  float current_health_;
  bool outpost_destroyed_;
  bool fort_bonus_active_;
  bool target_locked_;
  geometry_msgs::msg::Point target_position_;
  std::string target_id_;
  std::string current_mode_;
  int patrol_index_;
  double last_inspection_time_;
  geometry_msgs::msg::Pose navigation_goal_;
  
  // BehaviorTree CPP 黑板
  BT::Blackboard::Ptr bt_blackboard_;
};

}  // namespace Sentry_BT