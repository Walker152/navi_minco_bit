#pragma once

#include <memory>
#include <string>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "bt_manager/blackboard.hpp"
#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/action/auto_actions.hpp"
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

namespace Sentry_BT
{

class BTManager
{
public:
  BTManager();
  ~BTManager() = default;
  
  // 初始化行为树
  bool initialize(std::shared_ptr<BT::Blackboard> blackboard);
  
  // 执行行为树
  void execute();
  
  // 获取行为树状态
  BT::NodeStatus getStatus() const;
  
  // 获取功能包路径
  void getPackagePath(const std::string& path);
private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree nav_tree_;
  BT::Tree stance_tree_;
  BT::Tree gimbal_tree_;
  std::string tree_xml_path_;
};

}  // namespace Sentry_BT