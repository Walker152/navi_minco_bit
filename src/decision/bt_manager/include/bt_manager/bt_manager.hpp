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
  bool initialize(const std::string& xml_file_path, std::shared_ptr<BT::Blackboard> blackboard);
  
  // 执行行为树
  void execute();
  
  // 获取行为树状态
  BT::NodeStatus getStatus() const;
  
private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

}  // namespace Sentry_BT