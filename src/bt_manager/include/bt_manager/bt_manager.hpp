#pragma once

#include <memory>
#include <string>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "bt_manager/blackboard.hpp"
#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/auto_actions.hpp"

namespace Sentry_BT
{

class BTManager
{
public:
  BTManager();
  ~BTManager() = default;
  
  // 初始化行为树
  bool initialize(const std::string& xml_file_path);
  
  // 执行行为树
  void execute();
  
  // 获取行为树状态
  BT::NodeStatus getStatus() const;
  
  // 获取黑板
  std::shared_ptr<Blackboard> getBlackboard();
  
private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::shared_ptr<Blackboard> blackboard_;
};

}  // namespace Sentry_BT