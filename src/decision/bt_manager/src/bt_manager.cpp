#include "bt_manager/bt_manager.hpp"

namespace Sentry_BT
{
  BTManager::BTManager() {}

  bool BTManager::initialize(const std::string& xml_file_path, std::shared_ptr<BT::Blackboard> blackboard)
  {
    // 注册自定义节点
    factory_.registerNodeType<CheckRetreatCondition>("CheckRetreatCondition");
    factory_.registerNodeType<CheckOutpostRemained>("CheckOutpostRemained");
    factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
    factory_.registerNodeType<PublishNavigationGoal>("PublishNavigationGoal");
    factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");
    factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
    factory_.registerNodeType<CheckNavStatus>("CheckNavStatus");
    factory_.registerNodeType<CheckIfRetreating>("CheckIfRetreating");
    factory_.registerNodeType<SetCoordinate>("SetCoordinate");
    factory_.registerNodeType<WaitUntilStopped>("WaitUntilStopped");
    factory_.registerNodeType<Wait>("Wait");

    factory_.registerNodeType<CheckMPCondition>("CheckMPCondition");
    factory_.registerNodeType<CheckAPCondition>("CheckAPCondition");
    factory_.registerNodeType<CheckDPCondition>("CheckDPCondition");
    factory_.registerNodeType<CheckWhetherChange>("CheckWhetherChange");
    factory_.registerNodeType<ChangePosition>("ChangePosition");
    factory_.registerNodeType<ChangePosition>("JustProtect");
    // 创建行为树
    try
    {
      tree_ = factory_.createTreeFromFile(xml_file_path, blackboard);
    }
    catch(const std::exception& e)
    {
      std::cerr << "Error creating behavior tree: " << e.what() << std::endl;
      return false;
    }

    return true;
  }

  void BTManager::execute()
  {
    tree_.tickRoot();
  }

  BT::NodeStatus BTManager::getStatus() const
  {
    return tree_.rootNode()->status();
  }
}  // namespace Sentry_BT