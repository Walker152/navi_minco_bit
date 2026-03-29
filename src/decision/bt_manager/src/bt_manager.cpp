#include "bt_manager/bt_manager.hpp"
#include "bt_manager/action/change_stance_action.hpp"
#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/action/nav_action.hpp"

namespace Sentry_BT
{
  BTManager::BTManager() {}

  bool BTManager::initialize(const std::string& xml_file_path, std::shared_ptr<BT::Blackboard> blackboard)
  {
    // 注册自定义节点
    factory_.registerNodeType<CheckRetreatCondition>("CheckRetreatCondition");
    factory_.registerNodeType<CheckOutpostRemained>("CheckOutpostRemained");
    factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
    factory_.registerBuilder<NavigateToPoseAction>(
        "NavigateToPoseAction",
        [](const std::string& name, const BT::NodeConfiguration& config)
        {
          return std::make_unique<NavigateToPoseAction>(name, "navigate_to_pose", config);
        });
    factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");
    factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
    factory_.registerNodeType<SetCoordinate>("SetCoordinate");
    factory_.registerNodeType<Wait>("Wait");

    factory_.registerNodeType<CheckMPCondition>("CheckMPCondition");
    factory_.registerNodeType<CheckAPCondition>("CheckAPCondition");
    factory_.registerNodeType<CheckDPCondition>("CheckDPCondition");
    factory_.registerNodeType<ChangeStance>("ChangeStance");
    
    factory_.registerNodeType<DirectVelocityControl>("DirectVelocityControl");
    factory_.registerNodeType<SetStairsPosition>("SetStairsPosition");
    factory_.registerNodeType<CheckInStairsZone>("CheckInStairsZone");

    factory_.registerNodeType<CheckWillThroughTunnel>("CheckWillThroughTunnel");
    factory_.registerNodeType<ControlThroughTunnel>("ControlThroughTunnel");
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