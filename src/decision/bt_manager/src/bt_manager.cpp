#include "bt_manager/bt_manager.hpp"
#include "bt_manager/action/change_stance_action.hpp"
#include "bt_manager/action/gimbal_action.hpp"
#include "bt_manager/action/tactical_action.hpp"
#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/condition/gimbal_condition.hpp"
#include "bt_manager/condition/tactical_condition.hpp"
#include "bt_manager/action/nav_action.hpp"
#include "bt_manager/new_test.hpp"

namespace Sentry_BT
{
  BTManager::BTManager() {}

  bool BTManager::initialize(std::shared_ptr<BT::Blackboard> blackboard)
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
    factory_.registerNodeType<ChangeMapAction>("ChangeMapAction");
    factory_.registerNodeType<Wait>("Wait");

    factory_.registerNodeType<CheckMPCondition>("CheckMPCondition");
    factory_.registerNodeType<CheckAPCondition>("CheckAPCondition");
    factory_.registerNodeType<CheckDPCondition>("CheckDPCondition");
    factory_.registerNodeType<CheckTargetVisible>("CheckTargetVisible");
    factory_.registerNodeType<CheckDefendCondition>("CheckDefendCondition");
    factory_.registerNodeType<CheckAttackCondition>("CheckAttackCondition");
    factory_.registerNodeType<TrackTargetAction>("TrackTargetAction");
    factory_.registerNodeType<SetGimbalPose>("SetGimbalPose");
    factory_.registerNodeType<SetTacticalMode>("SetTacticalMode");
    factory_.registerNodeType<ChangeTacticalAction>("ChangeTacticalAction");
    factory_.registerNodeType<ChangeStance>("ChangeStance");
    
    factory_.registerNodeType<DirectVelocityControl>("DirectVelocityControl");
    factory_.registerNodeType<SetStairsPosition>("SetStairsPosition");
    factory_.registerNodeType<CheckInStairsZone>("CheckInStairsZone");

    factory_.registerNodeType<CheckWillThroughTunnel>("CheckWillThroughTunnel");
    factory_.registerNodeType<ControlThroughTunnel>("ControlThroughTunnel");
    factory_.registerNodeType<BlackboardTestNode>("BlackboardTestNode");
    // 创建行为树
    try
    {
      std::string nav_tree_xml = tree_xml_path_ + "/tree/nav_tree.xml";
      nav_tree_ = factory_.createTreeFromFile(nav_tree_xml, blackboard);

      std::string stance_tree_xml = tree_xml_path_ + "/tree/stance_tree.xml";
      stance_tree_ = factory_.createTreeFromFile(stance_tree_xml, blackboard);

      // std::string gimbal_tree_xml = tree_xml_path_ + "/tree/gimbal_tree.xml";
      // gimbal_tree_ = factory_.createTreeFromFile(gimbal_tree_xml, blackboard);
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
    nav_tree_.tickRoot();
    stance_tree_.tickRoot();
    // gimbal_tree_.tickRoot();
  }

  BT::NodeStatus BTManager::getStatus() const
  {
    return nav_tree_.rootNode()->status();
  }

  void BTManager::getPackagePath(const std::string& path)
  {
    tree_xml_path_ = path;
  }
}  // namespace Sentry_BT