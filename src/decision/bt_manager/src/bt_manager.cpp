#include "bt_manager/bt_manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include "bt_manager/action/auto_actions.hpp"
#include "bt_manager/action/change_stance_action.hpp"
#include "bt_manager/action/gimbal_action.hpp"
#include "bt_manager/action/nav_action.hpp"
#include "bt_manager/action/tactical_action.hpp"
#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/condition/gimbal_condition.hpp"
#include "bt_manager/condition/tactical_condition.hpp"
#include "bt_manager/new_test.hpp"

namespace Sentry_BT {
namespace {
inline void tickTreeExactlyOnce(BT::Tree & tree)
{
  // BehaviorTree.CPP v3 in this workspace does not expose Tree::tickExactlyOnce().
  // tickRoot() here is used as a compatible exact-once tick wrapper.
  tree.tickRoot();
}
}  // namespace

bool SentryBTManager::initialize(
  std::shared_ptr<BT::Blackboard> blackboard, const std::string & tree_root_dir)
{
  tree_root_dir_ = tree_root_dir;
  registerNodes();
  return loadTrees(blackboard);
}

void SentryBTManager::registerNodes()
{
  // nav + common
  factory_.registerNodeType<CheckRetreatCondition>("CheckRetreatCondition");
  factory_.registerNodeType<CheckManualOverride>("CheckManualOverride");
  factory_.registerNodeType<CheckOutpostSafeResponse>("CheckOutpostSafeResponse");
  factory_.registerNodeType<CheckOutpostRemained>("CheckOutpostRemained");
  factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
  factory_.registerBuilder<NavigateToPoseAction>(
    "NavigateToPoseAction", [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<NavigateToPoseAction>(name, "navigate_to_pose", config);
    });
  factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");
  factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
  factory_.registerNodeType<SetManualOverrideGoal>("SetManualOverrideGoal");
  factory_.registerNodeType<SetCoordinate>("SetCoordinate");
  factory_.registerNodeType<ChangeMapAction>("ChangeMapAction");
  factory_.registerNodeType<Wait>("Wait");
  factory_.registerNodeType<SetStairsPosition>("SetStairsPosition");
  factory_.registerNodeType<DescendStairsAction>("DescendStairsAction");
  factory_.registerNodeType<AccumulateAmmoPurchase>("AccumulateAmmoPurchase");
  factory_.registerNodeType<CheckInStairsZone>("CheckInStairsZone");
  factory_.registerNodeType<CheckNoAllyBelowStairs>("CheckNoAllyBelowStairs");
  factory_.registerNodeType<CheckAmmoLow>("CheckAmmoLow");
  factory_.registerNodeType<CheckTacticalModeCondition>("CheckTacticalModeCondition");
  factory_.registerNodeType<CheckOwnFortIdle>("CheckOwnFortIdle");
  factory_.registerNodeType<CheckEnemyBaseLowHp>("CheckEnemyBaseLowHp");
  factory_.registerNodeType<CheckWillThroughTunnel>("CheckWillThroughTunnel");
  factory_.registerNodeType<ControlThroughTunnel>("ControlThroughTunnel");

  // stance
  factory_.registerNodeType<CheckAttackStanceCondition>("CheckAttackStanceCondition");
  factory_.registerNodeType<CheckMoveStanceCondition>("CheckMoveStanceCondition");
  factory_.registerNodeType<CheckDefendStanceCondition>("CheckDefendStanceCondition");
  factory_.registerNodeType<CheckStanceRefreshRequired>("CheckStanceRefreshRequired");
  factory_.registerNodeType<ChangeStance>("ChangeStance");

  // gimbal
  factory_.registerNodeType<CheckTargetVisible>("CheckTargetVisible");
  factory_.registerNodeType<CheckNearEnemyOutpost>("CheckNearEnemyOutpost");
  factory_.registerNodeType<TrackTargetAction>("TrackTargetAction");
  factory_.registerNodeType<SetGimbalPose>("SetGimbalPose");
  factory_.registerNodeType<SetGimbalPoseByAreaAction>("SetGimbalPoseByAreaAction");

  // tactical
  factory_.registerNodeType<CheckDefendCondition>("CheckDefendCondition");
  factory_.registerNodeType<CheckAttackCondition>("CheckAttackCondition");
  factory_.registerNodeType<SetTacticalMode>("SetTacticalMode");
  factory_.registerNodeType<ChangeTacticalAction>("ChangeTacticalAction");

  // debug
  factory_.registerNodeType<BlackboardTestNode>("BlackboardTestNode");
}

bool SentryBTManager::loadTrees(const std::shared_ptr<BT::Blackboard> & blackboard)
{
  try {
    const std::string nav_tree_xml = tree_root_dir_ + "/tree/nav_tree.xml";
    const std::string gimbal_tree_xml = tree_root_dir_ + "/tree/gimbal_tree.xml";
    const std::string stance_tree_xml = tree_root_dir_ + "/tree/stance_tree.xml";
    const std::string tactical_tree_xml = tree_root_dir_ + "/tree/tactical_tree.xml";

    nav_tree_ = factory_.createTreeFromFile(nav_tree_xml, blackboard);
    gimbal_tree_ = factory_.createTreeFromFile(gimbal_tree_xml, blackboard);
    stance_tree_ = factory_.createTreeFromFile(stance_tree_xml, blackboard);
    tactical_tree_ = factory_.createTreeFromFile(tactical_tree_xml, blackboard);

    // Framework default: use nav tree as main tree.
    main_tree_ = &nav_tree_;
  } catch (const std::exception & e) {
    std::cerr << "Error creating behavior tree: " << e.what() << std::endl;
    return false;
  }

  return true;
}

void SentryBTManager::tickMainExactlyOnce()
{
  if (main_tree_ == nullptr) {
    return;
  }

  tickTreeExactlyOnce(*main_tree_);
}

void SentryBTManager::run(double frequency_hz)
{
  if (frequency_hz <= 0.0) {
    frequency_hz = 10.0;
  }

  rclcpp::Rate loop_rate(frequency_hz);
  while (rclcpp::ok()) {
    // Main scheduling hook.
    tickMainExactlyOnce();

    // Additional trees are also ticked here to keep the full framework integrated.
    tickTreeExactlyOnce(stance_tree_);
    tickTreeExactlyOnce(gimbal_tree_);
    tickTreeExactlyOnce(tactical_tree_);

    loop_rate.sleep();
  }
}

}  // namespace Sentry_BT
