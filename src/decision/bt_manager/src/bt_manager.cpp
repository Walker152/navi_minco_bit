#include "bt_manager/bt_manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include "bt_manager/action/auto_actions.hpp"
#include "bt_manager/action/change_stance_action.hpp"
#include "bt_manager/action/gimbal_action.hpp"
#include "bt_manager/action/nav_action.hpp"
#include "bt_manager/action/recovery_actions.hpp"
#include "bt_manager/action/resource_actions.hpp"
#include "bt_manager/action/tactical_action.hpp"
#include "bt_manager/condition/auto_conditions.hpp"
#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/condition/gimbal_condition.hpp"
#include "bt_manager/condition/resource_conditions.hpp"
#include "bt_manager/condition/tactical_condition.hpp"

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
  factory_.registerNodeType<CheckGameTimeWindow>("CheckGameTimeWindow");
  factory_.registerNodeType<CheckBigEnergyActive>("CheckBigEnergyActive");
  factory_.registerNodeType<CheckManualOverride>("CheckManualOverride");
  factory_.registerNodeType<CheckOutpostSafeResponse>("CheckOutpostSafeResponse");
  factory_.registerNodeType<CheckOutpostRemained>("CheckOutpostRemained");
  factory_.registerNodeType<SetEnemyOutpostDestroyed>("SetEnemyOutpostDestroyed");
  factory_.registerNodeType<CheckOwnOutpostAlive>("CheckOwnOutpostAlive");
  factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
  factory_.registerBuilder<NavigateToPoseAction>(
    "NavigateToPoseAction", [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<NavigateToPoseAction>(name, "navigate_to_pose", config);
    });
  factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");
  factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
  factory_.registerNodeType<SetManualOverrideGoal>("SetManualOverrideGoal");
  factory_.registerNodeType<SetCoordinate>("SetCoordinate");
  factory_.registerNodeType<SetNavMode>("SetNavMode");
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
  factory_.registerNodeType<EmergencyStop>("EmergencyStop");
  // resource / exchange
  factory_.registerNodeType<CheckCoinRemaining>("CheckCoinRemaining");
  factory_.registerNodeType<CheckEngagedSafeResponse>("CheckEngagedSafeResponse");
  factory_.registerNodeType<CheckRemoteExchangeCooldown>("CheckRemoteExchangeCooldown");
  factory_.registerNodeType<CheckRemainingAmmoExchange>("CheckRemainingAmmoExchange");
  factory_.registerNodeType<CheckAttackFortHealthExchangeNeeded>("CheckAttackFortHealthExchangeNeeded");
  factory_.registerNodeType<CheckInZone>("CheckInZone");
  factory_.registerNodeType<CheckCanFreeResurrect>("CheckCanFreeResurrect");
  factory_.registerNodeType<CheckEnergyActive>("CheckEnergyActive");
  factory_.registerNodeType<CheckCanActivateEnergy>("CheckCanActivateEnergy");
  factory_.registerNodeType<RequestReviveAction>("RequestReviveAction");
  factory_.registerNodeType<RequestRemoteAmmoExchangeAction>("RequestRemoteAmmoExchangeAction");
  factory_.registerNodeType<RequestRemoteHealthExchangeAction>("RequestRemoteHealthExchangeAction");
  factory_.registerNodeType<ControlThroughTunnel>("ControlThroughTunnel");
  factory_.registerNodeType<TunnelTimeoutBackoutAction>("TunnelTimeoutBackoutAction");
  factory_.registerNodeType<CheckNormalExchangeCooldown>("CheckNormalExchangeCooldown");

  // stance
  factory_.registerNodeType<CheckHeat>("CheckHeat");
  factory_.registerNodeType<CheckOutpostTarget>("CheckOutpostTarget");
  factory_.registerNodeType<CheckEngagedStatus>("CheckEngagedStatus");
  factory_.registerNodeType<CheckHealth>("CheckHealth");
  factory_.registerNodeType<CheckTargetDistance>("CheckTargetDistance");
  factory_.registerNodeType<CheckCrossZoneTransition>("CheckCrossZoneTransition");
  factory_.registerNodeType<CheckCapacitorCapacity>("CheckCapacitorCapacity");
  factory_.registerNodeType<CheckStanceCooldown>("CheckStanceCooldown");
  factory_.registerNodeType<CheckStanceEffectLimit>("CheckStanceEffectLimit");
  factory_.registerNodeType<CheckManualStanceOverride>("CheckManualStanceOverride");
  factory_.registerNodeType<ApplyManualStanceOverride>("ApplyManualStanceOverride");
  factory_.registerNodeType<CheckStanceRefreshRequired>("CheckStanceRefreshRequired");
  factory_.registerNodeType<SetGyroState>("SetGyroState");
  factory_.registerNodeType<TunnelGyroAlignAction>("TunnelGyroAlignAction");
  factory_.registerNodeType<ChangeStance>("ChangeStance");
  factory_.registerNodeType<UpdateStanceDuration>("UpdateStanceDuration");
  factory_.registerNodeType<CheckTunnelDeformation>("CheckTunnelDeformation");
  factory_.registerNodeType<CheckInEnemyFortZone>("CheckInEnemyFortZone");

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
}

bool SentryBTManager::loadTrees(const std::shared_ptr<BT::Blackboard> & blackboard)
{
  try {
    const std::string nav_tree_xml = tree_root_dir_ + "/tree/nav_tree.xml";
    const std::string gimbal_tree_xml = tree_root_dir_ + "/tree/gimbal_tree.xml";
    const std::string stance_tree_xml = tree_root_dir_ + "/tree/stance_tree.xml";
    const std::string tactical_tree_xml = tree_root_dir_ + "/tree/tactical_tree.xml";
    const std::string resource_tree_xml = tree_root_dir_ + "/tree/resource_tree.xml";

    resource_tree_ = factory_.createTreeFromFile(resource_tree_xml, blackboard);
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

void SentryBTManager::tickStanceExactlyOnce()
{
  tickTreeExactlyOnce(stance_tree_);
}

void SentryBTManager::tickTacticalExactlyOnce()
{
  tickTreeExactlyOnce(tactical_tree_);
}

void SentryBTManager::run(double frequency_hz)
{
  if (frequency_hz <= 0.0) {
    frequency_hz = 10.0;
  }

  rclcpp::Rate loop_rate(frequency_hz);
  while (rclcpp::ok()) {
    // Tick order: resource (exchange/revive) -> tactical (mode) -> nav -> stance -> gimbal
    tickTreeExactlyOnce(resource_tree_);
    tickTreeExactlyOnce(tactical_tree_);
    tickMainExactlyOnce();
    tickTreeExactlyOnce(stance_tree_);
    tickTreeExactlyOnce(gimbal_tree_);

    loop_rate.sleep();
  }
}

}  // namespace Sentry_BT
