#include "bt_manager/condition/resource_conditions.hpp"

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <sstream>

namespace Sentry_BT {

// ------------------- CheckCoinRemaining -------------------
CheckCoinRemaining::CheckCoinRemaining(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCoinRemaining::providedPorts()
{
  return {BT::InputPort<int>("threshold", 10, "Gold threshold"),
    BT::InputPort<std::string>("mode", "greater", "greater or less"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckCoinRemaining::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int threshold = getInput<int>("threshold").value_or(10);
  const std::string mode = getInput<std::string>("mode").value_or("greater");
  const int coin = blackboard->get<int>("coin_remaining");

  const bool active = (mode == "less") ? (coin < threshold) : (coin >= threshold);
  std::ostringstream oss;
  oss << "coin=" << coin << ", threshold=" << threshold << ", mode=" << mode;
  detail::logTransition(detail::TreeKind::RESOURCE, "CheckCoinRemaining", active, oss.str(), branch);
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckEngagedSafeResponse -------------------
CheckEngagedSafeResponse::CheckEngagedSafeResponse(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEngagedSafeResponse::providedPorts()
{
  return {BT::InputPort<double>("stable_seconds", 5.0, "Cooldown after health drop"),
    BT::InputPort<int>("exit_mode", -1, "Mode to exit to on health drop"),
    BT::InputPort<std::string>("exit_blackboard_key", "", "Blackboard key to set on exit"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckEngagedSafeResponse::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double stable_seconds = getInput<double>("stable_seconds").value_or(5.0);
  const int exit_mode = getInput<int>("exit_mode").value_or(-1);
  const std::string exit_key = getInput<std::string>("exit_blackboard_key").value_or("");

  const float health = blackboard->get<float>("health");
  const auto now = std::chrono::steady_clock::now();

  const bool health_dropped = (last_health_ - health) > 1e-3;
  if (health_dropped) {
    last_health_change_time_ = now;
    if (exit_mode >= 0) {
      blackboard->set<NavMode>("current_mode", static_cast<NavMode>(exit_mode));
    }
    if (!exit_key.empty()) {
      blackboard->set<bool>(exit_key, true);
    }
    cooldown_active_ = true;
  }

  last_health_ = health;

  if (cooldown_active_) {
    const double stable_duration = std::chrono::duration<double>(now - last_health_change_time_).count();
    if (stable_duration < stable_seconds) {
      std::ostringstream oss;
      oss << "cooldown active, stable_duration=" << stable_duration
          << ", stable_seconds=" << stable_seconds;
      detail::logTransition(
        detail::TreeKind::RESOURCE, "CheckEngagedSafeResponse", false, oss.str(), branch);
      return BT::NodeStatus::FAILURE;
    }
    cooldown_active_ = false;
  }

  if (!exit_key.empty()) {
    blackboard->set<bool>(exit_key, false);
  }
  detail::logTransition(detail::TreeKind::RESOURCE, "CheckEngagedSafeResponse", true, "safe", branch);
  return BT::NodeStatus::SUCCESS;
}

// ------------------- CheckRemoteExchangeCooldown -------------------
CheckRemoteExchangeCooldown::CheckRemoteExchangeCooldown(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckRemoteExchangeCooldown::providedPorts()
{
  return {BT::InputPort<double>("cooldown_seconds", 6.0, "Cooldown after exchange"),
    BT::InputPort<std::string>(
      "exchange_count_key", "remote_ammo_exchange_count", "Blackboard key for successful exchange count"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckRemoteExchangeCooldown::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double cooldown_seconds = getInput<double>("cooldown_seconds").value_or(6.0);
  const std::string exchange_count_key =
    getInput<std::string>("exchange_count_key").value_or("remote_ammo_exchange_count");
  const int current_count = blackboard->get<int>(exchange_count_key);
  const auto now = std::chrono::steady_clock::now();

  if (!initialized_) {
    last_exchange_count_ = current_count;
    last_exchange_time_ = now - std::chrono::seconds(static_cast<long>(cooldown_seconds) +
                                                     1);  // initialize to be out of cooldown
    initialized_ = true;
  }
  if (current_count > last_exchange_count_) {
    last_exchange_count_ = current_count;
    last_exchange_time_ = now;
  }

  const double elapsed = std::chrono::duration<double>(now - last_exchange_time_).count();
  const bool in_cooldown = elapsed < cooldown_seconds;

  if (!in_cooldown) {
    last_exchange_count_ = current_count;
    last_exchange_time_ = now;
  }
  std::ostringstream oss;
  oss << "exchange_count_key=" << exchange_count_key << ", exchange_count=" << current_count
      << ", elapsed=" << elapsed << ", cooldown=" << cooldown_seconds;
  detail::logTransition(
    detail::TreeKind::RESOURCE, "CheckRemoteExchangeCooldown", !in_cooldown, oss.str(), branch);
  return in_cooldown ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

// ------------------- CheckNormalExchangeCooldown -------------------
CheckNormalExchangeCooldown::CheckNormalExchangeCooldown(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckNormalExchangeCooldown::providedPorts()
{
  return {BT::InputPort<double>("cooldown_seconds", 1.5, "Cooldown after normal exchange"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckNormalExchangeCooldown::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const double cooldown_seconds = getInput<double>("cooldown_seconds").value_or(1.5);
  const auto now = std::chrono::steady_clock::now();

  // 1. 初始化：开局将上一次兑换时间设为很久以前，确保第一次能够顺利放行
  if (!initialized_) {
    last_exchange_time_ = now - std::chrono::seconds(static_cast<long>(cooldown_seconds + 1.0));
    initialized_ = true;
  }

  // 2. 计算距离上次放行经过的时间
  const double elapsed = std::chrono::duration<double>(now - last_exchange_time_).count();
  const bool in_cooldown = elapsed < cooldown_seconds;

  // 3. 核心防抖逻辑：如果不处于冷却，意味着本次 tick 会返回 SUCCESS 并触发后续买弹动作。
  if (!in_cooldown) {
    last_exchange_time_ = now;
  }

  std::ostringstream oss;
  oss << "elapsed=" << elapsed << ", cooldown=" << cooldown_seconds;
  detail::logTransition(
    detail::TreeKind::RESOURCE, "CheckNormalExchangeCooldown", !in_cooldown, oss.str(), branch);

  return in_cooldown ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

// ------------------- CheckRemainingAmmoExchange -------------------
CheckRemainingAmmoExchange::CheckRemainingAmmoExchange(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckRemainingAmmoExchange::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckRemainingAmmoExchange::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int remaining = blackboard->get<int>("remaining_ammo_exchange");
  const bool available = remaining > 0;
  std::ostringstream oss;
  oss << "remaining=" << remaining;
  detail::logTransition(
    detail::TreeKind::RESOURCE, "CheckRemainingAmmoExchange", available, oss.str(), branch);
  return available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckAttackFortHealthExchangeNeeded -------------------
CheckAttackFortHealthExchangeNeeded::CheckAttackFortHealthExchangeNeeded(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckAttackFortHealthExchangeNeeded::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckAttackFortHealthExchangeNeeded::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");

  const auto tactical_mode = blackboard->get<TacticalMode>("tactical_mode");
  const auto control_mode = blackboard->get<ControlMode>("control_mode");
  const bool in_transform_zone = blackboard->get<bool>("in_transform_zone");
  const bool needed = tactical_mode == TacticalMode::OFFENSIVE &&
                      control_mode == ControlMode::MANUAL_CONTROL && in_transform_zone && !requested_;
  if (needed) {
    requested_ = true;
  }

  std::ostringstream oss;
  oss << std::boolalpha << "tactical_mode=" << static_cast<int>(tactical_mode)
      << ", control_mode=" << static_cast<int>(control_mode) << ", in_transform_zone=" << in_transform_zone
      << ", requested=" << requested_;
  detail::logTransition(
    detail::TreeKind::RESOURCE, "CheckAttackFortHealthExchangeNeeded", needed, oss.str(), branch);
  return needed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckInZone -------------------
CheckInZone::CheckInZone(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckInZone::providedPorts()
{
  return {BT::InputPort<std::string>(
            "zone_name",
            "",
            "Zone: enemy_defense/own_defense/highland/own_outpost_zone/enemy_outpost_zone/"
            "own_supply_zone/enemy_supply_zone"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckInZone::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const std::string zone_name = getInput<std::string>("zone_name").value_or("");
  const auto current_pose = blackboard->get<geometry_msgs::msg::Pose>("current_pose");

  const Point2D point{current_pose.position.x, current_pose.position.y, 0.0};
  bool in_zone = false;

  if (zone_name == "enemy_defense") {
    in_zone = enemy_defense_zone.contains(point);
  } else if (zone_name == "own_defense") {
    in_zone = own_defense_zone.contains(point);
  } else if (zone_name == "highland") {
    in_zone = highland_zone.contains(point);
  } else if (zone_name == "own_outpost_zone") {
    in_zone = own_outpost_buff_zone.contains(point);
  } else if (zone_name == "enemy_outpost_zone") {
    in_zone = enemy_outpost_buff_zone.contains(point);
  } else if (zone_name == "own_supply_zone") {
    in_zone = own_base_buff_zone.contains(point);
  } else if (zone_name == "enemy_supply_zone") {
    in_zone = enemy_base_buff_zone.contains(point);
  }

  std::ostringstream oss;
  oss << "zone=" << zone_name;
  detail::logTransition(detail::TreeKind::RESOURCE, "CheckInZone", in_zone, oss.str(), branch);
  return in_zone ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckCanFreeResurrect -------------------
CheckCanFreeResurrect::CheckCanFreeResurrect(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCanFreeResurrect::providedPorts()
{
  return {};
}

BT::NodeStatus CheckCanFreeResurrect::tick()
{
  auto blackboard = config().blackboard;
  const bool can = blackboard->get<bool>("can_free_resurrect");
  return can ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckEnergyActive -------------------
CheckEnergyActive::CheckEnergyActive(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckEnergyActive::providedPorts()
{
  return {BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging")};
}

BT::NodeStatus CheckEnergyActive::tick()
{
  auto blackboard = config().blackboard;
  const std::string branch = getInput<std::string>("branch").value_or("");
  const int big_energy = blackboard->get<int>("big_energy_status");
  const int small_energy = blackboard->get<int>("small_energy_status");
  const bool active = big_energy == 0 || small_energy == 0 || big_energy == 2 || small_energy == 2;
  std::ostringstream oss;
  oss << "big_energy_status=" << big_energy << ", small_energy_status=" << small_energy;
  detail::logTransition(detail::TreeKind::RESOURCE, "CheckEnergyActive", active, oss.str(), branch);
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ------------------- CheckCanActivateEnergy -------------------
CheckCanActivateEnergy::CheckCanActivateEnergy(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCanActivateEnergy::providedPorts()
{
  return {};
}

BT::NodeStatus CheckCanActivateEnergy::tick()
{
  auto blackboard = config().blackboard;
  const bool can = blackboard->get<bool>("can_activate_energy");
  return can ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
