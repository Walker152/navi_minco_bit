#include "bt_manager/condition/recovery_conditions.hpp"

#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/log.hpp"
#include "bt_manager/utils/nav_zone.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <sstream>

namespace Sentry_BT {
namespace {

int getZoneIndexByName(const std::string & zone_name, const geometry_msgs::msg::Pose & pose)
{
  const Point2D point{pose.position.x, pose.position.y, 0.0};

  if (zone_name == "transform_zone") {
    for (std::size_t i = 0; i < transform_zone.size(); ++i) {
      if (transform_zone[i].contains(point)) {
        return static_cast<int>(i);
      }
    }
  }

  return -1;
}

bool isInsideZoneByName(const std::string & zone_name, const geometry_msgs::msg::Pose & pose)
{
  const Point2D point{pose.position.x, pose.position.y, 0.0};

  if (zone_name == "transform_zone") {
    for (const auto & zone : transform_zone) {
      if (zone.contains(point)) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace

CheckTimeInZone::CheckTimeInZone(const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckTimeInZone::providedPorts()
{
  return {
    BT::InputPort<double>("min_seconds", 0.0, "Minimum time in zone"),
    BT::InputPort<double>("max_seconds", 9999.0, "Maximum time in zone"),
    BT::InputPort<std::string>("zone_name", "transform_zone", "Zone name"),
    BT::InputPort<std::string>("branch", "", "Branch/sequence tag for logging"),
  };
}

BT::NodeStatus CheckTimeInZone::tick()
{
  const double min_seconds = getInput<double>("min_seconds").value_or(0.0);
  const double max_seconds = getInput<double>("max_seconds").value_or(9999.0);
  const std::string branch = getInput<std::string>("branch").value_or("");

  auto blackboard = config().blackboard;
  const bool in_zone = blackboard->get<bool>("in_transform_zone");
  const int zone_index = blackboard->get<int>("nearest_tunnel_idx");

  if (!in_zone) {
    was_in_zone_ = false;
    detail::logTransition(
      detail::TreeKind::RECOVERY, "CheckTimeInZone", false, "not in transform_zone", branch);
    return BT::NodeStatus::FAILURE;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!was_in_zone_) {
    entry_time_ = now;
    was_in_zone_ = true;
  }

  const double duration = std::chrono::duration<double>(now - entry_time_).count();
  if (duration >= min_seconds && duration < max_seconds) {
    std::ostringstream oss;
    oss << "zone_idx=" << zone_index << ", duration=" << duration << "s, window=[" << min_seconds << ", "
        << max_seconds << ")";
    detail::logTransition(detail::TreeKind::RECOVERY, "CheckTimeInZone", true, oss.str(), branch);
    return BT::NodeStatus::SUCCESS;
  }

  std::ostringstream oss;
  oss << "zone_idx=" << zone_index << ", duration=" << duration << "s, window=[" << min_seconds << ", "
      << max_seconds << ")";
  detail::logTransition(detail::TreeKind::RECOVERY, "CheckTimeInZone", false, oss.str(), branch);

  return BT::NodeStatus::FAILURE;
}

}  // namespace Sentry_BT
