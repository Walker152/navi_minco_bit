#include "bt_manager/blackboard.hpp"
#include "bt_manager/auto_conditions.hpp"
#include "bt_manager/auto_actions.hpp"

namespace Sentry_BT
{
// ------------------- SetHomeCoordinate -------------------
SetHomeCoordinate::SetHomeCoordinate(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetHomeCoordinate::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::Pose>("home_coordinate") };
}   

BT::NodeStatus SetHomeCoordinate::tick()
{
  // 从黑板获取家坐标
  auto home_coord = getInput<geometry_msgs::msg::Pose>("home_coordinate");
  if (!home_coord)
  {
    throw BT::RuntimeError("missing required input [home_coordinate]: ", home_coord.error());
  }

  // 将家坐标设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("home_coordinate", home_coord.value());

  return BT::NodeStatus::SUCCESS;
}

// ------------------- PublishNavigationGoal -------------------
PublishNavigationGoal::PublishNavigationGoal(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList PublishNavigationGoal::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::Pose>("navigation_goal") };
}

BT::NodeStatus PublishNavigationGoal::tick()
{
  // 从黑板获取导航目标
  auto nav_goal = getInput<geometry_msgs::msg::Pose>("navigation_goal");
  if (!nav_goal)
  {
    throw BT::RuntimeError("missing required input [navigation_goal]: ", nav_goal.error());
  }

  // 将导航目标设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("navigation_goal", nav_goal.value());

  return BT::NodeStatus::SUCCESS;
}

// ------------------- SetBonusCoordinate -------------------
SetBonusCoordinate::SetBonusCoordinate(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetBonusCoordinate::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::Pose>("bonus_coordinate") };
}

BT::NodeStatus SetBonusCoordinate::tick()
{
  // 从黑板获取据点坐标
  auto bonus_coord = getInput<geometry_msgs::msg::Pose>("bonus_coordinate");
  if (!bonus_coord)
  {
    throw BT::RuntimeError("missing required input [bonus_coordinate]: ", bonus_coord.error());
  }

  // 将据点坐标设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("bonus_coordinate", bonus_coord.value());

  return BT::NodeStatus::SUCCESS;
}

// ------------------- SetTargetCoordinate -------------------
SetTargetCoordinate::SetTargetCoordinate(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{   
}

BT::PortsList SetTargetCoordinate::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::Pose>("target_coordinate") };
}

BT::NodeStatus SetTargetCoordinate::tick()
{
  // 从黑板获取目标坐标
  auto target_coord = getInput<geometry_msgs::msg::Pose>("target_coordinate");
  if (!target_coord)
  {
    throw BT::RuntimeError("missing required input [target_coordinate]: ", target_coord.error());
  }

  // 将目标坐标设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("target_coordinate", target_coord.value());

  return BT::NodeStatus::SUCCESS;
}

// ------------------- SelectInspectionArea -------------------
SelectInspectionArea::SelectInspectionArea(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectInspectionArea::providedPorts()
{
  return { BT::InputPort<int>("patrol_index"),
           BT::InputPort<double>("last_inspection_time"),
           BT::InputPort<double>("inspection_interval"),
           BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("inspection_areas") };
}

BT::NodeStatus SelectInspectionArea::tick()
{
  // 从黑板获取巡逻点索引、上次巡检时间、巡检间隔和巡检区域列表
  auto patrol_index = getInput<int>("patrol_index");
  auto last_inspection_time = getInput<double>("last_inspection_time");
  auto inspection_interval = getInput<double>("inspection_interval");
  auto inspection_areas = getInput<std::vector<geometry_msgs::msg::Pose>>("inspection_areas");

  if (!patrol_index)
  {
    throw BT::RuntimeError("missing required input [patrol_index]: ", patrol_index.error());
  }
  if (!last_inspection_time)
  {
    throw BT::RuntimeError("missing required input [last_inspection_time]: ", last_inspection_time.error());
  }
  if (!inspection_interval)
  {
    throw BT::RuntimeError("missing required input [inspection_interval]: ", inspection_interval.error());
  }
  if (!inspection_areas)
  {
    throw BT::RuntimeError("missing required input [inspection_areas]: ", inspection_areas.error());
  }

  // 检查是否需要切换巡检区域
  double current_time = static_cast<double>(std::time(nullptr));
  if (current_time - last_inspection_time.value() >= inspection_interval.value())
  {
    int next_index = (patrol_index.value() + 1) % inspection_areas.value().size();

    // 将新的巡逻点索引和巡检区域设置到黑板
    auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
    blackboard->set("patrol_index", next_index);
    blackboard->set("inspection_area", inspection_areas.value()[next_index]);
    blackboard->set("last_inspection_time", current_time);

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;   
  }
}

// ------------------- SetAreaCoordinate -------------------
SetAreaCoordinate::SetAreaCoordinate(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SetAreaCoordinate::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::Pose>("area_coordinate") };
}   

BT::NodeStatus SetAreaCoordinate::tick()
{
  // 从黑板获取区域坐标
  auto area_coord = getInput<geometry_msgs::msg::Pose>("area_coordinate");
  if (!area_coord)
  {
    throw BT::RuntimeError("missing required input [area_coordinate]: ", area_coord.error());
  }

  // 将区域坐标设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("area_coordinate", area_coord.value());

  return BT::NodeStatus::SUCCESS;
}

// ------------------- SelectPatrolPoint -------------------
SelectPatrolPoint::SelectPatrolPoint(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config)
{
}

BT::PortsList SelectPatrolPoint::providedPorts()
{
  return { BT::InputPort<int>("patrol_index"),
           BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("patrol_points") };
}

BT::NodeStatus SelectPatrolPoint::tick()
{
  // 从黑板获取巡逻点索引和巡逻点列表
  auto patrol_index = getInput<int>("patrol_index");
  auto patrol_points = getInput<std::vector<geometry_msgs::msg::Pose>>("patrol_points");

  if (!patrol_index)
  {
    throw BT::RuntimeError("missing required input [patrol_index]: ", patrol_index.error());
  }
  if (!patrol_points)
  {
    throw BT::RuntimeError("missing required input [patrol_points]: ", patrol_points.error());
  }

  // 检查巡逻点索引是否有效
  if (patrol_index.value() < 0 || patrol_index.value() >= static_cast<int>(patrol_points.value().size()))
  {
    throw BT::RuntimeError("invalid patrol_index: ", patrol_index.value());
  }

  // 将选中的巡逻点设置到黑板
  auto blackboard = Sentry_BT::Blackboard().getBTBlackboard();
  blackboard->set("patrol_point", patrol_points.value()[patrol_index.value()]);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace Sentry_BT