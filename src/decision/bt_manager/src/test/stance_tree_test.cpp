#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "bt_manager/condition/change_stance_condition.hpp"
#include "bt_manager/condition/gimbal_condition.hpp"
#include "bt_manager/utils/area.hpp"

namespace {

using Sentry_BT::Point2D;
using Sentry_BT::SentryStance;

class TestChangeStance : public BT::SyncActionNode
{
public:
  TestChangeStance(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("stance", "Target stance: ATTACK/DEFEND/MOVE"),
      BT::InputPort<bool>("use_gyro", "Whether to enable gyro mode"),
      BT::InputPort<float>("gyro_vel", "Gyro speed in rpm")};
  }

  BT::NodeStatus tick() override
  {
    const auto stance_str = getInput<std::string>("stance").value_or("DEFEND");
    const auto use_gyro = getInput<bool>("use_gyro").value_or(false);
    const auto gyro_vel = getInput<float>("gyro_vel").value_or(0.0f);

    auto parse_stance = [](const std::string & value) {
      if (value == "ATTACK") {
        return SentryStance::ATTACK;
      }
      if (value == "MOVE") {
        return SentryStance::MOVE;
      }
      return SentryStance::DEFEND;
    };

    auto blackboard = config().blackboard;
    blackboard->set<SentryStance>("desired_stance", parse_stance(stance_str));
    blackboard->set<SentryStance>("current_stance", parse_stance(stance_str));
    blackboard->set<bool>("use_gyro_mode", use_gyro);
    blackboard->set<float>("gyro_vel", gyro_vel);
    blackboard->set<std::string>("last_stance_action", name());
    return BT::NodeStatus::SUCCESS;
  }
};

struct Scenario
{
  std::string name;
  std::string expected_action;
  SentryStance expected_stance;
  std::function<void(BT::Blackboard::Ptr &)> configure;
};

geometry_msgs::msg::Pose makePose(double x, double y)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0.0;
  p.orientation.w = 1.0;
  return p;
}

std::string loadFile(const std::string & path)
{
  std::ifstream ifs(path);
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

BT::Blackboard::Ptr makeDefaultBlackboard()
{
  auto bb = BT::Blackboard::create();

  bb->set<int>("current_heat", 0);
  bb->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::PATROL));
  bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(22.0, 7.0));
  bb->set<Point2D>("nav_goal", Point2D{22.0, 7.0, 0.0});
  bb->set<bool>("is_disengaged", false);
  bb->set<float>("health", 100.0f);
  bb->set<geometry_msgs::msg::Pose>("target_pose", makePose(22.3, 7.0));
  bb->set<bool>("target_valid", false);
  bb->set<float>("capacitor_capacity", 100.0f);
  bb->set<SentryStance>("current_stance", SentryStance::DEFEND);
  bb->set<SentryStance>("desired_stance", SentryStance::DEFEND);
  bb->set<bool>("use_gyro_mode", true);
  bb->set<float>("gyro_vel", 50.0f);
  bb->set<std::string>("last_stance_action", "");

  return bb;
}

bool runScenario(
  BT::BehaviorTreeFactory & factory, const std::string & xml, const Scenario & s, rclcpp::Logger logger)
{
  auto bb = makeDefaultBlackboard();
  s.configure(bb);

  auto tree = factory.createTreeFromText(xml, bb);
  tree.tickRoot();

  const auto action = bb->get<std::string>("last_stance_action");
  const auto stance = bb->get<SentryStance>("current_stance");

  const bool ok = (action == s.expected_action) && (stance == s.expected_stance);
  if (ok) {
    RCLCPP_INFO(logger, "[STANCE_TEST][PASS] %s -> %s", s.name.c_str(), action.c_str());
  } else {
    RCLCPP_ERROR(logger,
      "[STANCE_TEST][FAIL] %s expect(action=%s, stance=%d) got(action=%s, stance=%d)",
      s.name.c_str(),
      s.expected_action.c_str(),
      static_cast<int>(s.expected_stance),
      action.c_str(),
      static_cast<int>(stance));
  }
  return ok;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("stance_tree_test");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<Sentry_BT::CheckHeat>("CheckHeat");
  factory.registerNodeType<Sentry_BT::CheckOutpostTarget>("CheckOutpostTarget");
  factory.registerNodeType<Sentry_BT::CheckEngagedStatus>("CheckEngagedStatus");
  factory.registerNodeType<Sentry_BT::CheckHealth>("CheckHealth");
  factory.registerNodeType<Sentry_BT::CheckTargetDistance>("CheckTargetDistance");
  factory.registerNodeType<Sentry_BT::CheckCrossZoneTransition>("CheckCrossZoneTransition");
  factory.registerNodeType<Sentry_BT::CheckCapacitorCapacity>("CheckCapacitorCapacity");
  factory.registerNodeType<Sentry_BT::CheckStanceRefreshRequired>("CheckStanceRefreshRequired");
  factory.registerNodeType<Sentry_BT::CheckTargetVisible>("CheckTargetVisible");
  factory.registerNodeType<TestChangeStance>("ChangeStance");

  const auto share_dir = ament_index_cpp::get_package_share_directory("bt_manager");
  const auto xml_path = share_dir + "/tree/stance_tree.xml";
  const auto base_xml = loadFile(xml_path);

  std::string refresh_xml = base_xml;
  const std::string from = "max_hold_seconds=\"180\"";
  const std::string to = "max_hold_seconds=\"0\"";
  const auto pos = refresh_xml.find(from);
  if (pos != std::string::npos) {
    refresh_xml.replace(pos, from.size(), to);
  }

  std::vector<Scenario> scenarios;

  scenarios.push_back(
    {"baseline_combat_attack", "ChangeStanceCombatAttack", SentryStance::ATTACK, [](BT::Blackboard::Ptr &) {
     }});

  scenarios.push_back(
    {"cross_zone_move_no_gyro", "ChangeStanceMoveNoGyro", SentryStance::MOVE, [](BT::Blackboard::Ptr & bb) {
       bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(3.0, 8.0));
       bb->set<Point2D>("nav_goal", Point2D{13.0, 7.0, 0.0});
     }});

  scenarios.push_back({"absolute_attack_by_heat",
    "ChangeStanceAbsoluteAttack",
    SentryStance::ATTACK,
    [](BT::Blackboard::Ptr & bb) {
      bb->set<int>("current_heat", 250);
    }});

  scenarios.push_back({"absolute_attack_by_outpost",
    "ChangeStanceAbsoluteAttack",
    SentryStance::ATTACK,
    [](BT::Blackboard::Ptr & bb) {
      const auto outpost = Sentry_BT::nav_points[static_cast<size_t>(Sentry_BT::NavGoal::OUTPOST)];
      bb->set<int>("current_mode", static_cast<int>(Sentry_BT::NavMode::RESPONSE));
      bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(15.0, 11.0));
      bb->set<Point2D>("nav_goal", outpost);
    }});

  scenarios.push_back(
    {"pursuit_move", "ChangeStancePursuit", SentryStance::MOVE, [](BT::Blackboard::Ptr & bb) {
       bb->set<bool>("target_valid", true);
       bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(22.0, 7.0));
       bb->set<geometry_msgs::msg::Pose>("target_pose", makePose(24.5, 7.0));
     }});

  scenarios.push_back(
    {"combat_attack", "ChangeStanceCombatAttack", SentryStance::ATTACK, [](BT::Blackboard::Ptr & bb) {
       bb->set<bool>("is_disengaged", false);
       bb->set<float>("health", 80.0f);
     }});

  scenarios.push_back(
    {"move_gyro_by_low_cap", "ChangeStanceMoveWithGyro", SentryStance::MOVE, [](BT::Blackboard::Ptr & bb) {
       bb->set<bool>("is_disengaged", true);
       bb->set<float>("health", 80.0f);
       bb->set<float>("capacitor_capacity", 10.0f);
     }});

  scenarios.push_back(
    {"defend_low_health", "ChangeStanceDefend", SentryStance::DEFEND, [](BT::Blackboard::Ptr & bb) {
       bb->set<bool>("is_disengaged", false);
       bb->set<float>("health", 20.0f);
       bb->set<float>("capacitor_capacity", 80.0f);
     }});

  scenarios.push_back({"priority_crosszone_over_heat",
    "ChangeStanceMoveNoGyro",
    SentryStance::MOVE,
    [](BT::Blackboard::Ptr & bb) {
      bb->set<int>("current_heat", 250);
      bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(3.0, 8.0));
      bb->set<Point2D>("nav_goal", Point2D{13.0, 7.0, 0.0});
    }});

  scenarios.push_back({"priority_pursuit_over_combat",
    "ChangeStancePursuit",
    SentryStance::MOVE,
    [](BT::Blackboard::Ptr & bb) {
      bb->set<bool>("target_valid", true);
      bb->set<geometry_msgs::msg::Pose>("current_pose", makePose(22.0, 7.0));
      bb->set<geometry_msgs::msg::Pose>("target_pose", makePose(24.5, 7.0));
      bb->set<bool>("is_disengaged", false);
      bb->set<float>("health", 90.0f);
    }});

  scenarios.push_back(
    {"refresh_branch", "ChangeStanceRefresh", SentryStance::ATTACK, [](BT::Blackboard::Ptr & bb) {
       bb->set<SentryStance>("current_stance", SentryStance::DEFEND);
     }});

  bool all_ok = true;
  for (size_t i = 0; i < scenarios.size(); ++i) {
    const bool is_refresh_case = (scenarios[i].name == "refresh_branch");
    if (is_refresh_case) {
      auto bb = makeDefaultBlackboard();
      scenarios[i].configure(bb);
      auto tree = factory.createTreeFromText(refresh_xml, bb);
      tree.tickRoot();

      auto final_action = bb->get<std::string>("last_stance_action");
      auto final_stance = bb->get<SentryStance>("current_stance");
      bool ok =
        (final_action == scenarios[i].expected_action) && (final_stance == scenarios[i].expected_stance);

      // CheckStanceRefreshRequired has static state. If first tick is a transition-reset tick,
      // run one more tick and evaluate again.
      if (!ok) {
        tree.tickRoot();
        final_action = bb->get<std::string>("last_stance_action");
        final_stance = bb->get<SentryStance>("current_stance");
        ok =
          (final_action == scenarios[i].expected_action) && (final_stance == scenarios[i].expected_stance);
      }

      if (ok) {
        RCLCPP_INFO(node->get_logger(),
          "[STANCE_TEST][PASS] %s -> %s",
          scenarios[i].name.c_str(),
          final_action.c_str());
      } else {
        RCLCPP_ERROR(node->get_logger(),
          "[STANCE_TEST][FAIL] %s expect(action=%s, stance=%d) got(action=%s, stance=%d)",
          scenarios[i].name.c_str(),
          scenarios[i].expected_action.c_str(),
          static_cast<int>(scenarios[i].expected_stance),
          final_action.c_str(),
          static_cast<int>(final_stance));
      }
      all_ok = all_ok && ok;
      continue;
    }
    const bool ok = runScenario(factory, base_xml, scenarios[i], node->get_logger());
    all_ok = all_ok && ok;
  }

  if (all_ok) {
    RCLCPP_INFO(node->get_logger(), "[STANCE_TEST] all scenarios passed");
  } else {
    RCLCPP_ERROR(node->get_logger(), "[STANCE_TEST] scenario test failed");
  }

  rclcpp::shutdown();
  return all_ok ? 0 : 1;
}
