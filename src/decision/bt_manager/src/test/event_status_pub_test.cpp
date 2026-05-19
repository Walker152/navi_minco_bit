#include <chrono>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_interfaces/msg/behavior.hpp>
#include <ros_interfaces/msg/game_info.hpp>
#include <ros_interfaces/msg/sentry_info_offline.hpp>
#include <ros_interfaces/msg/sentry_info_online.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace {
constexpr double kScenarioPeriodSeconds = 10.0;
constexpr std::uint8_t kDefaultLifterPos = 0;
constexpr std::uint8_t kDefaultCapacitor = 100;
constexpr float kDefaultYaw = 0.0f;
constexpr std::uint16_t kHealthScale = 4;
constexpr std::uint8_t kDefaultArmorId = 1;
constexpr std::uint8_t kDefaultGameStatus = 4;
constexpr std::uint16_t kDefaultGameTime = 420;
}  // namespace

struct ScenarioStep
{
  std::string name;
  std::string description;
  std::string branch_hint;
  float health;
  int bullets;
  geometry_msgs::msg::Point sentry_pos;
  bool target_valid;
  geometry_msgs::msg::Point target_pos;
  std::uint8_t lifter_pos;
  std::uint8_t capacitor;
  int current_heat;
  bool is_disengaged;
};

class ScenarioPublisherNode : public rclcpp::Node
{
public:
  ScenarioPublisherNode()
  : Node("sentry_scenario_publisher"), last_desired_stance_(ros_interfaces::msg::Behavior::STANCE_MOVE)
  {
    game_pub_ = create_publisher<ros_interfaces::msg::GameInfo>("/sentry/game_info", 10);
    offline_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOffline>("/sentry/offline_info", 10);
    online_pub_ = create_publisher<ros_interfaces::msg::SentryInfoOnline>("/sentry/online_info", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);

    auto behavior_cb = [this](const ros_interfaces::msg::Behavior::SharedPtr msg) {
      if (msg->desired_stance != 0U) {
        last_desired_stance_ = msg->desired_stance;
      }
    };
    behavior_sub_ =
      create_subscription<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10, behavior_cb);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    buildScenarioSteps();

    publish_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      publishCurrentScenario();
      publishTfForScenario();
    });

    odom_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      publishZeroOdom();
    });

    scenario_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                          std::chrono::duration<double>(kScenarioPeriodSeconds)),
      [this]() {
        advanceScenario();
      });

    RCLCPP_INFO(get_logger(), "Scenario publisher started with %zu steps.", steps_.size());
    logCurrentScenario();
  }

private:
  void buildScenarioSteps()
  {
    steps_.clear();
    steps_.push_back(makeStep("1A_DefenseFar",
      "场景1: 前哨站未摧毁, 还在己方防守区, 距离隧道较远",
      "导航: 特殊响应->前哨站, 姿态: 防御/攻击+小陀螺",
      100.0f,
      300,
      5.0,
      7.0,
      false,
      0.0,
      0.0,
      kDefaultLifterPos));
    steps_.push_back(makeStep("1B_TransformZone",
      "场景1: 进入变形区, 期望云台下降, 关闭小陀螺",
      "导航: 通过隧道, 姿态: 进入非小陀螺控制",
      100.0f,
      300,
      8.5,
      4.5,
      false,
      0.0,
      0.0,
      1));
    steps_.push_back(makeStep("1C_Tunnel",
      "场景1: 进入隧道区域, PID 接管",
      "姿态: 小陀螺禁止被其他条件抢占",
      100.0f,
      300,
      1.0,
      2.7,
      true,
      2.0,
      2.0,
      1));

    steps_.push_back(makeStep("2A_Outpost",
      "场景2: 到达前哨站区域, 姿态应为攻击",
      "导航: 前哨站目标, 姿态: ATTACK",
      100.0f,
      300,
      15.0,
      11.0,
      false,
      0.0,
      0.0,
      kDefaultLifterPos));
    steps_.push_back(makeStep("2B_Hit",
      "场景2: 血量下降触发冷却, 进入巡逻",
      "导航: 响应->巡逻, 姿态: 冷却内保持",
      90.0f,
      300,
      15.0,
      11.0,
      false,
      0.0,
      0.0,
      kDefaultLifterPos));
    steps_.push_back(makeStep("2C_CooldownEnd",
      "场景2: 5s 无下降, 冷却结束恢复前哨站响应",
      "导航: 回到前哨站响应",
      90.0f,
      300,
      15.0,
      11.0,
      false,
      0.0,
      0.0,
      kDefaultLifterPos));

    steps_.push_back(makeStep("3A_RetreatTransform",
      "场景3: 血量20且弹量0, 回家补血, 进入变形区",
      "导航: RETREAT, 姿态: 隧道前关闭小陀螺",
      20.0f,
      0,
      15.0,
      13.0,
      false,
      0.0,
      0.0,
      1));
    steps_.push_back(makeStep("3B_RetreatTunnel",
      "场景3: 进入隧道, PID 接管",
      "姿态: 非小陀螺控制不应被抢占",
      20.0f,
      0,
      13.1,
      13.4,
      false,
      0.0,
      0.0,
      1));
    steps_.push_back(makeStep(
      "3C_Home", "场景3: 回家位置", "导航: HOME", 20.0f, 0, 3.0, 3.0, false, 0.0, 0.0, kDefaultLifterPos));

    steps_.push_back(makeStep("4_Recover",
      "场景4: 血量恢复且弹量恢复, 继续打前哨站",
      "导航: 前哨站响应, 姿态随条件切换",
      100.0f,
      200,
      5.0,
      7.0,
      false,
      0.0,
      0.0,
      kDefaultLifterPos));

    steps_.push_back(makeStep("5A_TrackingDefense",
      "场景5: 己方防守区触发追踪",
      "导航: 追击抢占",
      100.0f,
      200,
      7.0,
      7.0,
      true,
      8.0,
      2.0,
      kDefaultLifterPos));
    steps_.push_back(makeStep("5B_TrackingHighland",
      "场景5: 高地追踪区域触发",
      "导航: 追击抢占",
      100.0f,
      200,
      13.0,
      8.0,
      true,
      14.0,
      9.0,
      kDefaultLifterPos));

    steps_.push_back(makeStep("6_TunnelTracking",
      "场景6: 隧道内追踪, 不允许其他姿态抢占小陀螺控制",
      "姿态: 非小陀螺控制锁定",
      100.0f,
      200,
      1.0,
      2.7,
      true,
      2.0,
      2.0,
      1));
  }

  ScenarioStep makeStep(const std::string & name,
    const std::string & description,
    const std::string & branch_hint,
    float health,
    int bullets,
    double pos_x,
    double pos_y,
    bool target_valid,
    double target_x,
    double target_y,
    std::uint8_t lifter_pos)
  {
    ScenarioStep step;
    step.sentry_pos = geometry_msgs::msg::Point();
    step.target_pos = geometry_msgs::msg::Point();
    step.name = name;
    step.description = description;
    step.branch_hint = branch_hint;
    step.health = health;
    step.bullets = bullets;
    step.sentry_pos.x = pos_x;
    step.sentry_pos.y = pos_y;
    step.sentry_pos.z = 0.0;
    step.target_valid = target_valid;
    step.target_pos.x = target_x;
    step.target_pos.y = target_y;
    step.target_pos.z = 0.0;
    step.lifter_pos = lifter_pos;
    step.capacitor = kDefaultCapacitor;
    step.current_heat = 0;
    step.is_disengaged = !target_valid;
    return step;
  }

  void advanceScenario()
  {
    if (steps_.empty()) {
      return;
    }
    current_index_ = (current_index_ + 1) % steps_.size();
    logCurrentScenario();
  }

  void logCurrentScenario() const
  {
    if (steps_.empty()) {
      return;
    }
    const auto & step = steps_[current_index_];
    RCLCPP_INFO(
      get_logger(), "切换场景 [%zu/%zu]: %s", current_index_ + 1, steps_.size(), step.name.c_str());
    RCLCPP_INFO(get_logger(), "说明: %s", step.description.c_str());
    RCLCPP_INFO(get_logger(), "分支提示: %s", step.branch_hint.c_str());
    if (current_index_ > 0) {
      const auto & prev = steps_[current_index_ - 1];
      const auto diff = formatScenarioDiff(prev, step);
      if (!diff.empty()) {
        RCLCPP_INFO(get_logger(), "条件变化: %s", diff.c_str());
      }
    }
  }

  std::string formatScenarioDiff(const ScenarioStep & before, const ScenarioStep & after) const
  {
    std::ostringstream oss;
    bool first = true;
    auto append = [&](const std::string & text) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << text;
    };

    if (before.health != after.health) {
      append("health=" + std::to_string(before.health) + "->" + std::to_string(after.health));
    }
    if (before.bullets != after.bullets) {
      append("bullets=" + std::to_string(before.bullets) + "->" + std::to_string(after.bullets));
    }
    if (before.sentry_pos.x != after.sentry_pos.x || before.sentry_pos.y != after.sentry_pos.y) {
      std::ostringstream pos;
      pos << "pos=(" << before.sentry_pos.x << "," << before.sentry_pos.y << ")->(" << after.sentry_pos.x
          << "," << after.sentry_pos.y << ")";
      append(pos.str());
    }
    if (before.target_valid != after.target_valid) {
      append(std::string("target_valid=") + (before.target_valid ? "true" : "false") + "->" +
             (after.target_valid ? "true" : "false"));
    }
    if (before.target_pos.x != after.target_pos.x || before.target_pos.y != after.target_pos.y) {
      std::ostringstream tpos;
      tpos << "target=(" << before.target_pos.x << "," << before.target_pos.y << ")->("
           << after.target_pos.x << "," << after.target_pos.y << ")";
      append(tpos.str());
    }
    if (before.lifter_pos != after.lifter_pos) {
      append("lifter_pos=" + std::to_string(before.lifter_pos) + "->" + std::to_string(after.lifter_pos));
    }
    return oss.str();
  }

  void publishCurrentScenario()
  {
    if (steps_.empty()) {
      return;
    }

    const auto & step = steps_[current_index_];
    const auto now = this->now();

    ros_interfaces::msg::GameInfo game_msg;
    game_msg.header.stamp = now;
    game_msg.game_time_remaining = kDefaultGameTime;
    game_msg.coin_remaining = 0;
    game_msg.event_code = 0;
    game_msg.game_status = kDefaultGameStatus;
    game_pub_->publish(game_msg);

    ros_interfaces::msg::SentryInfoOffline offline_msg;
    offline_msg.header.stamp = now;
    offline_msg.is_get = step.target_valid;
    offline_msg.armor_pos.x = static_cast<float>(step.target_pos.x * 1000.0);
    offline_msg.armor_pos.y = static_cast<float>(step.target_pos.y * 1000.0);
    offline_msg.armor_pos.z = static_cast<float>(step.target_pos.z * 1000.0);
    offline_msg.armor_num = kDefaultArmorId;
    offline_msg.yaw_camerainit_to_gimbal = kDefaultYaw;
    offline_msg.lifter_current_pos = step.lifter_pos;
    offline_msg.is_transformable = true;
    offline_msg.transform_state = 0.0f;
    offline_msg.capacitor_capacity = step.capacitor;
    offline_pub_->publish(offline_msg);

    ros_interfaces::msg::SentryInfoOnline online_msg;
    online_msg.header.stamp = now;
    online_msg.self_health = static_cast<std::uint16_t>(step.health * kHealthScale);
    online_msg.bullets_remaining = static_cast<std::uint16_t>(step.bullets);
    online_msg.cooling_value = 0;
    online_msg.heat_limit = 0;
    online_msg.current_heat = static_cast<std::uint16_t>(step.current_heat);
    online_msg.sentry_pos = step.sentry_pos;
    online_msg.speed_monitor_angle = 0.0f;
    online_msg.sentry_info_1 = 0;
    online_msg.sentry_info_2 = buildSentryInfo2(step.is_disengaged);
    online_pub_->publish(online_msg);
  }

  void publishTfForScenario()
  {
    if (steps_.empty()) {
      return;
    }

    const auto & step = steps_[current_index_];
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "camera_init";
    tf_msg.transform.translation.x = step.sentry_pos.x;
    tf_msg.transform.translation.y = step.sentry_pos.y;
    tf_msg.transform.translation.z = step.sentry_pos.z;

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    tf_msg.transform.rotation = tf2::toMsg(quat);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishZeroOdom()
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.w = 1.0;
    odom_pub_->publish(odom_msg);
  }

  std::uint16_t buildSentryInfo2(bool is_disengaged) const
  {
    std::uint16_t info = 0;
    if (is_disengaged) {
      info |= 0x0001;
    }

    const std::uint8_t stance = normalizeStance(last_desired_stance_);
    info |= static_cast<std::uint16_t>((stance & 0x03) << 12);
    return info;
  }

  std::uint8_t normalizeStance(std::uint8_t stance) const
  {
    if (stance < ros_interfaces::msg::Behavior::STANCE_ATTACK ||
        stance > ros_interfaces::msg::Behavior::STANCE_MOVE) {
      return ros_interfaces::msg::Behavior::STANCE_MOVE;
    }
    return stance;
  }

  rclcpp::Publisher<ros_interfaces::msg::GameInfo>::SharedPtr game_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOffline>::SharedPtr offline_pub_;
  rclcpp::Publisher<ros_interfaces::msg::SentryInfoOnline>::SharedPtr online_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<ros_interfaces::msg::Behavior>::SharedPtr behavior_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr scenario_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;

  std::vector<ScenarioStep> steps_;
  std::size_t current_index_ = 0;
  std::uint8_t last_desired_stance_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScenarioPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
