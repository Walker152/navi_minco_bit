#include "bt_manager/ros_interface.hpp"
#include <cmath>
#include <chrono>
#include <cstdint>
#include <string>

namespace Sentry_BT
{
  ros_interface::ros_interface(std::shared_ptr<Blackboard>& blackboard_ptr)
    : Node("ros_interface_" +
               std::to_string(std::chrono::system_clock::now().time_since_epoch().count() % 10000),
           rclcpp::NodeOptions().use_global_arguments(false))
    , blackboard_(blackboard_ptr)
  {
    // 订阅事件状态话题
    event_sub = this->create_subscription<ros_interfaces::msg::EventStatus>(
        "/sentry/event_status",
        1,
        [this](const ros_interfaces::msg::EventStatus::SharedPtr msg) { this->eventCallback(msg); });

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/aft_mapped_to_init",
                                                                  1,
                                                                  [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                                  {
                                                                    // 更新当前位置
                                                                    std::lock_guard<std::mutex> lock(current_pose_mutex_);
                                                                    current_pose_ = msg->pose.pose;
                                                                  });
    // 订阅MPC轨迹指令
    mpc_cmd_sub = this->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
        "/opt_path", 1, [this](const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg)
        {
          //std::cout << "Received MPC command with horizon: " << msg->mpc_horizon << std::endl;
          blackboard_->set("through_tunnel", isTroughTunnel(msg, Point2D{9.46, 2.65}, Point2D{10.40, 1.80}));
        });
    // 订阅外部速度指令
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          blackboard_->set("cmd_vel", *msg);
        });
    // 定时发布行为状态（10Hz）
    gimbal_yaw_pub = this->create_publisher<std_msgs::msg::Float32>("/sentry/gimbal_yaw", 10);
    behavior_pub = this->create_publisher<ros_interfaces::msg::Behavior>("/sentry/behaivor_send", 10);
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
          const auto current_mode = blackboard_->get<int>("current_mode");
          const auto desired_stance = blackboard_->get<Sentry_BT::SentryStance>("desired_stance");
          const auto desired_lifter_pos = blackboard_->get<int>("desired_lifter_pos");
          const auto current_pose = getCurrentPose();

          const bool is_reach_outpost_enemy =
              current_mode == Sentry_BT::NavMode::RESPONSE &&
              std::hypot(current_pose.position.x - nav_points[2].x, current_pose.position.y - nav_points[2].y) < 1.0;

          const bool is_reach_outpost_own =
              std::hypot(current_pose.position.x - nav_points[0].x, current_pose.position.y - nav_points[0].y) < 1.0;

          blackboard_->set<bool>("outpost_msg", is_reach_outpost_enemy);

          ros_interfaces::msg::Behavior behavior_msg;
          behavior_msg.desired_stance = static_cast<int8_t>(desired_stance);
          behavior_msg.is_reach_outpost_enemy = is_reach_outpost_enemy;
          behavior_msg.is_reach_outpost_own = is_reach_outpost_own;
          behavior_msg.desire_lifter_pos = desired_lifter_pos;
          behavior_pub->publish(behavior_msg);
        });
  }

  void ros_interface::publishCmdVel(double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.y = linear_y;
    cmd_vel.angular.z = angular_z;
    cmd_vel_pub->publish(cmd_vel);
  }
  void ros_interface::publishCmdVel(const geometry_msgs::msg::Twist& cmd_vel)
  {
    cmd_vel_pub->publish(cmd_vel);
  }
  geometry_msgs::msg::Pose ros_interface::getCurrentPose() const
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    auto transforme_utils = std::make_shared<Sentry_BT::TransformUtils>();
    geometry_msgs::msg::Pose transformed_pose;
    if(transforme_utils->transformPoseToMap(current_pose_, transformed_pose, "camera_init"))
    {      
      return transformed_pose;
    }
    return current_pose_;
  }

  void ros_interface::eventCallback(const ros_interfaces::msg::EventStatus::SharedPtr msg)
  {
    // 更新黑板中的数据
    blackboard_->set<float>("health", ((int)msg->self_health / 4));
    blackboard_->set<int>("own_outpost_health", msg->own_outpost_health);
    blackboard_->set<bool>("enemy_outpost_destroyed", msg->enemy_outpost_destroyed);
    blackboard_->set<bool>("bonus_active", msg->buff_active);
    blackboard_->set<bool>("target_valid", msg->enemy_detected.is_detect);
    blackboard_->set<Sentry_BT::SentryStance>("current_stance", static_cast<Sentry_BT::SentryStance>(msg->current_stance));

    // 3月9日新增数据，不知道有什么用，先放在黑板里
    blackboard_->set<int>("game_status", static_cast<int>(msg->game_status));
    blackboard_->set<int>("lifter_pos_now", static_cast<int>(msg->lifter_pos_now));
    blackboard_->set<float>("gimbal_yaw", msg->gimbal_yaw);
    blackboard_->set<uint16_t>("num_shoot", msg->num_shoot);
    blackboard_->set<uint16_t>("hero_health", msg->hero_health);
    blackboard_->set<uint16_t>("infantry3_health", msg->infantry3_health);

    gimbal_yaw_pub->publish(std_msgs::msg::Float32().set__data(msg->gimbal_yaw));
    // 更新目标位置
    if(msg->enemy_detected.is_detect)
    // if(false)
    {
      geometry_msgs::msg::Pose target_pose_in, target_pose;
      static bool has_last_logged_pose = false;
      static geometry_msgs::msg::Pose last_target_pose_in;
      static geometry_msgs::msg::Pose last_target_pose;

      target_pose_in.position.x = (msg->enemy_detected.position.x) / 1000.0;  // 转换为米
      target_pose_in.position.y = (msg->enemy_detected.position.y) / 1000.0;
      target_pose_in.position.z = (msg->enemy_detected.position.z) / 1000.0;
      TransformPose(target_pose_in, target_pose);

      // Quiet logging: only print when target input/output pose changes significantly.
      const double input_diff = std::hypot(
          target_pose_in.position.x - last_target_pose_in.position.x,
          target_pose_in.position.y - last_target_pose_in.position.y);
      const double output_diff = std::hypot(
          target_pose.position.x - last_target_pose.position.x,
          target_pose.position.y - last_target_pose.position.y);
      const bool should_log = !has_last_logged_pose || input_diff > 0.5 || output_diff > 0.5;

      if(should_log)
      {
        // std::cout << "Target pose: " << target_pose.position.x << ", " << target_pose.position.y << std::endl;
        // std::cout << "target pose in: " << target_pose_in.position.x << "," << target_pose_in.position.y
        //           << "," << target_pose_in.position.z << std::endl;
        last_target_pose_in = target_pose_in;
        last_target_pose = target_pose;
        has_last_logged_pose = true;
      }

      target_pose.orientation.w = 1.0;  // 设置默认朝向
      blackboard_->set<int>("target_armor_id", (int)msg->enemy_detected.armor_id);
      blackboard_->set<geometry_msgs::msg::Pose>("target_pose", target_pose);
    }

  }

  bool ros_interface::TransformPose(const geometry_msgs::msg::Pose& input_pose, geometry_msgs::msg::Pose& output_pose)
  {
    // 创建TransformUtils实例
    auto transform_utils = std::make_shared<Sentry_BT::TransformUtils>();

    // 执行坐标转换
    bool success = transform_utils->transformPoseToMap(input_pose, output_pose, "gimbal");

    return success;
  }

  // 判断MPC轨迹是否穿过指定矩形区域
  bool ros_interface::isTroughZone(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Area_Square& zone)
  {
    for (const auto& cmd : msg->cmds)
    {
      Point2D point{cmd.position.x, cmd.position.y};
      if (zone.contains(point))
      {
        return true;
      }
    }
    return false;
  }

  // 判断MPC轨迹是否穿过指定隧道区域（由入口左端点和出口右端点两个点定义）
  bool ros_interface::isTroughTunnel(const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg, const Point2D& tunnel_entry, const Point2D& tunnel_exit)
  {
    Area_Square inflated_zone;
    Point2D point_of_robot{msg->cmds[0].position.x, msg->cmds[0].position.y};
    inflated_zone.top_left.x = std::min(tunnel_entry.x, tunnel_exit.x) - 0.9;  // 扩大一定的安全距离
    inflated_zone.top_left.y = std::max(tunnel_entry.y, tunnel_exit.y) + 1.2;
    inflated_zone.bottom_right.x = std::max(tunnel_entry.x, tunnel_exit.x) + 1.3;
    inflated_zone.bottom_right.y = std::min(tunnel_entry.y, tunnel_exit.y) - 0.66;
    bool flag1 = isTroughZone(msg, inflated_zone);
    bool flag2 = isTroughZone(msg, Area_Square{tunnel_entry, tunnel_exit});
    bool flag3 = inflated_zone.contains(point_of_robot);
    if (flag1)
    {
      // std::cout << "MPC trajectory is close to tunnel zone." << std::endl;
      if (flag2)
      {
        // std::cout << "MPC trajectory is through the tunnel." << std::endl;
        return true;
      }
      else if (flag3)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    return false;
  }
}  // namespace Sentry_BT