#include "yhs_twist_converter/twist_to_ctrl_cmd_node.hpp"
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include <algorithm>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

namespace yhs_twist_converter
{

  TwistToCtrlCmdNode::TwistToCtrlCmdNode()
    : Node("twist_to_ctrl_cmd_node")
  {
    // 声明参数并设置默认值
    this->declare_parameter<int>("default_gear", 8);
    this->declare_parameter<double>("max_linear_x", 2.0);
    this->declare_parameter<double>("max_linear_y", 2.0);
    this->declare_parameter<double>("max_angular_z", 2.0);

    // 获取参数值
    default_gear_ = static_cast<uint8_t>(this->get_parameter("default_gear").as_int());
    max_linear_x_ = this->get_parameter("max_linear_x").as_double();
    max_linear_y_ = this->get_parameter("max_linear_y").as_double();
    max_angular_z_ = this->get_parameter("max_angular_z").as_double();

    // 创建订阅者，订阅 cmd_vel 话题 (标准的Twist消息话题)
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&TwistToCtrlCmdNode::twist_callback, this, std::placeholders::_1));

    // 订阅 /planning/pos_cmd (mars_quadrotor_msgs/PositionCommand)
    pos_cmd_subscriber_ = this->create_subscription<mars_quadrotor_msgs::msg::PositionCommand>(
        "/planning/pos_cmd",
        10,
        std::bind(&TwistToCtrlCmdNode::position_command_callback, this, std::placeholders::_1));

    // 创建发布者，发布到 ctrl_cmd 话题
    ctrl_cmd_publisher_ = this->create_publisher<yhs_can_interfaces::msg::CtrlCmd>("ctrl_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "TwistToCtrlCmd节点已启动");
    RCLCPP_INFO(this->get_logger(), "模式: %d", default_gear_);
    RCLCPP_INFO(this->get_logger(), "最大线速度 x: %.2f m/s", max_linear_x_);
    RCLCPP_INFO(this->get_logger(), "最大线速度 y: %.2f m/s", max_linear_y_);
    RCLCPP_INFO(this->get_logger(), "最大角速度 z: %.2f rad/s", max_angular_z_);
  }

  void TwistToCtrlCmdNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // 创建 CtrlCmd 消息
    auto ctrl_cmd_msg = yhs_can_interfaces::msg::CtrlCmd();

    // 设置档位
    ctrl_cmd_msg.ctrl_cmd_gear = default_gear_;

    // 限制和转换线性速度
    ctrl_cmd_msg.ctrl_cmd_x_linear = std::clamp(
        static_cast<float>(msg->linear.x), static_cast<float>(-max_linear_x_), static_cast<float>(max_linear_x_));

    ctrl_cmd_msg.ctrl_cmd_y_linear = std::clamp(
        static_cast<float>(msg->linear.y), static_cast<float>(-max_linear_y_), static_cast<float>(max_linear_y_));

    // 限制和转换角速度
    ctrl_cmd_msg.ctrl_cmd_z_angular = std::clamp(
        static_cast<float>(msg->angular.z), static_cast<float>(-max_angular_z_), static_cast<float>(max_angular_z_));

    // 发布转换后的消息
    ctrl_cmd_publisher_->publish(ctrl_cmd_msg);

    // 打印调试信息（可选）
    RCLCPP_INFO(this->get_logger(),
                "转换 Twist -> CtrlCmd: gear=%d, x=%.3f, y=%.3f, z=%.3f",
                ctrl_cmd_msg.ctrl_cmd_gear,
                ctrl_cmd_msg.ctrl_cmd_x_linear,
                ctrl_cmd_msg.ctrl_cmd_y_linear,
                ctrl_cmd_msg.ctrl_cmd_z_angular);
  }

  void TwistToCtrlCmdNode::position_command_callback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    auto ctrl_cmd_msg = yhs_can_interfaces::msg::CtrlCmd();
    ctrl_cmd_msg.ctrl_cmd_gear = default_gear_;

    // 线速度取 velocity.x / velocity.y
    ctrl_cmd_msg.ctrl_cmd_x_linear = std::clamp(
        static_cast<float>(msg->velocity.x), static_cast<float>(-max_linear_x_), static_cast<float>(max_linear_x_));
    ctrl_cmd_msg.ctrl_cmd_y_linear = std::clamp(
        static_cast<float>(msg->velocity.y), static_cast<float>(-max_linear_y_), static_cast<float>(max_linear_y_));

    // 角速度: 优先 angular_velocity.z，若为0用 yaw_dot
    double angular_z = (msg->angular_velocity.z != 0.0) ? msg->angular_velocity.z : msg->yaw_dot;
    ctrl_cmd_msg.ctrl_cmd_z_angular = std::clamp(
        static_cast<float>(angular_z), static_cast<float>(-max_angular_z_), static_cast<float>(max_angular_z_));

    ctrl_cmd_publisher_->publish(ctrl_cmd_msg);
    RCLCPP_INFO(this->get_logger(),
                "转换 PositionCommand -> CtrlCmd: gear=%d, vx=%.3f, vy=%.3f, wz=%.3f (traj_flag=%u)",
                ctrl_cmd_msg.ctrl_cmd_gear,
                ctrl_cmd_msg.ctrl_cmd_x_linear,
                ctrl_cmd_msg.ctrl_cmd_y_linear,
                ctrl_cmd_msg.ctrl_cmd_z_angular,
                msg->trajectory_flag);
  }

}  // namespace yhs_twist_converter

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<yhs_twist_converter::TwistToCtrlCmdNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}