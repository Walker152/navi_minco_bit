#ifndef YHS_TWIST_CONVERTER__TWIST_TO_CTRL_CMD_NODE_HPP_
#define YHS_TWIST_CONVERTER__TWIST_TO_CTRL_CMD_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "mars_quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"

namespace yhs_twist_converter
{

  class TwistToCtrlCmdNode : public rclcpp::Node
  {
  public:
    explicit TwistToCtrlCmdNode();

  private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void position_command_callback(const mars_quadrotor_msgs::msg::PositionCommand::SharedPtr msg);

    // Publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<mars_quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_subscriber_;
    rclcpp::Publisher<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr ctrl_cmd_publisher_;

    // Parameters
    uint8_t default_gear_;  // 默认档位
    double max_linear_x_;   // 最大线性速度 x
    double max_linear_y_;   // 最大线性速度 y
    double max_angular_z_;  // 最大角速度 z
  };

}  // namespace yhs_twist_converter

#endif  // YHS_TWIST_CONVERTER__TWIST_TO_CTRL_CMD_NODE_HPP_