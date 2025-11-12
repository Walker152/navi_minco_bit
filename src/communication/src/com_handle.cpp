#include "com_handle.hpp"
namespace ns_com
{
  BehavierTreeCom::BehavierTreeCom(const std::string& name)
    : Node(name)
  {
    Init();
    com_.init();
  }
  void BehavierTreeCom::Init()
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;

    chassis_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, [this](const geometry_msgs::msg::Twist::ConstSharedPtr& msg) { this->sendChassisCtrlCB(msg); });
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aft_mapped_to_init", 1, [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) { this->odomCB(msg); });
    gimbal_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/gimbal_yaw", 1, [this](const std_msgs::msg::Float32::ConstSharedPtr& msg) { this->desiredYawCB(msg); });
    outpost_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sentry/outpost_status", 1, [this](const std_msgs::msg::Bool::ConstSharedPtr& msg) { this->outpostCB(msg); });
    RCLCPP_INFO(this->get_logger(), "\nGot /cmd_vel message");
  }

  void BehavierTreeCom::sendChassisCtrlCB(const geometry_msgs::msg::Twist::ConstSharedPtr& velPtr)
  {
    // 速度
    cmd_vel_ = *velPtr;

    float vx_mps = cmd_vel_.linear.x;
    float vy_mps = cmd_vel_.linear.y;
    float vw_rpm = 0;
    // float vw_rpm = 90;
    float current_x = odom_.pose.pose.position.x;
    float current_y = odom_.pose.pose.position.y;
    float current_z = odom_.pose.pose.position.z;
    (void)current_z;
    // float vw_rpm = 30;
    // 计算雷达 yaw
    // 计算当前 yaw 发给 current_yaw
    tf2::Quaternion q;
    tf2::fromMsg(odom_.pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    auto current_yaw = static_cast<float>(yaw * 180.0 / M_PI);
    // 构造数据包 - Test
    // ChassisTarget target(1, 2, 3, 4, 5, 6,7);
    if(current_x > 8.0)
    {
      vw_rpm = 90;
    }
    if(std::sqrt((current_x - (0.0)) * (current_x - (0.0)) + (current_y - (-6.5)) * (current_y - (-6.5))) < 1.0)
    {
      vw_rpm = 0;
    }
    if(std::sqrt((current_x - (12.2)) * (current_x - (12.2)) + (current_y - (0.65)) * (current_y - (0.65))) < 1.0)
    {
      outpost_status_ = true;
    }
    ChassisTarget target(vx_mps,
                         vy_mps,
                         vw_rpm,
                         odom_.pose.pose.position.x,
                         odom_.pose.pose.position.y,
                         current_yaw,
                         gimbal_yaw_.data,
                         0,
                         outpost_status_);

    // 发送数据包
    com_.send2stm32(target);
    // std::cout << "vx: " << vx_mps << " vy: " << vy_mps << std::endl;
    // std::cout << "current_yaw: " << current_yaw << " gimbal_yaw: " << gimbal_yaw_.data << std::endl;
    std::cout << "vx: " << vx_mps << " vy: " << vy_mps << "  vw:" << vw_rpm << std::endl;
    std::cout << "is_aim_outpost: " << (outpost_status_ ? "true" : "false") << std::endl;
  }

  void BehavierTreeCom::odomCB(const nav_msgs::msg::Odometry::ConstSharedPtr& odomPtr)
  {
    odom_ = *odomPtr;
  }

  void BehavierTreeCom::desiredYawCB(const std_msgs::msg::Float32::ConstSharedPtr& yawPtr)
  {
    gimbal_yaw_ = *yawPtr;
  }

  void BehavierTreeCom::outpostCB(const std_msgs::msg::Bool::ConstSharedPtr& outpostPtr)
  {
    outpost_status_ = outpostPtr->data;
  }
}  // namespace ns_com
