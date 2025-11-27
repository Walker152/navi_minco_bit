#ifndef DREAMCHASER_CTRL_HPP
#define DREAMCHASER_CTRL_HPP
#include "pid.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"             // Nav2控制器基类接口
#include "nav2_costmap_2d/costmap_2d_ros.hpp"   // 代价地图
#include "rclcpp_lifecycle/lifecycle_node.hpp"  // 生命周期节点
#include "tf2_ros/buffer.h"                     // TF2缓冲区
#include "visualization_msgs/msg/marker_array.hpp"
#include "pid.hpp"
namespace dreamchaser_ctrl
{
  class PIDController : public nav2_core::Controller
  {
  public:
    //
    PIDController() = default;
    ~PIDController() override = default;
    // 配置生命周期节点、名称、TF缓冲区和代价地图
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                   std::string name,
                   const std::shared_ptr<tf2_ros::Buffer>& tf,
                   const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    // core function
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                             const geometry_msgs::msg::TwistStamped& velocity,
                                                             const nav_msgs::msg::Path& path) override;

    bool setPlan(const nav_msgs::msg::Path& path) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

    private:
    rclcpp::Logger logger_{rclcpp::get_logger("PIDController")};
    PIDParams pid_params_;
    PIDSolver pid_solver_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::string plugin_name_;
    nav_msgs::msg::Path global_plan_;
    double speed_limit_{0.0};
    bool percentage_{false};
    std::mutex mutex_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  };

}  // namespace dreamchaser_ctrl
#endif  // DREAMCHASER_CTRL_HPP