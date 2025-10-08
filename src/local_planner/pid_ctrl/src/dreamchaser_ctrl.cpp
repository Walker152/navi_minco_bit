#include "dreamchaser_ctrl.hpp"

namespace dreamchaser_ctrl
{

  void PIDController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
                                std::string name,
                                const std::shared_ptr<tf2_ros::Buffer>& tf,
                                const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap_ros)
  {
    logger_ = parent.lock()->get_logger();

    RCLCPP_INFO(logger_, "Configuring PID Controller: %s", name.c_str());
  }

}  // namespace dreamchaser_ctrl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dreamchaser_ctrl::PIDController, nav2_core::Controller)