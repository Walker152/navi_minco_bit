#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "robots_msgs/msg/event_status.hpp"
#include "bt_manager/blackboard.hpp"
namespace Sentry_BT
{   
    class ros_interface: public rclcpp::Node
    {
    private:
        rclcpp::Subscription<robots_msgs::msg::EventStatus>::SharedPtr event_sub;
        std::shared_ptr<Blackboard> blackboard_;

        void eventCallback(const robots_msgs::msg::EventStatus::SharedPtr msg);
    public:
        ros_interface(std::shared_ptr<Blackboard> blackboard_ptr);
        ~ros_interface();
    };
}
    