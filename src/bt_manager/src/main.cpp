#include "bt_manager/bt_manager.hpp"
#include "bt_manager/ros_interface.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_manager_node");

    auto blackboard = std::make_shared<Sentry_BT::Blackboard>();
    auto ros_interface_node = std::make_shared<Sentry_BT::ros_interface>(blackboard);
    Sentry_BT::BTManager bt_manager;
    std::string xml_file_path = "/home/alioth/2025-sentry-navi/src/bt_manager/config/sentry_behavior_tree.xml";
    if (!bt_manager.initialize(xml_file_path))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize BTManager with XML file: %s", xml_file_path.c_str());
        return -1;
    }

    rclcpp::Rate loop_rate(10); // 10 Hz
    while (rclcpp::ok())
    {
        bt_manager.execute();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
