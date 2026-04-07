//
// Created by adam on 18-10-8.
//

#include "ros_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DBSCANCluster::RosInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}