#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include "robot_msgs/msg/test.hpp"

class mynode : public rclcpp::Node
{
public:
    mynode() : Node("huatitongxin")
    {
        RCLCPP_INFO(this->get_logger(), "woshileigouzaode1");
        sub_ = this->create_subscription<robot_msgs::msg::Test>("/huati_topic", 10, std::bind(&mynode::defaultCB, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<robot_msgs::msg::Test>::SharedPtr sub_;
    void defaultCB(const robot_msgs::msg::Test::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "\nName:%s\nAge:%d\nHight:%.2f", msg->name.c_str(), msg->age, msg->height);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mynode>());
    return 0;
}
