#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include "robot_msgs/msg/test.hpp"
using namespace std::chrono_literals;
class pub_test : public rclcpp::Node
{
public:
    pub_test() : rclcpp::Node("pubinfo")
    {
        pub_ = this->create_publisher<robot_msgs::msg::Test>("/huati_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&pub_test::timerCB, this));
    }

private:
    rclcpp::Publisher<robot_msgs::msg::Test>::SharedPtr pub_;
    // create timer && timer callback
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    void timerCB()
    {
        robot_msgs::msg::Test msg;
        msg.age = 22;
        msg.name = "Fanyuchen";
        msg.height = 1.88;
        count_++;
        RCLCPP_INFO(this->get_logger(), "ok send!");
        pub_->publish(msg);
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pub_test>());
    return 0;
}
