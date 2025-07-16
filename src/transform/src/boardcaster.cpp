#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
class tfnode : public rclcpp::Node
{
public:
    tfnode() : rclcpp::Node("boardcaster")
    {
        tfpub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&tfnode::tf_timerCB, this));
        RCLCPP_INFO(this->get_logger(),"TF start!");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfpub_;
    void tf_timerCB()
    {
        auto now = this->now();
        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = now;
        double time_second = now.seconds();
        msg.header.frame_id = "lidar";
        msg.child_frame_id = "base_link";
        msg.transform.translation.x = -0.12;
        msg.transform.translation.y = 0.0;
        msg.transform.translation.z = 0.0;
        tf2::Quaternion qtn;
        qtn.setRPY(0.0,0.0, time_second*3.14);
        msg.transform.rotation.x = qtn.getX();
        msg.transform.rotation.y = qtn.getY();
        msg.transform.rotation.z = qtn.getZ();
        msg.transform.rotation.w = qtn.getW();
        tfpub_->sendTransform(msg);
    }
    rclcpp::TimerBase::SharedPtr tf_timer_;
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tfnode>());

    return 0;
}
