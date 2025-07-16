#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
class lister : public rclcpp::Node
{
public:
    lister() : rclcpp::Node("listener")
    {
        buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        listener_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&lister::timerCB,
                      this));
    }

private:
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    rclcpp::TimerBase::SharedPtr listener_timer_;
    void timerCB()
    {
        try
        {
            auto tf_info = buffer_->lookupTransform("lidar", "base_link", tf2::TimePointZero);
            tf2::Quaternion qtn;
            RCLCPP_INFO(this->get_logger(), "\nframe ID:%s\nchild ID:%s\nxyz:%.2f  %.2f  %.2f\nqtn:%.2f  %.2f  %.2f %.2f", 
            tf_info.header.frame_id.c_str(),tf_info.child_frame_id.c_str(),
            tf_info.transform.translation.x,tf_info.transform.translation.y,tf_info.transform.translation.z,
            tf_info.transform.rotation.x,tf_info.transform.rotation.y,tf_info.transform.rotation.z,tf_info.transform.rotation.w);
        }
        catch(const tf2::LookupException& e)
        {
            RCLCPP_WARN(this->get_logger(),"get:%s",e.what());
        }
        
      
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lister>());
    return 0;
}
