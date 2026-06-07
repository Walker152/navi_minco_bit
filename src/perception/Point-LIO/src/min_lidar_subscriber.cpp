#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <memory>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class MinLidarSubscriber : public rclcpp::Node
{
public:
  explicit MinLidarSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("min_lidar_subscriber", options)
  {
    lid_topic_ = declare_parameter<std::string>("lid_topic", "/livox/lidar");
    msg_type_ = declare_parameter<std::string>("msg_type", "pointcloud2");
    print_period_ = declare_parameter<double>("print_period", 1.0);
    qos_mode_ = declare_parameter<std::string>("qos_mode", "sensor");
    qos_depth_ = declare_parameter<int>("qos_depth", 5);

    std::transform(msg_type_.begin(), msg_type_.end(), msg_type_.begin(), ::tolower);
    std::transform(qos_mode_.begin(), qos_mode_.end(), qos_mode_.begin(), ::tolower);

    if (print_period_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "print_period must be positive, using 1.0s");
      print_period_ = 1.0;
    }
    if (qos_depth_ <= 0) {
      RCLCPP_WARN(get_logger(), "qos_depth must be positive, using 5");
      qos_depth_ = 5;
    }

    const auto qos = make_qos();

    if (msg_type_ == "custom") {
      custom_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lid_topic_, qos,
        [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
          handle_custom_msg(msg);
        });
      RCLCPP_INFO(
        get_logger(), "Subscribing CustomMsg topic=%s qos_mode=%s qos_depth=%d print_period=%.2fs",
        lid_topic_.c_str(), qos_mode_.c_str(), qos_depth_, print_period_);
    } else {
      if (msg_type_ != "pointcloud2") {
        RCLCPP_WARN(
          get_logger(), "Unsupported msg_type '%s', using pointcloud2", msg_type_.c_str());
        msg_type_ = "pointcloud2";
      }
      pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic_, qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          handle_pointcloud2_msg(msg);
        });
      RCLCPP_INFO(
        get_logger(), "Subscribing PointCloud2 topic=%s qos_mode=%s qos_depth=%d print_period=%.2fs",
        lid_topic_.c_str(), qos_mode_.c_str(), qos_depth_, print_period_);
    }

    window_start_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(print_period_)),
      [this]() { print_stats(); });
  }

private:
  rclcpp::QoS make_qos() const
  {
    if (qos_mode_ == "reliable") {
      rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)));
      qos.durability_volatile();
      qos.reliable();
      return qos;
    } else {
      if (qos_mode_ != "sensor") {
        RCLCPP_WARN(
          get_logger(), "Unsupported qos_mode '%s', using sensor/best_effort", qos_mode_.c_str());
      }
      rclcpp::SensorDataQoS qos;
      qos.keep_last(static_cast<size_t>(qos_depth_));
      return qos;
    }
  }

  void handle_pointcloud2_msg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    ++window_count_;

    const auto now = std::chrono::steady_clock::now();
    if (has_last_wall_time_) {
      last_wall_dt_ms_ = std::chrono::duration<double, std::milli>(now - last_wall_time_).count();
      wall_dt_sum_ms_ += last_wall_dt_ms_;
      ++wall_dt_count_;
    }
    last_wall_time_ = now;
    has_last_wall_time_ = true;

    const rclcpp::Time stamp(msg->header.stamp);
    if (has_last_stamp_) {
      last_stamp_dt_ms_ = static_cast<double>((stamp - last_stamp_).nanoseconds()) / 1.0e6;
    }
    last_stamp_ = stamp;
    has_last_stamp_ = true;

    last_delay_ms_ = 0.0;
    has_delay_ = false;
    try {
      last_delay_ms_ = static_cast<double>((get_clock()->now() - stamp).nanoseconds()) / 1.0e6;
      has_delay_ = true;
    } catch (const std::exception & ex) {
      RCLCPP_DEBUG(get_logger(), "Unable to compute header delay: %s", ex.what());
    }

    last_width_ = msg->width;
    last_height_ = msg->height;
    last_point_step_ = msg->point_step;
    last_bytes_ = msg->data.size();
  }

  void handle_custom_msg(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
  {
    ++window_count_;

    const auto now = std::chrono::steady_clock::now();
    if (has_last_wall_time_) {
      last_wall_dt_ms_ = std::chrono::duration<double, std::milli>(now - last_wall_time_).count();
      wall_dt_sum_ms_ += last_wall_dt_ms_;
      ++wall_dt_count_;
    }
    last_wall_time_ = now;
    has_last_wall_time_ = true;

    if (has_last_timebase_) {
      last_timebase_dt_ms_ =
        static_cast<double>(static_cast<int64_t>(msg->timebase - last_timebase_)) / 1.0e6;
    }
    last_timebase_ = msg->timebase;
    has_last_timebase_ = true;

    last_point_num_ = msg->point_num;
    last_points_size_ = msg->points.size();
    last_bytes_ = msg->points.size() * sizeof(livox_ros_driver2::msg::CustomPoint);
  }

  void print_stats()
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - window_start_).count();
    const double hz = elapsed > 0.0 ? static_cast<double>(window_count_) / elapsed : 0.0;
    const double mb = static_cast<double>(last_bytes_) / (1024.0 * 1024.0);
    const double wall_dt_avg = wall_dt_count_ > 0 ? wall_dt_sum_ms_ / wall_dt_count_ : 0.0;

    if (msg_type_ == "custom") {
      RCLCPP_INFO(
        get_logger(),
        "[MIN_SUB][CustomMsg] hz=%.2f count=%llu bytes=%.2fMB point_num=%u points=%zu "
        "timebase_dt=%.2fms wall_dt_avg=%.2fms wall_dt_last=%.2fms",
        hz, static_cast<unsigned long long>(window_count_), mb, last_point_num_,
        last_points_size_, last_timebase_dt_ms_, wall_dt_avg, last_wall_dt_ms_);
    } else {
      const char * delay_state = has_delay_ ? "" : " unavailable";
      RCLCPP_INFO(
        get_logger(),
        "[MIN_SUB][PointCloud2] hz=%.2f count=%llu bytes=%.2fMB width=%u height=%u "
        "point_step=%u stamp_dt=%.2fms wall_dt_avg=%.2fms wall_dt_last=%.2fms delay=%.2fms%s",
        hz, static_cast<unsigned long long>(window_count_), mb, last_width_, last_height_,
        last_point_step_, last_stamp_dt_ms_, wall_dt_avg, last_wall_dt_ms_, last_delay_ms_,
        delay_state);
    }

    window_count_ = 0;
    wall_dt_sum_ms_ = 0.0;
    wall_dt_count_ = 0;
    window_start_ = now;
  }

  std::string lid_topic_;
  std::string msg_type_;
  double print_period_{1.0};
  std::string qos_mode_;
  int qos_depth_{5};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point window_start_;
  uint64_t window_count_{0};
  double wall_dt_sum_ms_{0.0};
  uint64_t wall_dt_count_{0};
  std::chrono::steady_clock::time_point last_wall_time_;
  bool has_last_wall_time_{false};
  double last_wall_dt_ms_{0.0};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_last_stamp_{false};
  double last_stamp_dt_ms_{0.0};
  double last_delay_ms_{0.0};
  bool has_delay_{false};

  uint64_t last_timebase_{0};
  bool has_last_timebase_{false};
  double last_timebase_dt_ms_{0.0};

  uint32_t last_width_{0};
  uint32_t last_height_{0};
  uint32_t last_point_step_{0};
  uint32_t last_point_num_{0};
  size_t last_points_size_{0};
  size_t last_bytes_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinLidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
