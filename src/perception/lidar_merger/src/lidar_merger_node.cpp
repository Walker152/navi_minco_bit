#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sensor_msgs/msg/point_field.hpp>

#include <std_msgs/msg/header.hpp>

#include "lidar_merger/lidar_merger_node.hpp"

using namespace std::placeholders;

LidarMergerNode::LidarMergerNode() : LidarMergerNode(rclcpp::NodeOptions())
{
}

LidarMergerNode::LidarMergerNode(const rclcpp::NodeOptions & options) : Node("lidar_merger_node", options)
{
  this->declare_parameter<std::string>("front_topic", "/livox/lidar_192_168_1_10");
  this->declare_parameter<std::string>("back_topic", "/livox/lidar_192_168_1_12");
  this->declare_parameter<std::string>("merged_topic", "/livox/lidar_merged");
  this->declare_parameter<std::string>("merged_frame_id", "livox_front_frame");
  this->declare_parameter<int>("qos_depth", 10);
  this->declare_parameter<bool>("best_effort", true);
  this->declare_parameter<int>("sync_queue_size", 20);
  this->declare_parameter<double>("max_sync_interval_ms", 5.0);
  this->declare_parameter<bool>("publish_pointcloud", false);
  this->declare_parameter<std::string>("front_cloud_topic", "/livox/front_cloud");
  this->declare_parameter<std::string>("back_cloud_topic", "/livox/back_cloud");
  this->declare_parameter<std::string>("merged_cloud_topic", "/livox/lidar_merged_cloud");
  this->declare_parameter("extrinsic_back_to_front", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  loadParams();
  loadExtrinsics();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_)).durability_volatile();
  if (best_effort_) {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  sub_front_.subscribe(this, front_topic_, qos.get_rmw_qos_profile());
  sub_back_.subscribe(this, back_topic_, qos.get_rmw_qos_profile());

  sync_ = std::make_shared<Synchronizer>(SyncPolicy(sync_queue_size_), sub_front_, sub_back_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_ms_ / 1000.0));
  sync_->registerCallback(std::bind(&LidarMergerNode::syncCallback, this, _1, _2));

  pub_merged_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(merged_topic_, qos);

  if (publish_pointcloud_) {
    pub_front_cloud_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(front_cloud_topic_, qos);
    pub_back_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(back_cloud_topic_, qos);
    pub_merged_cloud_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_cloud_topic_, qos);
  }

  RCLCPP_INFO(this->get_logger(), "双雷达融合节点已启动。目标坐标系：主雷达 (前雷达)。");
  RCLCPP_INFO(this->get_logger(), "front_topic: %s", front_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "back_topic:  %s", back_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "merged_topic:%s", merged_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "merged_frame_id: %s", merged_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_pointcloud: %s", publish_pointcloud_ ? "true" : "false");
  if (publish_pointcloud_) {
    RCLCPP_INFO(this->get_logger(), "front_cloud_topic: %s", front_cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "back_cloud_topic:  %s", back_cloud_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "merged_cloud_topic:%s", merged_cloud_topic_.c_str());
  }
  RCLCPP_INFO(this->get_logger(),
    "qos_depth=%d best_effort=%s sync_queue_size=%d",
    qos_depth_,
    best_effort_ ? "true" : "false",
    sync_queue_size_);
}

void LidarMergerNode::loadParams()
{
  front_topic_ = this->get_parameter("front_topic").as_string();
  back_topic_ = this->get_parameter("back_topic").as_string();
  merged_topic_ = this->get_parameter("merged_topic").as_string();
  merged_frame_id_ = this->get_parameter("merged_frame_id").as_string();
  qos_depth_ = this->get_parameter("qos_depth").as_int();
  best_effort_ = this->get_parameter("best_effort").as_bool();
  sync_queue_size_ = this->get_parameter("sync_queue_size").as_int();
  max_sync_interval_ms_ = this->get_parameter("max_sync_interval_ms").as_double();
  publish_pointcloud_ = this->get_parameter("publish_pointcloud").as_bool();
  front_cloud_topic_ = this->get_parameter("front_cloud_topic").as_string();
  back_cloud_topic_ = this->get_parameter("back_cloud_topic").as_string();
  merged_cloud_topic_ = this->get_parameter("merged_cloud_topic").as_string();

  if (qos_depth_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "qos_depth=%d 非法，已重置为 10", qos_depth_);
    qos_depth_ = 10;
  }
  if (sync_queue_size_ <= 0) {
    RCLCPP_WARN(this->get_logger(), "sync_queue_size=%d 非法，已重置为 20", sync_queue_size_);
    sync_queue_size_ = 20;
  }
  if (max_sync_interval_ms_ <= 0.0) {
    max_sync_interval_ms_ = 5.0;
  }
  if (merged_frame_id_.empty()) {
    merged_frame_id_ = "livox_front_frame";
  }
  if (front_cloud_topic_.empty()) {
    front_cloud_topic_ = "/livox/front_cloud";
  }
  if (back_cloud_topic_.empty()) {
    back_cloud_topic_ = "/livox/back_cloud";
  }
  if (merged_cloud_topic_.empty()) {
    merged_cloud_topic_ = "/livox/lidar_merged_cloud";
  }
}

void LidarMergerNode::loadExtrinsics()
{
  extrinsic_back_to_front_ = this->get_parameter("extrinsic_back_to_front").as_double_array();
  if (extrinsic_back_to_front_.size() != 6) {
    RCLCPP_ERROR(this->get_logger(),
      "参数 extrinsic_back_to_front 期望长度为 6，但实际为 %zu。将使用单位外参。",
      extrinsic_back_to_front_.size());
    T_front_back_ = Eigen::Matrix4f::Identity();
    R_front_back_ = Eigen::Matrix3f::Identity();
    t_front_back_ = Eigen::Vector3f::Zero();
    return;
  }

  const auto & p = extrinsic_back_to_front_;

  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation() << static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]);

  // Roll-Pitch-Yaw: X-Y-Z (intrinsic). Apply in Z(Yaw) -> Y(Pitch) -> X(Roll) order.
  t.rotate(Eigen::AngleAxisf(static_cast<float>(p[5]), Eigen::Vector3f::UnitZ()));
  t.rotate(Eigen::AngleAxisf(static_cast<float>(p[4]), Eigen::Vector3f::UnitY()));
  t.rotate(Eigen::AngleAxisf(static_cast<float>(p[3]), Eigen::Vector3f::UnitX()));

  T_front_back_ = t.matrix();
  R_front_back_ = t.rotation();
  t_front_back_ = t.translation();

  RCLCPP_INFO(this->get_logger(),
    "已加载外参 back->front: [%.3f %.3f %.3f %.3f %.3f %.3f]",
    p[0],
    p[1],
    p[2],
    p[3],
    p[4],
    p[5]);
}

sensor_msgs::msg::PointCloud2 LidarMergerNode::customMsgToPointCloud2(
  const livox_ros_driver2::msg::CustomMsg & msg,
  const std_msgs::msg::Header & header,
  const Eigen::Matrix4f * transform)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.stamp = header.stamp;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4,
    "x",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "y",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "z",
    1,
    sensor_msgs::msg::PointField::FLOAT32,
    "intensity",
    1,
    sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(msg.points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

  for (const auto & pt : msg.points) {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    if (transform != nullptr) {
      p = transform->block<3, 3>(0, 0) * p + transform->block<3, 1>(0, 3);
    }

    *iter_x = p.x();
    *iter_y = p.y();
    *iter_z = p.z();
    *iter_intensity = static_cast<float>(pt.reflectivity);

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
  }

  return cloud;
}

inline livox_ros_driver2::msg::CustomPoint LidarMergerNode::transformPoint(
  const livox_ros_driver2::msg::CustomPoint & pt_in, const Eigen::Matrix3f & R, const Eigen::Vector3f & t)
{
  livox_ros_driver2::msg::CustomPoint pt_out = pt_in;

  const Eigen::Vector3f p_in(pt_in.x, pt_in.y, pt_in.z);
  const Eigen::Vector3f p_out = (R * p_in) + t;

  pt_out.x = p_out.x();
  pt_out.y = p_out.y();
  pt_out.z = p_out.z();

  return pt_out;
}

void LidarMergerNode::syncCallback(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_front,
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr & msg_back)
{
  livox_ros_driver2::msg::CustomMsg msg_merged = *msg_front;
  msg_merged.header.frame_id = merged_frame_id_;

  uint64_t min_timebase = 0.0;
  if (msg_front->timebase <= msg_back->timebase) {
    min_timebase = msg_front->timebase;
    msg_merged.header = msg_front->header;
  } else {
    min_timebase = msg_back->timebase;
    msg_merged.header = msg_back->header;
  }
  msg_merged.timebase = min_timebase;

  const size_t n_front = msg_front->points.size();
  const size_t n_back = msg_back->points.size();

  // 2. 预分配空间，避免 push_back 时的动态扩容，同时杜绝 resize 的无意义初始化
  msg_merged.points.reserve(n_front + n_back);

  // 3. 时间戳常数提取
  const uint32_t dt_front = static_cast<uint32_t>(msg_front->timebase - min_timebase);
  const uint32_t dt_back = static_cast<uint32_t>(msg_back->timebase - min_timebase);

  // 4. 前雷达时间补偿 (仅当主雷达时间不是基准时间时才遍历，通常 dt_front 为 0，直接 O(1) 跳过)
  if (dt_front > 0) {
    for (auto & pt : msg_merged.points) {
      pt.offset_time += dt_front;
    }
  }

  // 5. 后雷达点云变换与融合 (单线程，去除非必要的中间变量分配)
  for (const auto & pt_in : msg_back->points) {
    livox_ros_driver2::msg::CustomPoint pt_out = pt_in;  // 拷贝 intensity, tag 等非空间属性

    // 内联的矩阵乘加操作，避免调用任何耗时函数
    pt_out.x = R_front_back_(0, 0) * pt_in.x + R_front_back_(0, 1) * pt_in.y +
               R_front_back_(0, 2) * pt_in.z + t_front_back_(0);
    pt_out.y = R_front_back_(1, 0) * pt_in.x + R_front_back_(1, 1) * pt_in.y +
               R_front_back_(1, 2) * pt_in.z + t_front_back_(1);
    pt_out.z = R_front_back_(2, 0) * pt_in.x + R_front_back_(2, 1) * pt_in.y +
               R_front_back_(2, 2) * pt_in.z + t_front_back_(2);

    pt_out.offset_time += dt_back;

    msg_merged.points.push_back(pt_out);
  }

  msg_merged.point_num = static_cast<uint32_t>(n_front + n_back);
  // msg_merged.header.stamp = rclcpp::Time(msg_merged.timebase);
  // 点云格式转换及发布
  if (publish_pointcloud_) {
    pub_front_cloud_->publish(customMsgToPointCloud2(*msg_front, msg_front->header));
    pub_back_cloud_->publish(customMsgToPointCloud2(*msg_back, msg_back->header));
    pub_merged_cloud_->publish(customMsgToPointCloud2(msg_merged, msg_merged.header));
  }

  // 6. Zero-copy 发布：使用 std::move 交出内存所有权，消除最后 1MB 的栈到堆拷贝
  pub_merged_->publish(std::move(msg_merged));
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(LidarMergerNode)
