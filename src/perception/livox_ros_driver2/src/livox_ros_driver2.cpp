//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "driver_node.h"
#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char ** argv)
{
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(
    xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar * read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ =
    std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ =
    std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) {
    usleep(10000);
  }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros {
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options) : Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;
  bool enable_internal_lidar_merge = false;
  bool enable_merge_debug = false;
  std::string merge_front_ip = "192.168.1.135";
  std::string merge_back_ip = "192.168.1.122";
  std::string merge_output_topic = "livox/lidar";
  std::string merge_frame_id = "livox_frame";
  double merge_max_interval_ms = 5.0;
  std::vector<double> merge_extrinsic_back_to_front{0.0, 0.4, 0.0, -0.35453, 0.0, 0.0};

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");
  this->declare_parameter("enable_internal_lidar_merge", enable_internal_lidar_merge);
  this->declare_parameter("enable_merge_debug", enable_merge_debug);
  this->declare_parameter("merge_front_ip", merge_front_ip);
  this->declare_parameter("merge_back_ip", merge_back_ip);
  this->declare_parameter("merge_output_topic", merge_output_topic);
  this->declare_parameter("merge_frame_id", merge_frame_id);
  this->declare_parameter("merge_max_interval_ms", merge_max_interval_ms);
  this->declare_parameter("merge_extrinsic_back_to_front", merge_extrinsic_back_to_front);

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("enable_internal_lidar_merge", enable_internal_lidar_merge);
  this->get_parameter("enable_merge_debug", enable_merge_debug);
  this->get_parameter("merge_front_ip", merge_front_ip);
  this->get_parameter("merge_back_ip", merge_back_ip);
  this->get_parameter("merge_output_topic", merge_output_topic);
  this->get_parameter("merge_frame_id", merge_frame_id);
  this->get_parameter("merge_max_interval_ms", merge_max_interval_ms);
  this->get_parameter("merge_extrinsic_back_to_front", merge_extrinsic_back_to_front);

  const bool effective_internal_merge =
    enable_internal_lidar_merge && (multi_topic == 1);
  if (enable_internal_lidar_merge && multi_topic == 0) {
    RCLCPP_WARN(
      get_logger(),
      "[LivoxDriver] enable_internal_lidar_merge=true is ignored because multi_topic=0. "
      "Use direct single-lidar path.");
  }

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ =
    std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (effective_internal_merge) {
    if (!IsInternalLidarMergeTransferFormatSupported(xfer_format)) {
      RCLCPP_ERROR(get_logger(), "Unsupported transfer format for internal lidar merge");
      throw std::invalid_argument("unsupported transfer format for internal lidar merge");
    }
    if (merge_extrinsic_back_to_front.size() != 6) {
      RCLCPP_ERROR(get_logger(), "Invalid merge extrinsic size: %zu", merge_extrinsic_back_to_front.size());
      throw std::invalid_argument("invalid merge extrinsic size");
    }
    if (merge_max_interval_ms <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid merge max interval: %.3f ms", merge_max_interval_ms);
      throw std::invalid_argument("invalid merge max interval");
    }
    if (merge_front_ip.empty() || merge_back_ip.empty() || merge_front_ip == merge_back_ip) {
      RCLCPP_ERROR(
        get_logger(), "Invalid merge lidar IP configuration: front='%s' back='%s'",
        merge_front_ip.c_str(), merge_back_ip.c_str());
      throw std::invalid_argument("invalid merge lidar ip configuration");
    }
    if (merge_output_topic.empty()) {
      RCLCPP_ERROR(get_logger(), "Internal lidar merge output topic must not be empty");
      throw std::invalid_argument("invalid merge output topic");
    }
    if (merge_frame_id.empty()) {
      merge_frame_id = frame_id;
    }

    InternalLidarMergeConfig merge_config;
    merge_config.enabled = true;
    merge_config.enable_merge_debug = enable_merge_debug;
    merge_config.front_handle = IpStringToNum(merge_front_ip);
    merge_config.back_handle = IpStringToNum(merge_back_ip);
    merge_config.output_topic = merge_output_topic;
    merge_config.frame_id = merge_frame_id;
    merge_config.max_interval_ns = static_cast<uint64_t>(merge_max_interval_ms * 1000000.0);
    for (size_t i = 0; i < merge_config.extrinsic_back_to_front.size(); ++i) {
      merge_config.extrinsic_back_to_front[i] = merge_extrinsic_back_to_front[i];
    }
    lddc_ptr_->ConfigureInternalLidarMerge(merge_config);
  }

  if (data_src != kSourceRawLidar) {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
    throw std::invalid_argument("invalid lidar data source");
  }

  DRIVER_INFO(*this, "Data Source is raw lidar.");

  std::string user_config_path;
  this->get_parameter("user_config_path", user_config_path);
  DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

  std::string cmdline_bd_code;
  this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

  LdsLidar * read_lidar = LdsLidar::GetInstance(publish_freq);
  if (!read_lidar || lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar)) != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to register lidar data source");
    throw std::runtime_error("failed to register lidar data source");
  }
  if (!read_lidar->InitLdsLidar(user_config_path)) {
    RCLCPP_ERROR(get_logger(), "Init lds lidar failed");
    throw std::runtime_error("failed to initialize lidar data source");
  }
  DRIVER_INFO(*this, "Init lds lidar success!");

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2

void DriverNode::PointCloudDataPollThread()
{
#ifdef BUILDING_ROS1
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
#elif defined BUILDING_ROS2
  if (future_.wait_for(std::chrono::seconds(3)) != std::future_status::timeout) {
    return;
  }
  while (rclcpp::ok() && !stop_requested_.load()) {
    Lddc * lddc = lddc_ptr_.get();
    if (!lddc) {
      return;
    }
    try {
      lddc->DistributePointCloudData();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Point cloud poll thread stopped: %s", error.what());
      return;
    }
    if (future_.wait_for(std::chrono::microseconds(0)) != std::future_status::timeout) {
      return;
    }
  }
#endif
}

void DriverNode::ImuDataPollThread()
{
#ifdef BUILDING_ROS1
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
#elif defined BUILDING_ROS2
  if (future_.wait_for(std::chrono::seconds(3)) != std::future_status::timeout) {
    return;
  }
  while (rclcpp::ok() && !stop_requested_.load()) {
    Lddc * lddc = lddc_ptr_.get();
    if (!lddc) {
      return;
    }
    try {
      lddc->DistributeImuData();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "IMU poll thread stopped: %s", error.what());
      return;
    }
    if (future_.wait_for(std::chrono::microseconds(0)) != std::future_status::timeout) {
      return;
    }
  }
#endif
}
