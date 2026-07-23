/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <rog_map/rog_map_visualizer.hpp>
#include <super_utils/color_msg_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap
{
  rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_nh_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<ROGMapVisualizer> visualizer_driver_;

  const double getSystemWalltimeNow() override { return now().seconds(); }

  void getSystemWalltimeNow(rclcpp::Time & _in) { _in = now(); };

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  bool getPriorMapTransform(PriorMapTransform2D & transform) override
  {
    if (cfg_.prior_map_frame == cfg_.frame_id) {
      transform = PriorMapTransform2D{};
      return true;
    }
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF is unavailable because the shared TF buffer is null");
      return false;
    }
    try {
      // lookupTransform(target, source, ...) returns T_target_source.
      const auto tf_msg = tf_->lookupTransform(cfg_.prior_map_frame, cfg_.frame_id, tf2::TimePointZero);
      transform.tx = tf_msg.transform.translation.x;
      transform.ty = tf_msg.transform.translation.y;
      const auto & q = tf_msg.transform.rotation;
      transform.yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      if (std::isfinite(transform.tx) && std::isfinite(transform.ty) &&
          std::isfinite(transform.yaw)) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] prior map TF from '%s' to '%s' contains a non-finite 2D transform",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str());
      return false;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
        *node_clock_->get_clock(),
        2000,
        "[ROGMap] cannot transform projection from '%s' to prior map frame '%s': %s",
        cfg_.frame_id.c_str(),
        cfg_.prior_map_frame.c_str(),
        error.what());
      return false;
    }
  }

  template <typename NodeT> void bindNode(NodeT node)
  {
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_timers_ = node->get_node_timers_interface();
    node_clock_ = node->get_node_clock_interface();
    node_logging_ = node->get_node_logging_interface();
    node_parameters_ = node->get_node_parameters_interface();
  }

  rclcpp::CallbackGroup::SharedPtr createCallbackGroup(rclcpp::CallbackGroupType type)
  {
    return node_base_->create_callback_group(type);
  }

  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr createPublisher(
    const std::string & topic, const rclcpp::QoS & qos)
  {
    return rclcpp::create_publisher<MsgT>(node_parameters_, node_topics_, topic, qos);
  }

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr createSubscription(const std::string & topic,
    const rclcpp::QoS & qos,
    CallbackT && callback,
    const rclcpp::SubscriptionOptions & options = rclcpp::SubscriptionOptions())
  {
    return rclcpp::create_subscription<MsgT>(
      node_parameters_, node_topics_, topic, qos, std::forward<CallbackT>(callback), options);
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr createWallTimer(std::chrono::duration<DurationRepT, DurationT> period,
    CallbackT && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return rclcpp::create_wall_timer(
      period, std::forward<CallbackT>(callback), group, node_base_.get(), node_timers_.get());
  }

  ROGMapVisualizer::Publishers vm_;

  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    PointCloud pc;
    double pc_odom_age_ms{0.0};
    rclcpp::TimerBase::SharedPtr update_timer;
    mutex updete_lock;
  } rc_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    if (performance_monitor_) {
      performance_monitor_->recordOdom(now().seconds());
    }
    updateRobotState(std::make_pair(
      Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z),
      Quatf(odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z)));
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg)
  {
    const double cbk_t = now().seconds();
    const double msg_stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
    const double queue_delay_ms = msg_stamp > 0.0 ? std::max(0.0, cbk_t - msg_stamp) * 1000.0 : 0.0;
    const double msg_points =
      static_cast<double>(cloud_msg->width) * static_cast<double>(cloud_msg->height);
    if (performance_monitor_) {
      performance_monitor_->recordCloudCallback(cbk_t, msg_points, queue_delay_ms, 0.0);
    }
    if (msg_points <= 0.0) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    if (!robot_state_.rcv) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropNoOdom();
      }
      std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET << std::endl;
      return;
    }
    if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropOdomTimeout();
      }
      std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
      return;
    }
    PointCloud temp_pc;
    const auto convert_start = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*cloud_msg, temp_pc);
    const double convert_time_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - convert_start).count();
    if (performance_monitor_) {
      performance_monitor_->recordCloudConvertTime(convert_time_ms);
    }
    if (temp_pc.empty()) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    rc_.updete_lock.lock();
    rc_.pc = temp_pc;
    rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
    rc_.pc_odom_age_ms = std::max(0.0, cbk_t - robot_state_.rcv_time) * 1000.0;
    rc_.unfinished_frame_cnt++;
    map_empty_ = false;
    rc_.updete_lock.unlock();
  }

  void updateCallback()
  {
    if (map_empty_) {
      static double last_print_t = now().seconds();
      double cur_t = now().seconds();
      if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
        std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET
                  << std::endl;
        last_print_t = cur_t;
      }
      return;
    }
    if (rc_.unfinished_frame_cnt == 0) {
      return;
    }

    if (rc_.unfinished_frame_cnt > 1) {
      std::cout << YELLOW << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time"
                << RESET << std::endl;
    }
    static PointCloud temp_pc;
    static Pose temp_pose;
    static double temp_odom_age_ms{0.0};

    rc_.updete_lock.lock();
    temp_pc = rc_.pc;
    temp_pose = rc_.pc_pose;
    temp_odom_age_ms = rc_.pc_odom_age_ms;
    rc_.unfinished_frame_cnt = 0;
    rc_.updete_lock.unlock();

    if (performance_monitor_) {
      performance_monitor_->recordValidCloud(temp_odom_age_ms);
    }
    updateMapInternal(temp_pc, temp_pose);
  }

  void vizCallback()
  {
    if (!cfg_.visualization_en) {
      return;
    }
    if (map_empty_) {
      return;
    }
    if (!hasVisualizationSubscriber()) {
      return;
    }

    Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
      return;
    }

    if (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> unknown_map;
      boxSearch(box_min, box_max, UNKNOWN, unknown_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(unknown_map, cloud_msg);
      vm_.unknown_pub->publish(cloud_msg);
    }
    if (cfg_.unk_inflation_en && vm_.unknown_inf_pub &&
        vm_.unknown_inf_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> inf_unknown_map;
      boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(inf_unknown_map, cloud_msg);
      vm_.unknown_inf_pub->publish(cloud_msg);
    }

    if (layer_ && !layer_->empty()) {
      if (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(fused_projection_mask_, grid);
        vm_.layer_value_pub->publish(grid);
      }
      if (vm_.layer_value_dynamic_pub &&
          vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(layer_->mask(), grid);
        vm_.layer_value_dynamic_pub->publish(grid);
      }
      if (vm_.layer_value_static_pub &&
          vm_.layer_value_static_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(prior_projection_mask_, grid);
        vm_.layer_value_static_pub->publish(grid);
      }
      if (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> types(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          switch (layer_->cells()[i].type) {
          case CellType::UNKNOWN:
            types[i] = 0U;
            break;
          case CellType::FREE:
            types[i] = 33U;
            break;
          case CellType::PASSABLE:
            types[i] = 66U;
            break;
          case CellType::OCCUPIED:
            types[i] = 100U;
            break;
          }
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(types, grid);
        vm_.layer_type_pub->publish(grid);
      }
      if (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> confidence(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          confidence[i] = static_cast<uint8_t>(
            std::clamp(static_cast<int>(std::round(layer_->cells()[i].confidence * 100.0f)), 0, 100));
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(confidence, grid);
        vm_.layer_confidence_pub->publish(grid);
      }
      if (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) {
        sensor_msgs::msg::PointCloud2 height_delta_cloud;
        fillLayerHeightDeltaCloud(height_delta_cloud);
        vm_.layer_height_delta_pub->publish(height_delta_cloud);
      }
    }

    if (field_ && field_->isValid() && vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 field_cloud;
      fillFieldCloud(field_cloud);
      vm_.field_pub->publish(field_cloud);
    }
    if (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 decay_cloud;
      fillDecayCellsCloud(decay_cloud);
      vm_.decay_cells_pub->publish(decay_cloud);
    }

    if (cfg_.frontier_extraction_en && vm_.frontier_pub &&
        vm_.frontier_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> frontier_map;
      boxSearch(box_min, box_max, FRONTIER, frontier_map);
      sensor_msgs::msg::PointCloud2 cloud_msg;
      vecEVec3fToPC2(frontier_map, cloud_msg);
      cloud_msg.header.stamp = now();
      vm_.frontier_pub->publish(cloud_msg);
    }

    vec_E<Vec3f> occ_map, inf_occ_map;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    bool original_occ_available = false;

    if (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) {
      boxSearch(box_min, box_max, OCCUPIED, occ_map);
      vecEVec3fToPC2(occ_map, cloud_msg);
      vm_.occ_pub->publish(cloud_msg);
      original_occ_available = true;
    }

    if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> raw_occ_map;
      rawOccupiedBoxSearch(box_min, box_max, raw_occ_map);
      sensor_msgs::msg::PointCloud2 raw_cloud_msg;
      vecEVec3fToPC2(raw_occ_map, raw_cloud_msg);
      vm_.raw_occ_pub->publish(raw_cloud_msg);

      static auto last_raw_occ_log = std::chrono::steady_clock::time_point{};
      const auto log_now = std::chrono::steady_clock::now();
      if (last_raw_occ_log.time_since_epoch().count() == 0 ||
          std::chrono::duration<double>(log_now - last_raw_occ_log).count() >= 1.0) {
        if (!original_occ_available) {
          boxSearch(box_min, box_max, OCCUPIED, occ_map);
        }
        std::cout << "[ROGMapViz] raw_occupied_points=" << raw_occ_map.size()
                  << ", original_occupied_points=" << occ_map.size() << std::endl;
        last_raw_occ_log = log_now;
      }
    }

    if (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) {
      boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
      vecEVec3fToPC2(inf_occ_map, cloud_msg);
      vm_.occ_inf_pub->publish(cloud_msg);
    }

    /* visualize ESDF Map*/
    if (cfg_.esdf_en) {
      if (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
        pcl::toROSMsg(pc, cloud_msg);
        cloud_msg.header.frame_id = cfg_.visualization_frame_id;
        cloud_msg.header.stamp = now();
        vm_.esdf_pub->publish(cloud_msg);
      }

      // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
      //     PointCloud pc;
      //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
      //     pcl::toROSMsg(pc, cloud_msg);
      //     cloud_msg.header.frame_id = "world";
      //     cloud_msg.header.stamp = now();
      //     vm_.esdf_neg_pub->publish(cloud_msg);
      // }

#ifdef ESDF_MAP_DEBUG
      esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
      cloud_msg.header.stamp = now();
      vm_.esdf_occ_pub->publish(cloud_msg);
#endif
    }

    if (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1) {
      visualization_msgs::msg::MarkerArray mkr_arr;
      visualizeBoundingBox(
        mkr_arr, now().seconds(), box_min, box_max, "Visualization Range", Color::Purple());
      visualizeText(mkr_arr,
        now().seconds(),
        "Visualization Range Text",
        "Visualization Range",
        box_max + Vec3f(0, 0, 0.5),
        Color::Purple(),
        0.6,
        0);

      Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
      boundBoxByLocalMap(local_map_min, local_map_max);
      visualizeBoundingBox(
        mkr_arr, now().seconds(), local_map_min, local_map_max, "Local Map Range", Color::Orange());
      visualizeText(mkr_arr,
        now().seconds(),
        "Local Map Range Text",
        "Local Map Range",
        local_map_max + Vec3f(0, 0, 1.0),
        Color::Orange(),
        0.6,
        0);

      visualizeBoundingBox(mkr_arr,
        now().seconds(),
        raycast_data_.cache_box_min,
        raycast_data_.cache_box_max,
        "Updating Range",
        Color::Green());
      visualizeText(mkr_arr,
        now().seconds(),
        "Updating Range Text",
        "Updating Range",
        raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
        Color::Green(),
        0.6,
        0);

      visualizePoint(
        mkr_arr, now().seconds(), local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

      if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(mkr_arr,
          now().seconds(),
          "ESDF Map Text",
          "ESDF Map",
          esdf_box_max + Vec3f(0, 0, 1.0),
          Color::Blue(),
          0.6,
          0);
        visualizeBoundingBox(
          mkr_arr, now().seconds(), esdf_box_min, esdf_box_max, "ESDF Updating Range", Color::Blue());
      }

      for (auto & marker : mkr_arr.markers) {
        marker.header.frame_id = cfg_.visualization_frame_id;
      }
      vm_.mkr_arr_pub->publish(mkr_arr);
    }
  }

  bool hasVisualizationSubscriber()
  {
    return (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) ||
           (vm_.unknown_inf_pub && vm_.unknown_inf_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_pub && vm_.layer_value_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_dynamic_pub &&
            vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_static_pub &&
            vm_.layer_value_static_pub->get_subscription_count() >= 1) ||
           (vm_.layer_type_pub && vm_.layer_type_pub->get_subscription_count() >= 1) ||
           (vm_.layer_confidence_pub && vm_.layer_confidence_pub->get_subscription_count() >= 1) ||
           (vm_.layer_height_delta_pub && vm_.layer_height_delta_pub->get_subscription_count() >= 1) ||
           (vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) ||
           (vm_.decay_cells_pub && vm_.decay_cells_pub->get_subscription_count() >= 1) ||
           (vm_.frontier_pub && vm_.frontier_pub->get_subscription_count() >= 1) ||
           (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
           (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) ||
           (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) ||
           (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) ||
           (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1);
  }

  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
  {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillLayerGrid(const std::vector<uint8_t> & data, nav_msgs::msg::OccupancyGrid & grid)
  {
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(layer_->resolution());
    grid.info.width = static_cast<uint32_t>(layer_->width());
    grid.info.height = static_cast<uint32_t>(layer_->height());
    grid.info.origin.position.x = layer_->origin().x();
    grid.info.origin.position.y = layer_->origin().y();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      grid.data[i] = static_cast<int8_t>(std::min<int>(100, data[i]));
    }
  }

  void fillLayerMaskGrid(const std::vector<uint8_t> & mask, nav_msgs::msg::OccupancyGrid & grid)
  {
    std::vector<uint8_t> occupancy(mask.size(), 0U);
    for (size_t i = 0; i < mask.size(); ++i) {
      occupancy[i] = mask[i] == 0U ? 100U : 0U;
    }
    fillLayerGrid(occupancy, grid);
  }

  void fillLayerHeightDeltaCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto & cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
          static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) + static_cast<size_t>(x);
        if (idx >= cells.size()) {
          continue;
        }
        const auto & cell = cells[idx];
        if (cell.type == CellType::UNKNOWN) {
          continue;
        }
        pcl::PointXYZI p;
        p.x =
          static_cast<float>(layer_->origin().x() + (static_cast<double>(x) + 0.5) * layer_->resolution());
        p.y =
          static_cast<float>(layer_->origin().y() + (static_cast<double>(y) + 0.5) * layer_->resolution());
        p.z = cell.occupied_z_max_abs;
        p.intensity = cell.height_delta;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillFieldCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto distances = field_->distances();
    const int width = field_->width();
    const int height = field_->height();
    const double resolution = field_->resolution();
    const Eigen::Vector2d origin = field_->origin();
    pcl_cloud.reserve(distances.size());
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
        if (idx >= distances.size()) {
          continue;
        }
        const double d = distances[idx];
        if (!std::isfinite(d)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(origin.x() + (static_cast<double>(x) + 0.5) * resolution);
        p.y = static_cast<float>(origin.y() + (static_cast<double>(y) + 0.5) * resolution);
        p.z = 0.0f;
        p.intensity = static_cast<float>(d);
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillDecayCellsCloud(sensor_msgs::msg::PointCloud2 & cloud)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(active_ids_.size());
    const double now_s = now().seconds();
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size()) || !active_flags_[hash_id] ||
          !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      pcl::PointXYZI p;
      p.x = static_cast<float>(pos.x());
      p.y = static_cast<float>(pos.y());
      p.z = static_cast<float>(pos.z());
      p.intensity = static_cast<float>(std::max(0.0, now_s - static_cast<double>(last_hit_time_[hash_id])));
      pcl_cloud.push_back(p);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void initializeRos()
  {
    // TODO: The current implementation uses a lenient QoS configuration for message transmission.
    const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    if (cfg_.prior_map_enable) {
      prior_map_ = loadPriorMap(cfg_.prior_map_yaml_path, cfg_.prior_map_pgm_path);
      RCLCPP_INFO(node_logging_->get_logger(),
        "[ROGMap] loaded prior map %dx%d at %.3f m/cell in frame '%s'",
        prior_map_.width,
        prior_map_.height,
        prior_map_.resolution,
        cfg_.prior_map_frame.c_str());
    }
    init();
    /// Initialize visualization module
    if (cfg_.visualization_en) {
      visualizer_driver_ = std::make_unique<ROGMapVisualizer>();
      visualizer_driver_->configure(node_base_,
        node_parameters_,
        node_topics_,
        node_timers_,
        cfg_,
        std::bind(&ROGMapROS::vizCallback, this));
      const auto & pubs = visualizer_driver_->publishers();
      vm_.occ_pub = pubs.occ_pub;
      vm_.raw_occ_pub = pubs.raw_occ_pub;
      vm_.unknown_pub = pubs.unknown_pub;
      vm_.esdf_neg_pub = pubs.esdf_neg_pub;
      vm_.esdf_occ_pub = pubs.esdf_occ_pub;
      vm_.occ_inf_pub = pubs.occ_inf_pub;
      vm_.unknown_inf_pub = pubs.unknown_inf_pub;
      vm_.frontier_pub = pubs.frontier_pub;
      vm_.esdf_pub = pubs.esdf_pub;
      vm_.layer_height_delta_pub = pubs.layer_height_delta_pub;
      vm_.field_pub = pubs.field_pub;
      vm_.decay_cells_pub = pubs.decay_cells_pub;
      vm_.layer_value_pub = pubs.layer_value_pub;
      vm_.layer_value_dynamic_pub = pubs.layer_value_dynamic_pub;
      vm_.layer_value_static_pub = pubs.layer_value_static_pub;
      vm_.layer_type_pub = pubs.layer_type_pub;
      vm_.layer_confidence_pub = pubs.layer_confidence_pub;
      vm_.mkr_arr_pub = pubs.mkr_arr_pub;
    }

    if (cfg_.ros_callback_en) {
      rc_.odom_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = createSubscription<nav_msgs::msg::Odometry>(
        cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      rc_.cloud_sub = createSubscription<sensor_msgs::msg::PointCloud2>(
        cfg_.cloud_topic,
        rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { this->cloudCallback(std::move(msg)); },
        so);
      rc_.update_cbk_group = createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = createWallTimer(std::chrono::milliseconds(cfg_.update_period_ms),
        std::bind(&ROGMapROS::updateCallback, this),
        rc_.update_cbk_group);
    }
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : lifecycle_nh_(nh), tf_(tf)
  {
    bindNode(lifecycle_nh_);
    cfg_ = cfg;
    initializeRos();
  }

  ROGMapROS(const rclcpp::Node::SharedPtr nh,
    const rog_map::Config & cfg,
    const std::shared_ptr<tf2_ros::Buffer> & tf = nullptr)
  : nh_(nh), tf_(tf)
  {
    bindNode(nh_);
    cfg_ = cfg;
    initializeRos();
  }

  std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const
  {
    return getQueryInterface();
  }

private:
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray & mkrarr,
    const double & stamp,
    const Vec3f & box_min,
    const Vec3f & box_max,
    const string & ns,
    const Color & color,
    const double & size_x = 0.1,
    const double & alpha = 1.0,
    const bool & print_ns = true)
  {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = rclcpp::Time(stamp);
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++;  // unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;  // marker type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha;  //不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  static void visualizeText(visualization_msgs::msg::MarkerArray & mkr_arr,
    const double & stamp,
    const std::string & ns,
    const std::string & text,
    const Vec3f & position,
    const Color & c = Color::White(),
    const double & size = 0.6,
    const int & id = -1)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = rclcpp::Time(stamp);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  static void visualizePoint(visualization_msgs::msg::MarkerArray & mkr_arr,
    const double & stamp,
    const Vec3f & pt,
    Color color = Color::Pink(),
    std::string ns = "pt",
    double size = 0.1,
    int id = -1,
    const bool & print_ns = true)
  {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = rclcpp::Time(stamp);
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Time(stamp);
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
