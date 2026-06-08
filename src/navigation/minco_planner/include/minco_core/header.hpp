#ifndef MINCO_PLANNER__UTILS__HEADER_HPP_
#define MINCO_PLANNER__UTILS__HEADER_HPP_

// C++ standard library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

// Third-party
#include <Eigen/Core>

// ROS 2
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "visualization_msgs/msg/marker.hpp"

// Project messages
#include "ros_interfaces/msg/mpc_position_command.hpp"
#include "ros_interfaces/msg/position_command.hpp"
#include "std_msgs/msg/header.hpp"

// Project dependencies
#include "data_structure/base/trajectory.h"
#include "rog_map/map_query_interface.hpp"
#include "small_rog_map/hybrid_esdf_map.hpp"
#include "smac_search/smac_planner_2d_simple.hpp"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "traj_opt/minco_optimizer.hpp"
#include "traj_opt/yaw_traj_opt.h"
#include "utils/header/color_text.hpp"

// Minco core headers
#include "minco_core/astar.hpp"
#include "minco_core/corridor_generator.hpp"
#include "minco_core/components/map_query_adapters.hpp"
#include "minco_core/components/planner_mode_context.hpp"
#include "minco_core/components/global_path_searcher.hpp"
#include "minco_core/components/local_path_processor.hpp"
#include "minco_core/components/recovery_behaivor.hpp"
#include "minco_core/components/trajectory_safety_checker.hpp"
#include "minco_core/minco_fsm.hpp"
#include "minco_core/minco_utils.hpp"
#include "minco_core/static_esdf_map.hpp"
#include "minco_core/visualizer.hpp"

#endif  // MINCO_PLANNER__UTILS__HEADER_HPP_
