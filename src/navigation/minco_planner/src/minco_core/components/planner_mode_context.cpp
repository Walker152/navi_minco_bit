#include "minco_core/components/planner_mode_context.hpp"

#include <cctype>

namespace minco_planner {

void PlannerModeContext::configure(const PlannerModeParams & params,
  const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger)
{
  params_ = params;
  map_frame_ = params_.map_frame.empty() ? "map" : params_.map_frame;
  rog_frame_ = params_.rog_frame.empty() ? "camera_init" : params_.rog_frame;

  std::string mode_upper = params_.planner_mode;
  std::transform(mode_upper.begin(), mode_upper.end(), mode_upper.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });

  mode_ = PlannerMode::PRIORMAP;
  if (mode_upper == "EXPLORATION") {
    mode_ = PlannerMode::EXPLORATION;
  } else if (mode_upper != "PRIORMAP") {
    RCLCPP_WARN(logger,
      "[MincoPlanner] Unknown planner_mode='%s', fallback to PRIORMAP.",
      params_.planner_mode.c_str());
  }

  if (mode_ == PlannerMode::PRIORMAP) {
    planning_frame_ = map_frame_;
    output_frame_ = map_frame_;
    direct_odom_pose_ = false;
    if (!params_.priormap_use_nav2_global_search) {
      RCLCPP_WARN(logger,
        "[MincoPlanner] priormap.use_nav2_global_search=false is unsupported by planner_mode=PRIORMAP; "
        "forcing Nav2 costmap global search to preserve map semantics.");
    }
  } else {
    planning_frame_ = rog_frame_;
    output_frame_ = rog_frame_;
    direct_odom_pose_ = true;
  }

  rebuildQueries(raw_rog_query, costmap_ros, tf, logger);
}

void PlannerModeContext::rebuildQueries(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
  nav2_costmap_2d::Costmap2DROS * costmap_ros,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp::Logger & logger)
{
  if (mode_ == PlannerMode::PRIORMAP) {
    global_query_ = nullptr;
    if (costmap_ros && costmap_ros->getCostmap()) {
      global_query_ = std::make_shared<Nav2CostmapQuery>(costmap_ros->getCostmap());
    }
    dynamic_query_ = raw_rog_query ? std::make_shared<FrameAwareRogQuery>(
                                       raw_rog_query, tf, map_frame_, rog_frame_, logger)
                                   : nullptr;
    sparsify_query_ = global_query_;
  } else {
    global_query_ = raw_rog_query;
    dynamic_query_ = raw_rog_query;
    sparsify_query_ = raw_rog_query;
  }
}

}  // namespace minco_planner
