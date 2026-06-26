#ifndef MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_
#define MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

enum class PlannerMode
{
  PRIORMAP,
  EXPLORATION
};

struct PlannerModeParams
{
  std::string planner_mode{"PRIORMAP"};
  std::string map_frame{"map"};
  std::string rog_frame{"camera_init"};

  bool priormap_use_nav2_global_search{true};
  bool priormap_clip_seed_by_rog_boundary{true};
  double priormap_rog_boundary_margin{0.8};
  double priormap_rog_boundary_sample_step{0.1};

  double exploration_boundary_margin{0.8};
  double exploration_boundary_sample_step{0.1};
  bool exploration_unknown_as_occupied{true};
  bool exploration_prefer_goal_direction{true};
};

class PlannerModeContext
{
public:
  void configure(const PlannerModeParams & params,
    const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger);

  void rebuildQueries(const std::shared_ptr<rog_map::MapQueryInterface> & raw_rog_query,
    nav2_costmap_2d::Costmap2DROS * costmap_ros,
    const std::shared_ptr<tf2_ros::Buffer> & tf,
    const rclcpp::Logger & logger);

  PlannerMode mode() const { return mode_; }
  const std::string & planningFrame() const { return planning_frame_; }
  const std::string & outputFrame() const { return output_frame_; }
  const std::string & mapFrame() const { return map_frame_; }
  const std::string & rogFrame() const { return rog_frame_; }

  bool directOdomPose() const { return direct_odom_pose_; }

  bool clipSeedByRogBoundary() const { return params_.priormap_clip_seed_by_rog_boundary; }
  double rogBoundaryMargin() const { return params_.priormap_rog_boundary_margin; }
  double rogBoundarySampleStep() const { return params_.priormap_rog_boundary_sample_step; }

  double explorationBoundaryMargin() const { return params_.exploration_boundary_margin; }
  double explorationBoundarySampleStep() const { return params_.exploration_boundary_sample_step; }
  bool explorationUnknownAsOccupied() const { return params_.exploration_unknown_as_occupied; }
  bool explorationPreferGoalDirection() const { return params_.exploration_prefer_goal_direction; }

  std::shared_ptr<rog_map::MapQueryInterface> globalQuery() const { return global_query_; }
  std::shared_ptr<rog_map::MapQueryInterface> dynamicQuery() const { return dynamic_query_; }
  std::shared_ptr<rog_map::MapQueryInterface> sparsifyQuery() const { return sparsify_query_; }

private:
  PlannerModeParams params_{};
  PlannerMode mode_{PlannerMode::PRIORMAP};
  std::string planning_frame_{"map"};
  std::string output_frame_{"map"};
  std::string map_frame_{"map"};
  std::string rog_frame_{"camera_init"};
  bool direct_odom_pose_{false};

  std::shared_ptr<rog_map::MapQueryInterface> global_query_;
  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  std::shared_ptr<rog_map::MapQueryInterface> sparsify_query_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNER_MODE_CONTEXT_HPP_
