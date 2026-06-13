#ifndef MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
#define MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class Nav2CostmapQuery : public rog_map::MapQueryInterface
{
public:
  explicit Nav2CostmapQuery(nav2_costmap_2d::Costmap2D * costmap);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
};

class FrameAwareRogQuery : public rog_map::MapQueryInterface
{
public:
  FrameAwareRogQuery(std::shared_ptr<rog_map::MapQueryInterface> raw,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::string planning_frame,
    std::string rog_frame,
    rclcpp::Logger logger);

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override;
  unsigned int sizeX() const override;
  unsigned int sizeY() const override;
  double resolution() const override;
  double originX() const override;
  double originY() const override;
  uint8_t value(unsigned int mx, unsigned int my) const override;
  const unsigned char * values() const override;
  bool copyValues(std::vector<unsigned char> & out) const override;
  bool isValid(unsigned int mx, unsigned int my) const override;
  bool isFree(unsigned int mx, unsigned int my) const override;
  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override;
  bool evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const override;

private:
  bool transformPlanningToRog(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool transformRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;
  bool rotateRogToPlanning(const Eigen::Vector3d & in, Eigen::Vector3d & out) const;

  std::shared_ptr<rog_map::MapQueryInterface> raw_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string planning_frame_;
  std::string rog_frame_;
  rclcpp::Logger logger_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MAP_QUERY_ADAPTERS_HPP_
