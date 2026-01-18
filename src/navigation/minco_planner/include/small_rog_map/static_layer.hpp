#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace small_rog_map
{

class StaticLayer
{
public:
  using Ptr = std::shared_ptr<StaticLayer>;

  StaticLayer();

  bool loadFromPCD(const std::string & pcd_path, double resolution);

  // Query distance (meters) and gradient (meters^-1) using bilinear interpolation.
  void evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const;

  bool isValid() const;

  int width() const;
  int height() const;
  double resolution() const;
  const Eigen::Vector2d & origin() const;
  const std::vector<double> & distanceBuffer() const;

private:
  static constexpr double kFarDistance = 10.0;

  std::vector<double> dist_m_;  // distance in meters
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  Eigen::Vector2d origin_{0.0, 0.0};
};

}  // namespace small_rog_map
