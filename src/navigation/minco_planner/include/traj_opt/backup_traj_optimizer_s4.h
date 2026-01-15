// Lightweight backup trajectory optimizer interface for MincoPlanner.
//
// This header intentionally provides a minimal API surface needed by MincoPlanner
// (setInitState / setStopConstraints / setPolygons / optimize).

#pragma once

#include <Eigen/Core>

#include <memory>
#include <vector>

#include "data_structure/base/trajectory.h"

namespace traj_opt
{

using Trajectory = geometry_utils::Trajectory;
using PolyhedronH = Eigen::MatrixX4d;

class BackupTrajOpt
{
public:
  using Ptr = std::shared_ptr<BackupTrajOpt>;

  BackupTrajOpt() = default;

  void setInitState(const Eigen::Matrix3d & start_state);

  void setStopConstraints();

  void setPolygons(const std::vector<PolyhedronH> & polys);

  bool optimize(Trajectory & out_traj) const;

private:
  Eigen::Matrix3d start_state_{Eigen::Matrix3d::Zero()};
  bool has_start_{false};
  bool stop_constraints_{false};
  std::vector<PolyhedronH> polys_;
};

}  // namespace traj_opt
