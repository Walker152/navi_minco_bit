/**
 * This file is part of SUPER
 *
 * Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/SUPER>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * SUPER is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SUPER is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with SUPER. If not, see <http://www.gnu.org/licenses/>.
 */

#include "traj_opt/yaw_traj_opt.h"
#include <algorithm>
#include <cmath>
#include <iostream>

#include "utils/geometry/geometry_utils.h"
#include <utils/optimization/polynomial_interpolation.h>
namespace traj_opt {
void YawTrajOpt::getYawTimeAllocation(double duration, Eigen::VectorXd & times) const
{
  const double pi = std::acos(-1.0);
  const double interp_dt = pi / yaw_dot_max_;
  if (duration < interp_dt * 2.0) {
    times.resize(1);
    times(0) = duration;
  } else {
    const int interp_num = static_cast<int>(std::ceil((duration - 2.0 * interp_dt) / interp_dt));
    const double interp_t = (duration - 2.0 * interp_dt) / static_cast<double>(interp_num);
    times.resize(2 + interp_num);
    for (int i = 0; i < interp_num; ++i) {
      times(i + 1) = interp_t;
    }
    times(times.size() - 1) = interp_dt;
    times(0) = interp_dt;
  }
  if (times.size() == 3 && times(1) < times(0) / 3.0) {
    times.resize(2);
    times.setConstant(duration / 2.0);
  }
}

void YawTrajOpt::getYawWaypointAllocation(const Eigen::Vector4d & init_state,
  Eigen::Vector4d & goal_state,
  Eigen::VectorXd & way_pts,
  const Eigen::VectorXd & times,
  const Trajectory & pos_traj)
{
  double eval_t = 0.0;
  double last_yaw = init_state(0);
  way_pts.resize(times.size() - 1);

  const double pos_traj_duration = pos_traj.getTotalDuration();
  for (int i = 0; i < times.size() - 1; ++i) {
    eval_t += times(i);
    Eigen::Vector3d pt_i = pos_traj.getPos(eval_t);
    Eigen::Vector3d pt_g;
    if (eval_t + 0.5 >= pos_traj_duration) {
      pt_g = pos_traj.getPos(pos_traj_duration);
      // Keep pt_i at eval_t — progressively shorter lookahead near the end.
    } else {
      pt_g = pos_traj.getPos(eval_t + 0.5);
    }

    const Eigen::Vector3d dir = pt_g - pt_i;
    double cur_yaw = last_yaw;
    if (dir.norm() > 0.1) {
      cur_yaw = std::atan2(dir.y(), dir.x());
      geometry_utils::normalizeNextYaw(last_yaw, cur_yaw);
    }

    way_pts(i) = cur_yaw;
    last_yaw = cur_yaw;
  }

  if (way_pts.size() == 0) {
    geometry_utils::normalizeNextYaw(init_state(0), goal_state(0));
  } else {
    geometry_utils::normalizeNextYaw(way_pts(way_pts.size() - 1), goal_state(0));
  }
}

YawTrajOpt::YawTrajOpt(double yaw_dot_max) : yaw_dot_max_(yaw_dot_max)
{
}

bool YawTrajOpt::optimize(const Eigen::Vector4d & istate_in,
  const Eigen::Vector4d & gstate_in,
  const Trajectory & pos_traj,
  Trajectory & out_traj,
  int order,
  bool free_start,
  bool free_goal)
{
  free_goal_ = free_goal;
  Eigen::Vector4d init_state = istate_in;
  Eigen::Vector4d goal_state = gstate_in;
  double pos_traj_dur = pos_traj.getTotalDuration();
  if (free_start) {
    Eigen::Vector3d pt_i = pos_traj.getPos(0);

    double t_g = std::min(0.5, pos_traj_dur);
    Eigen::Vector3d pt_g = pos_traj.getPos(t_g);
    Eigen::Vector3d dir = pt_g - pt_i;
    while (dir.norm() < 0.5 && t_g < pos_traj_dur) {
      t_g += 0.1;
      pt_g = pos_traj.getPos(t_g);
      dir = pt_g - pt_i;
    }
    init_state(0) = std::atan2(dir.y(), dir.x());
  }
  if (free_goal_) {
    Eigen::Vector3d pt_g = pos_traj.getPos(pos_traj_dur);
    double t_i = pos_traj_dur - 0.5 > 0 ? pos_traj_dur - 0.5 : 0;
    Eigen::Vector3d pt_i = pos_traj.getPos(t_i);
    Eigen::Vector3d dir = pt_g - pt_i;
    while (dir.norm() < 0.5 && t_i > 0) {
      t_i -= 0.1;
      pt_i = pos_traj.getPos(t_i);
      dir = pt_g - pt_i;
    }
    goal_state(0) = std::atan2(dir.y(), dir.x());
  }

  Eigen::VectorXd times;
  getYawTimeAllocation(pos_traj_dur, times);
  Eigen::VectorXd way_pts;
  getYawWaypointAllocation(init_state, goal_state, way_pts, times, pos_traj);
  Trajectory yaw_traj;
  switch (order) {
  case 3: {
    Eigen::Matrix<double, 1, 2> init_state2;
    Eigen::Matrix<double, 1, 2> goal_state2;
    init_state2 << init_state(0), init_state(1);
    goal_state2 << goal_state(0), goal_state(1);
    yaw_traj = geometry_utils::poly_interpo::minimumAccInterpolation<1>(
      init_state2, goal_state2, way_pts.transpose(), times);
    break;
  }
  case 5: {
    Eigen::Matrix<double, 1, 3> init_state3;
    Eigen::Matrix<double, 1, 3> goal_state3;
    init_state3 << init_state(0), init_state(1), init_state(2);
    goal_state3 << goal_state(0), goal_state(1), goal_state(2);
    yaw_traj = geometry_utils::poly_interpo::minimumJerkInterpolation<1>(
      init_state3, goal_state3, way_pts.transpose(), times);
    break;
  }
  case 7: {
    Eigen::Matrix<double, 1, 4> init_state4;
    Eigen::Matrix<double, 1, 4> goal_state4;
    init_state4 << init_state(0), init_state(1), init_state(2), init_state(3);
    goal_state4 << goal_state(0), goal_state(1), goal_state(2), goal_state(3);
    yaw_traj = geometry_utils::poly_interpo::minimumSnapInterpolation<1>(
      init_state4, goal_state4, way_pts.transpose(), times);
    break;
  }
  default: {
    std::cout << "Unsupported order for yaw trajectory optimization." << std::endl;
    return false;
  }
  }
  double max_yaw_rate = yaw_traj.getMaxVelRate();
  if (max_yaw_rate > yaw_dot_max_ + 2.0) {
    std::cout << "Yaw rate too large: " << max_yaw_rate << std::endl;
    return false;
  }
  out_traj = yaw_traj;
  out_traj.start_WT = pos_traj.start_WT;
  return true;
}
}  // namespace traj_opt