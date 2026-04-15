#include "traj_opt/backup_traj_optimizer_s4.h"

#include <cmath>
#include <limits>
#include <vector>

namespace traj_opt {

namespace {

struct Aabb
{
  double xmin{-std::numeric_limits<double>::infinity()};
  double xmax{std::numeric_limits<double>::infinity()};
  double ymin{-std::numeric_limits<double>::infinity()};
  double ymax{std::numeric_limits<double>::infinity()};
  double zmin{-std::numeric_limits<double>::infinity()};
  double zmax{std::numeric_limits<double>::infinity()};
};

inline bool nearlyEqual(double a, double b, double eps = 1e-9)
{
  return std::abs(a - b) < eps;
}

inline bool extractAxisAlignedBounds(const PolyhedronH & h, Aabb & out)
{
  if (h.cols() != 4 || h.rows() < 6) {
    return false;
  }

  // Parse half-space rows: n_x x + n_y y + n_z z < d
  for (int i = 0; i < h.rows(); ++i) {
    const double nx = h(i, 0);
    const double ny = h(i, 1);
    const double nz = h(i, 2);
    const double d = h(i, 3);

    if (nearlyEqual(nx, 1.0) && nearlyEqual(ny, 0.0) && nearlyEqual(nz, 0.0)) {
      out.xmax = std::min(out.xmax, d);
    } else if (nearlyEqual(nx, -1.0) && nearlyEqual(ny, 0.0) && nearlyEqual(nz, 0.0)) {
      out.xmin = std::max(out.xmin, -d);
    } else if (nearlyEqual(nx, 0.0) && nearlyEqual(ny, 1.0) && nearlyEqual(nz, 0.0)) {
      out.ymax = std::min(out.ymax, d);
    } else if (nearlyEqual(nx, 0.0) && nearlyEqual(ny, -1.0) && nearlyEqual(nz, 0.0)) {
      out.ymin = std::max(out.ymin, -d);
    } else if (nearlyEqual(nx, 0.0) && nearlyEqual(ny, 0.0) && nearlyEqual(nz, 1.0)) {
      out.zmax = std::min(out.zmax, d);
    } else if (nearlyEqual(nx, 0.0) && nearlyEqual(ny, 0.0) && nearlyEqual(nz, -1.0)) {
      out.zmin = std::max(out.zmin, -d);
    }
  }

  return std::isfinite(out.xmin) && std::isfinite(out.xmax) && std::isfinite(out.ymin) &&
         std::isfinite(out.ymax) && std::isfinite(out.zmin) && std::isfinite(out.zmax) &&
         (out.xmin < out.xmax) && (out.ymin < out.ymax) && (out.zmin < out.zmax);
}

inline bool insideBounds(const Aabb & b, const Eigen::Vector3d & p)
{
  constexpr double eps = 1e-6;
  return (p.x() > b.xmin + eps && p.x() < b.xmax - eps && p.y() > b.ymin + eps && p.y() < b.ymax - eps &&
          p.z() > b.zmin + eps && p.z() < b.zmax - eps);
}

inline double raycastBox(const Eigen::Vector3d & start_pos,
  const Eigen::Vector3d & direction,
  const Eigen::Vector3d & box_min,
  const Eigen::Vector3d & box_max)
{
  constexpr double kEps = 1e-12;
  if (!start_pos.allFinite() || !direction.allFinite() || direction.squaredNorm() < kEps) {
    return 0.0;
  }

  const bool inside =
    (start_pos.x() >= box_min.x() && start_pos.x() <= box_max.x() && start_pos.y() >= box_min.y() &&
      start_pos.y() <= box_max.y() && start_pos.z() >= box_min.z() && start_pos.z() <= box_max.z());
  if (!inside) {
    return 0.0;
  }

  double tmin = -std::numeric_limits<double>::infinity();
  double tmax = std::numeric_limits<double>::infinity();

  for (int axis = 0; axis < 3; ++axis) {
    const double o = start_pos(axis);
    const double d = direction(axis);
    const double mn = box_min(axis);
    const double mx = box_max(axis);

    if (std::abs(d) < kEps) {
      // Parallel to this slab; inside already.
      continue;
    }

    double t1 = (mn - o) / d;
    double t2 = (mx - o) / d;
    if (t1 > t2) {
      std::swap(t1, t2);
    }

    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmax < tmin) {
      return 0.0;
    }
  }

  if (!(tmax > 0.0) || !std::isfinite(tmax)) {
    return 0.0;
  }
  return tmax;
}

// Build a 5th-order polynomial satisfying:
// t=0: P=P0, V=V0, A=A0
// t=T: P=P_end, V=0, A=0
// Coeff matrix follows Piece convention: [t^5 t^4 t^3 t^2 t 1]
inline bool buildForwardStopQuintic(const Eigen::Vector3d & p0,
  const Eigen::Vector3d & v0,
  const Eigen::Vector3d & a0,
  const Eigen::Vector3d & p_end,
  double T,
  Eigen::MatrixXd & out_coeffMat)
{
  if (!(T > 1e-4) || !std::isfinite(T)) {
    return false;
  }

  out_coeffMat = Eigen::MatrixXd::Zero(3, 6);

  const double t = T;
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double t4 = t3 * t;
  const double t5 = t4 * t;

  for (int axis = 0; axis < 3; ++axis) {
    const double p0a = p0(axis);
    const double v0a = v0(axis);
    const double a0a = a0(axis);
    const double pTa = p_end(axis);

    // Closed-form quintic interpolation:
    // p(t)=c0+c1 t+c2 t^2+c3 t^3+c4 t^4+c5 t^5
    const double c0 = p0a;
    const double c1 = v0a;
    const double c2 = 0.5 * a0a;

    const double dp = pTa - p0a;
    const double c3 = (10.0 * dp) / t3 - (6.0 * v0a) / t2 - (1.5 * a0a) / t;
    const double c4 = (-15.0 * dp) / t4 + (8.0 * v0a) / t3 + (1.5 * a0a) / t2;
    const double c5 = (6.0 * dp) / t5 - (3.0 * v0a) / t4 - (0.5 * a0a) / t3;

    // Piece expects columns: [t^5, t^4, t^3, t^2, t^1, t^0]
    out_coeffMat(axis, 5) = c0;
    out_coeffMat(axis, 4) = c1;
    out_coeffMat(axis, 3) = c2;
    out_coeffMat(axis, 2) = c3;
    out_coeffMat(axis, 1) = c4;
    out_coeffMat(axis, 0) = c5;
  }

  return true;
}

inline Eigen::Vector3d evalPosFromCoeffMat(const Eigen::MatrixXd & coeffMat, double t)
{
  // coeffMat: 3x6 with columns [t^5..t^0]
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  double tn = 1.0;
  for (int col = 5; col >= 0; --col) {
    p += tn * coeffMat.col(col);
    tn *= t;
  }
  return p;
}

}  // namespace

void BackupTrajOpt::setInitState(const Eigen::Matrix3d & start_state)
{
  start_state_ = start_state;
  has_start_ = true;
}

void BackupTrajOpt::setStopConstraints()
{
  stop_constraints_ = true;
}

void BackupTrajOpt::setPolygons(const std::vector<PolyhedronH> & polys)
{
  polys_ = polys;
}

bool BackupTrajOpt::optimize(Trajectory & out_traj) const
{
  if (!has_start_) {
    return false;
  }

  const Eigen::Vector3d p0 = start_state_.col(0);
  Eigen::Vector3d v0 = start_state_.col(1);
  Eigen::Vector3d a0 = start_state_.col(2);

  // Force 2D planar behavior
  v0.z() = 0.0;
  a0.z() = 0.0;

  // Extract an axis-aligned safe box from the first polyhedron
  Aabb bounds;
  const bool has_bounds = (!polys_.empty()) ? extractAxisAlignedBounds(polys_.front(), bounds) : false;

  const double v_mag = v0.norm();
  if (!std::isfinite(v_mag)) {
    return false;
  }

  // Compute physical braking distance
  constexpr double a_max = 3.0;
  const double L_phy = (v_mag > 1e-4) ? (v_mag * v_mag) / (2.0 * a_max) : 0.0;

  // Compute geometric free distance inside the safe box
  double L_geo = std::numeric_limits<double>::infinity();
  Eigen::Vector3d dir = Eigen::Vector3d::Zero();
  if (v_mag > 1e-6) {
    dir = v0 / v_mag;
  }
  if (has_bounds && v_mag > 1e-4) {
    const Eigen::Vector3d box_min(bounds.xmin, bounds.ymin, bounds.zmin);
    const Eigen::Vector3d box_max(bounds.xmax, bounds.ymax, bounds.zmax);
    const double hit = raycastBox(p0, dir, box_min, box_max);
    L_geo = (hit > 0.0) ? hit * 0.95 : 0.0;
  }

  double a_brake = a_max;

  if (L_phy > L_geo && L_geo > 0.05) {
    a_brake = (v_mag * v_mag) / (2.0 * L_geo);
    a_brake = std::min(a_brake, 8.0);
  }

  if (L_geo <= 0.05) {
    a_brake = 8.0;
  }

  // 5. 生成恒定减速时间与加速度向量
  double T = v_mag / a_brake;
  T = std::max(0.05, T);  // 防止时间过短
  Eigen::Vector3d a_final = -a_brake * dir;

  // 6. 填充 MINCO 要求的 [t^5, t^4, t^3, t^2, t^1, t^0] 系数矩阵
  // 对于匀减速运动： p(t) = p0 + v0*t + 0.5*a*t^2
  Eigen::MatrixXd coeffMat = Eigen::MatrixXd::Zero(3, 6);
  for (int axis = 0; axis < 3; ++axis) {
    coeffMat(axis, 5) = p0(axis);             // c0
    coeffMat(axis, 4) = v0(axis);             // c1
    coeffMat(axis, 3) = 0.5 * a_final(axis);  // c2
    // c3, c4, c5 保持为 0
  }

  out_traj.clear();
  out_traj.emplace_back(T, coeffMat);
  return true;

  return false;
}

}  // namespace traj_opt
