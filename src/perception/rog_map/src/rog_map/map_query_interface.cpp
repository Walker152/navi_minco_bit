#include <rog_map/map_query_interface.hpp>

#include <cmath>

namespace rog_map {

const char * queryStatusName(QueryStatus status)
{
  switch (status) {
  case QueryStatus::OK:
    return "OK";
  case QueryStatus::OUT_OF_MAP:
    return "OUT_OF_MAP";
  case QueryStatus::SNAPSHOT_INVALID:
    return "SNAPSHOT_INVALID";
  case QueryStatus::FIELD_UNINITIALIZED:
    return "FIELD_UNINITIALIZED";
  case QueryStatus::INTERPOLATION_FAILED:
    return "INTERPOLATION_FAILED";
  case QueryStatus::TF_FAILED:
    return "TF_FAILED";
  case QueryStatus::NONFINITE_INPUT:
    return "NONFINITE_INPUT";
  case QueryStatus::NONFINITE_OUTPUT:
    return "NONFINITE_OUTPUT";
  }
  return "UNKNOWN";
}

QueryResult MapQueryInterface::query(const Eigen::Vector3d & pos) const
{
  QueryResult result;
  if (!pos.allFinite()) {
    result.status = QueryStatus::NONFINITE_INPUT;
    return result;
  }

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  if (!evaluate(pos, dist, grad)) {
    result.status = QueryStatus::INTERPOLATION_FAILED;
    return result;
  }
  if (!std::isfinite(dist) || !grad.allFinite()) {
    result.status = QueryStatus::NONFINITE_OUTPUT;
    return result;
  }

  result.ok = true;
  result.status = QueryStatus::OK;
  result.distance = dist;
  result.gradient = grad;
  return result;
}

}  // namespace rog_map
