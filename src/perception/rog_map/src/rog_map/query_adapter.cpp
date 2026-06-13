#include <rog_map/query_adapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rog_map {

namespace {

bool sampleBilinear(
  const MapSnapshot & snap, int ix, int iy, double fx, double fy, double & dist, Eigen::Vector3d & grad)
{
  const size_t width = static_cast<size_t>(snap.width);
  const size_t idx00 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix);
  const size_t idx10 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix + 1);
  const size_t idx01 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix);
  const size_t idx11 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix + 1);

  const double d00 = snap.distances[idx00];
  const double d10 = snap.distances[idx10];
  const double d01 = snap.distances[idx01];
  const double d11 = snap.distances[idx11];
  if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
    return false;
  }

  const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
  const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
  dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;
  if (!std::isfinite(dist)) {
    return false;
  }

  grad.x() = ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / snap.resolution;
  grad.y() = ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

bool sampleQuadratic(
  const MapSnapshot & snap, int ix, int iy, double fx, double fy, double & dist, Eigen::Vector3d & grad)
{
  if (ix < 1 || iy < 1 || ix + 1 >= snap.width || iy + 1 >= snap.height) {
    return false;
  }
  const size_t width = static_cast<size_t>(snap.width);
  const auto sample = [&snap, width](int x, int y) {
    return snap.distances[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
  };
  const double wx[3] = {0.5 * fx * (fx - 1.0), 1.0 - fx * fx, 0.5 * fx * (fx + 1.0)};
  const double wy[3] = {0.5 * fy * (fy - 1.0), 1.0 - fy * fy, 0.5 * fy * (fy + 1.0)};
  const double dwx[3] = {fx - 0.5, -2.0 * fx, fx + 0.5};
  const double dwy[3] = {fy - 0.5, -2.0 * fy, fy + 0.5};

  dist = 0.0;
  double dd_dx_pix = 0.0;
  double dd_dy_pix = 0.0;
  for (int dy = 0; dy < 3; ++dy) {
    for (int dx = 0; dx < 3; ++dx) {
      const double d = sample(ix + dx - 1, iy + dy - 1);
      if (!std::isfinite(d)) {
        return false;
      }
      dist += wx[dx] * wy[dy] * d;
      dd_dx_pix += dwx[dx] * wy[dy] * d;
      dd_dy_pix += wx[dx] * dwy[dy] * d;
    }
  }
  if (!std::isfinite(dist)) {
    return false;
  }
  grad.x() = dd_dx_pix / snap.resolution;
  grad.y() = dd_dy_pix / snap.resolution;
  grad.z() = 0.0;
  if (!grad.allFinite()) {
    grad.setZero();
  }
  return true;
}

void clampResult(const MapSnapshot & snap, double & dist, Eigen::Vector3d & grad)
{
  if (!std::isfinite(dist)) {
    dist = snap.field_max_distance;
    grad.setZero();
    return;
  }
  if (snap.field_clamp_distance) {
    const double unclamped = dist;
    dist = std::clamp(dist, snap.field_min_distance, snap.field_max_distance);
    if (unclamped != dist) {
      grad.setZero();
    }
  }
  if (!grad.allFinite()) {
    grad.setZero();
  }
}

QueryResult makeResult(const MapSnapshot & snap, QueryStatus status)
{
  QueryResult result;
  result.status = status;
  result.projection_sequence = snap.projection_sequence;
  result.mask_sequence = snap.mask_sequence;
  result.field_sequence = snap.field_sequence;
  result.snapshot_sequence = snap.snapshot_sequence != 0U ? snap.snapshot_sequence : snap.sequence;
  result.field_age_ms =
    snap.field_stamp > 0.0 && snap.stamp >= snap.field_stamp ? (snap.stamp - snap.field_stamp) * 1000.0
                                                             : std::numeric_limits<double>::quiet_NaN();
  return result;
}

}  // namespace

void QueryAdapter::update(
  const std::shared_ptr<const MapSnapshot> & snapshot, const std::shared_ptr<DynamicLayer> & field)
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = snapshot;
  field_ = field;
}

std::shared_ptr<const MapSnapshot> QueryAdapter::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

QueryCounters QueryAdapter::queryCounters() const
{
  std::lock_guard<std::mutex> lock(counters_mutex_);
  return counters_;
}

void recordQueryStatus(QueryCounters & counters, QueryStatus status)
{
  if (status == QueryStatus::OK) {
    ++counters.ok;
    return;
  }
  ++counters.failed;
  switch (status) {
  case QueryStatus::OK:
    break;
  case QueryStatus::OUT_OF_MAP:
    ++counters.out_of_map;
    break;
  case QueryStatus::SNAPSHOT_INVALID:
    ++counters.snapshot_invalid;
    break;
  case QueryStatus::FIELD_UNINITIALIZED:
    ++counters.field_uninitialized;
    break;
  case QueryStatus::INTERPOLATION_FAILED:
    ++counters.interpolation_failed;
    break;
  case QueryStatus::TF_FAILED:
    ++counters.tf_failed;
    break;
  case QueryStatus::NONFINITE_INPUT:
    ++counters.nonfinite_input;
    break;
  case QueryStatus::NONFINITE_OUTPUT:
    ++counters.nonfinite_output;
    break;
  }
}

QueryResult QueryAdapter::recorded(QueryResult result) const
{
  std::lock_guard<std::mutex> lock(counters_mutex_);
  recordQueryStatus(counters_, result.status);
  return result;
}

bool QueryAdapter::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  const auto snap = snapshot();
  if (!snap || snap->width <= 0 || snap->height <= 0 || snap->resolution <= 0.0) {
    return false;
  }
  if (wx < snap->origin_x || wy < snap->origin_y) {
    return false;
  }
  const int ix = static_cast<int>(std::floor((wx - snap->origin_x) / snap->resolution));
  const int iy = static_cast<int>(std::floor((wy - snap->origin_y) / snap->resolution));
  if (ix < 0 || iy < 0 || ix >= snap->width || iy >= snap->height) {
    return false;
  }
  mx = static_cast<unsigned int>(ix);
  my = static_cast<unsigned int>(iy);
  return true;
}

void QueryAdapter::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  const auto snap = snapshot();
  if (!snap || snap->resolution <= 0.0) {
    wx = 0.0;
    wy = 0.0;
    return;
  }
  wx = snap->origin_x + (static_cast<double>(mx) + 0.5) * snap->resolution;
  wy = snap->origin_y + (static_cast<double>(my) + 0.5) * snap->resolution;
}

unsigned int QueryAdapter::sizeX() const
{
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->width) : 0U;
}

unsigned int QueryAdapter::sizeY() const
{
  const auto snap = snapshot();
  return snap ? static_cast<unsigned int>(snap->height) : 0U;
}

double QueryAdapter::resolution() const
{
  const auto snap = snapshot();
  return snap ? snap->resolution : 0.0;
}

double QueryAdapter::originX() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_x : 0.0;
}

double QueryAdapter::originY() const
{
  const auto snap = snapshot();
  return snap ? snap->origin_y : 0.0;
}

uint8_t QueryAdapter::value(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    return 254U;
  }
  const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
  return snap->values[idx];
}

const unsigned char * QueryAdapter::values() const
{
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    return nullptr;
  }
  return snap->values.data();
}

bool QueryAdapter::copyValues(std::vector<unsigned char> & out) const
{
  const auto snap = snapshot();
  if (!snap || snap->values.empty()) {
    out.clear();
    return false;
  }
  out.assign(snap->values.begin(), snap->values.end());
  return true;
}

bool QueryAdapter::isValid(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  return snap && mx < static_cast<unsigned int>(snap->width) &&
         my < static_cast<unsigned int>(snap->height);
}

bool QueryAdapter::isFree(unsigned int mx, unsigned int my) const
{
  const auto snap = snapshot();
  if (!snap || mx >= static_cast<unsigned int>(snap->width) ||
      my >= static_cast<unsigned int>(snap->height)) {
    return false;
  }
  const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
  return snap->values[idx] < 253U;
}

QueryResult QueryAdapter::query(const Eigen::Vector3d & pos) const
{
  // 向规划器暴露二维 ESDF 查询；距离来自 field mask，不直接来自 layer_value。
  // quadratic 在边界或邻域不足时回退到 bilinear，失败用 QueryStatus 显式表达。
  QueryResult invalid;
  if (!pos.allFinite()) {
    invalid.status = QueryStatus::NONFINITE_INPUT;
    return recorded(invalid);
  }

  const auto snap = snapshot();
  if (!snap || snap->width <= 1 || snap->height <= 1 || snap->resolution <= 0.0 ||
      snap->values.size() != static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
    return recorded(invalid);
  }
  if (snap->distances.size() != static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
    return recorded(makeResult(*snap, QueryStatus::FIELD_UNINITIALIZED));
  }

  const double px = (pos.x() - snap->origin_x) / snap->resolution;
  const double py = (pos.y() - snap->origin_y) / snap->resolution;
  const int ix = static_cast<int>(std::floor(px));
  const int iy = static_cast<int>(std::floor(py));
  if (ix < 0 || iy < 0 || ix >= snap->width - 1 || iy >= snap->height - 1) {
    return recorded(makeResult(*snap, QueryStatus::OUT_OF_MAP));
  }

  const double fx = px - static_cast<double>(ix);
  const double fy = py - static_cast<double>(iy);

  double dist = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  bool ok = false;
  if (snap->interpolation == InterpolationMode::QUADRATIC) {
    ok = sampleQuadratic(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    ok = sampleBilinear(*snap, ix, iy, fx, fy, dist, grad);
  }
  if (!ok) {
    return recorded(makeResult(*snap, QueryStatus::INTERPOLATION_FAILED));
  }

  clampResult(*snap, dist, grad);
  if (!std::isfinite(dist) || !grad.allFinite()) {
    return recorded(makeResult(*snap, QueryStatus::NONFINITE_OUTPUT));
  }

  QueryResult result = makeResult(*snap, QueryStatus::OK);
  result.ok = true;
  result.distance = dist;
  result.gradient = grad;
  return recorded(result);
}

bool QueryAdapter::evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
{
  const QueryResult result = query(pos);
  dist = result.distance;
  grad = result.gradient;
  return result.ok;
}

}  // namespace rog_map
