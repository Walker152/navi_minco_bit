#include <rog_map/query_adapter.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace rog_map {

void QueryAdapter::update(
    const std::shared_ptr<const MapSnapshot> &snapshot,
    const std::shared_ptr<DynamicLayer> &field) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = snapshot;
    field_ = field;
}

std::shared_ptr<const MapSnapshot> QueryAdapter::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

bool QueryAdapter::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const {
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

void QueryAdapter::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const {
    const auto snap = snapshot();
    if (!snap || snap->resolution <= 0.0) {
        wx = 0.0;
        wy = 0.0;
        return;
    }
    wx = snap->origin_x + (static_cast<double>(mx) + 0.5) * snap->resolution;
    wy = snap->origin_y + (static_cast<double>(my) + 0.5) * snap->resolution;
}

unsigned int QueryAdapter::sizeX() const {
    const auto snap = snapshot();
    return snap ? static_cast<unsigned int>(snap->width) : 0U;
}

unsigned int QueryAdapter::sizeY() const {
    const auto snap = snapshot();
    return snap ? static_cast<unsigned int>(snap->height) : 0U;
}

double QueryAdapter::resolution() const {
    const auto snap = snapshot();
    return snap ? snap->resolution : 0.0;
}

double QueryAdapter::originX() const {
    const auto snap = snapshot();
    return snap ? snap->origin_x : 0.0;
}

double QueryAdapter::originY() const {
    const auto snap = snapshot();
    return snap ? snap->origin_y : 0.0;
}

uint8_t QueryAdapter::value(unsigned int mx, unsigned int my) const {
    const auto snap = snapshot();
    if (!snap || mx >= static_cast<unsigned int>(snap->width) || my >= static_cast<unsigned int>(snap->height)) {
        return 254U;
    }
    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
    return snap->values[idx];
}

const unsigned char *QueryAdapter::values() const {
    const auto snap = snapshot();
    if (!snap || snap->values.empty()) {
        return nullptr;
    }
    return snap->values.data();
}

bool QueryAdapter::copyValues(std::vector<unsigned char> &out) const {
    const auto snap = snapshot();
    if (!snap || snap->values.empty()) {
        out.clear();
        return false;
    }
    out.assign(snap->values.begin(), snap->values.end());
    return true;
}

bool QueryAdapter::isValid(unsigned int mx, unsigned int my) const {
    const auto snap = snapshot();
    return snap && mx < static_cast<unsigned int>(snap->width) && my < static_cast<unsigned int>(snap->height);
}

bool QueryAdapter::isFree(unsigned int mx, unsigned int my) const {
    const auto snap = snapshot();
    if (!snap || mx >= static_cast<unsigned int>(snap->width) || my >= static_cast<unsigned int>(snap->height)) {
        return false;
    }
    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(snap->width) + static_cast<size_t>(mx);
    return snap->values[idx] < 253U;
}

bool QueryAdapter::evaluate(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const {
    const auto snap = snapshot();
    if (!snap || snap->width <= 1 || snap->height <= 1 || snap->resolution <= 0.0 ||
        snap->distances.size() != static_cast<size_t>(snap->width) * static_cast<size_t>(snap->height)) {
        dist = 10.0;
        grad.setZero();
        return false;
    }

    const double px = (pos.x() - snap->origin_x) / snap->resolution;
    const double py = (pos.y() - snap->origin_y) / snap->resolution;
    const int ix = static_cast<int>(std::floor(px));
    const int iy = static_cast<int>(std::floor(py));
    if (ix < 0 || iy < 0 || ix >= snap->width - 1 || iy >= snap->height - 1) {
        dist = 10.0;
        grad.setZero();
        return false;
    }

    const double fx = px - static_cast<double>(ix);
    const double fy = py - static_cast<double>(iy);
    const size_t width = static_cast<size_t>(snap->width);
    const size_t idx00 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix);
    const size_t idx10 = static_cast<size_t>(iy) * width + static_cast<size_t>(ix + 1);
    const size_t idx01 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix);
    const size_t idx11 = static_cast<size_t>(iy + 1) * width + static_cast<size_t>(ix + 1);

    const double d00 = snap->distances[idx00];
    const double d10 = snap->distances[idx10];
    const double d01 = snap->distances[idx01];
    const double d11 = snap->distances[idx11];
    if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
        dist = -std::numeric_limits<double>::infinity();
        grad.setZero();
        return false;
    }

    const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
    const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
    dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;

    const double dd_dx_pix = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
    const double dd_dy_pix = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);
    grad.x() = dd_dx_pix / snap->resolution;
    grad.y() = dd_dy_pix / snap->resolution;
    grad.z() = 0.0;
    return true;
}

}  // namespace rog_map
