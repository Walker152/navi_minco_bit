#include <rog_map/field_layer.hpp>

#include <rog_map/esdf_utils.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rog_map {

void DynamicLayer::updateFromMask(
    int width,
    int height,
    double resolution,
    const Eigen::Vector2d &origin,
    const std::vector<uint8_t> &mask,
    double inflation_radius) {
    rebuild(width, height, resolution, origin, mask, inflation_radius);
}

void DynamicLayer::rebuild(
    int width,
    int height,
    double resolution,
    const Eigen::Vector2d &origin,
    const std::vector<uint8_t> &mask,
    double inflation_radius) {
    if (width <= 0 || height <= 0 || resolution <= 0.0) {
        throw std::invalid_argument("DynamicLayer::rebuild: invalid grid metadata");
    }

    const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (mask.size() != expected) {
        throw std::invalid_argument("DynamicLayer::rebuild: mask size mismatch");
    }

    std::vector<double> dist_sq_pos;
    ESDFUtils::computeEDT2D(width, height, mask, dist_sq_pos);

    std::vector<uint8_t> inv_mask(expected, 0U);
    for (size_t i = 0; i < expected; ++i) {
        inv_mask[i] = (mask[i] == 0U) ? 1U : 0U;
    }

    std::vector<double> dist_sq_neg;
    ESDFUtils::computeEDT2D(width, height, inv_mask, dist_sq_neg);

    std::vector<double> dist_m(expected, kFarDistance);
    for (size_t i = 0; i < expected; ++i) {
        double raw = 0.0;
        if (mask[i] == 1U) {
            raw = (dist_sq_pos[i] >= 1.0e19) ? kFarDistance : std::sqrt(dist_sq_pos[i]) * resolution;
        } else {
            raw = (dist_sq_neg[i] >= 1.0e19) ? -kFarDistance : -std::sqrt(dist_sq_neg[i]) * resolution;
        }
        const double inflated = raw - inflation_radius;
        dist_m[i] = std::isfinite(inflated) ? inflated : ((mask[i] == 1U) ? kFarDistance : -kFarDistance);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = origin;
    dist_m_.swap(dist_m);
}

bool DynamicLayer::isValid() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return width_ > 1 && height_ > 1 && resolution_ > 0.0 && !dist_m_.empty();
}

int DynamicLayer::width() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return width_;
}

int DynamicLayer::height() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return height_;
}

double DynamicLayer::resolution() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return resolution_;
}

Eigen::Vector2d DynamicLayer::origin() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return origin_;
}

std::vector<double> DynamicLayer::distances() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dist_m_;
}

void DynamicLayer::evaluate(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (width_ <= 1 || height_ <= 1 || resolution_ <= 0.0 || dist_m_.empty()) {
        dist = kFarDistance;
        grad.setZero();
        return;
    }

    const double px = (pos.x() - origin_.x()) / resolution_;
    const double py = (pos.y() - origin_.y()) / resolution_;
    const int ix = static_cast<int>(std::floor(px));
    const int iy = static_cast<int>(std::floor(py));
    if (ix < 0 || iy < 0 || ix >= width_ - 1 || iy >= height_ - 1) {
        dist = kFarDistance;
        grad.setZero();
        return;
    }

    const double fx = px - static_cast<double>(ix);
    const double fy = py - static_cast<double>(iy);
    const size_t w = static_cast<size_t>(width_);
    const size_t idx00 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix);
    const size_t idx10 = static_cast<size_t>(iy) * w + static_cast<size_t>(ix + 1);
    const size_t idx01 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix);
    const size_t idx11 = static_cast<size_t>(iy + 1) * w + static_cast<size_t>(ix + 1);

    const double d00 = dist_m_[idx00];
    const double d10 = dist_m_[idx10];
    const double d01 = dist_m_[idx01];
    const double d11 = dist_m_[idx11];
    if (!std::isfinite(d00) || !std::isfinite(d10) || !std::isfinite(d01) || !std::isfinite(d11)) {
        dist = kESDFStrength;
        grad.setZero();
        return;
    }

    const double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
    const double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
    dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;

    const double dd_dx_pix = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
    const double dd_dy_pix = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);
    grad.x() = dd_dx_pix / resolution_;
    grad.y() = dd_dy_pix / resolution_;
    grad.z() = 0.0;
}

}  // namespace rog_map
