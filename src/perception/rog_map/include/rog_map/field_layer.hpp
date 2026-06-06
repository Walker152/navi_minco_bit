#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <limits>
#include <mutex>
#include <vector>

namespace rog_map {

class DynamicLayer {
public:
    DynamicLayer() = default;

    void updateFromMask(
        int width,
        int height,
        double resolution,
        const Eigen::Vector2d &origin,
        const std::vector<uint8_t> &mask,
        double inflation_radius);

    void rebuild(
        int width,
        int height,
        double resolution,
        const Eigen::Vector2d &origin,
        const std::vector<uint8_t> &mask,
        double inflation_radius);

    void evaluate(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const;

    bool isValid() const;
    int width() const;
    int height() const;
    double resolution() const;
    Eigen::Vector2d origin() const;
    std::vector<double> distances() const;

private:
    static constexpr double kFarDistance = 10.0;
    static constexpr double kESDFStrength = -std::numeric_limits<double>::infinity();

    mutable std::mutex mutex_;
    std::vector<double> dist_m_;
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    Eigen::Vector2d origin_{0.0, 0.0};
};

}  // namespace rog_map
