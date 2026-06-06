#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <mutex>
#include <vector>

namespace rog_map {

enum class InterpolationMode {
    BILINEAR = 0,
    QUADRATIC = 1
};

class DynamicLayer {
public:
    DynamicLayer() = default;

    void updateFromMask(
        int width,
        int height,
        double resolution,
        const Eigen::Vector2d &origin,
        const std::vector<uint8_t> &mask,
        double inflation_radius,
        double max_distance = 3.0,
        double min_distance = -1.0,
        bool clamp_distance = true,
        InterpolationMode interpolation = InterpolationMode::BILINEAR);

    void rebuild(
        int width,
        int height,
        double resolution,
        const Eigen::Vector2d &origin,
        const std::vector<uint8_t> &mask,
        double inflation_radius,
        double max_distance = 3.0,
        double min_distance = -1.0,
        bool clamp_distance = true,
        InterpolationMode interpolation = InterpolationMode::BILINEAR);

    void evaluate(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const;

    bool isValid() const;
    int width() const;
    int height() const;
    double resolution() const;
    Eigen::Vector2d origin() const;
    std::vector<double> distances() const;
    double maxDistance() const;
    double minDistance() const;
    bool clampDistanceEnabled() const;
    InterpolationMode interpolationMode() const;

private:
    static constexpr double kFarDistance = 10.0;

    mutable std::mutex mutex_;
    std::vector<double> dist_m_;
    int width_{0};
    int height_{0};
    double resolution_{0.0};
    Eigen::Vector2d origin_{0.0, 0.0};
    double max_distance_{3.0};
    double min_distance_{-1.0};
    bool clamp_distance_{true};
    InterpolationMode interpolation_{InterpolationMode::BILINEAR};
};

}  // namespace rog_map
