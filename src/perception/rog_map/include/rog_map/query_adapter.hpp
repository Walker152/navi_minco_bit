#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rog_map/field_layer.hpp>
#include <rog_map/map_query_interface.hpp>

namespace rog_map {

struct MapSnapshot {
    int width{0};
    int height{0};
    double resolution{0.0};
    double origin_x{0.0};
    double origin_y{0.0};
    std::vector<uint8_t> values;
    std::vector<uint8_t> types;
    std::vector<double> distances;
};

class QueryAdapter : public MapQueryInterface {
public:
    void update(
        const std::shared_ptr<const MapSnapshot> &snapshot,
        const std::shared_ptr<DynamicLayer> &field);

    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const override;
    void mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) const override;

    unsigned int sizeX() const override;
    unsigned int sizeY() const override;
    double resolution() const override;
    double originX() const override;
    double originY() const override;

    uint8_t value(unsigned int mx, unsigned int my) const override;
    const unsigned char *values() const override;
    bool copyValues(std::vector<unsigned char> &out) const override;

    bool isValid(unsigned int mx, unsigned int my) const override;
    bool isFree(unsigned int mx, unsigned int my) const override;

    bool evaluate(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad) const override;

private:
    std::shared_ptr<const MapSnapshot> snapshot() const;

    mutable std::mutex mutex_;
    std::shared_ptr<const MapSnapshot> snapshot_;
    std::shared_ptr<DynamicLayer> field_;
};

}  // namespace rog_map
