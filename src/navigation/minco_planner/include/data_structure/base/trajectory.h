#ifndef DATA_STRUCTURE_BASE_TRAJECTORY_H_
#define DATA_STRUCTURE_BASE_TRAJECTORY_H_

#include <vector>
#include <Eigen/Eigen>

namespace traj_opt {

class Trajectory {
public:
    Trajectory() = default;
    
    // Add necessary members and methods based on usage in Minco
    // For now, just a container for pieces
    struct Piece {
        double duration;
        Eigen::Matrix<double, 3, 8> coeffs; // 3 dimensions, 8 coefficients (degree 7)
    };
    
    std::vector<Piece> pieces;
    
    void reserve(size_t n) { pieces.reserve(n); }
    void emplace_back(const Piece& p) { pieces.emplace_back(p); }
    void clear() { pieces.clear(); }
    
    // Helper to evaluate position at time t
    Eigen::Vector3d getPos(double t) const {
        // Simple implementation for now
        return Eigen::Vector3d::Zero();
    }
};

} // namespace traj_opt

#endif // DATA_STRUCTURE_BASE_TRAJECTORY_H_
