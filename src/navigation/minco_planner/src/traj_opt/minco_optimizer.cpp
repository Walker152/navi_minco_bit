#include "traj_opt/minco_optimizer.hpp"

namespace minco_planner {

bool MincoOptimizer::optimize(const std::vector<Eigen::Vector3d>& waypoints,
                             const Eigen::Matrix3d& start_state,
                             const Eigen::Matrix3d& end_state,
                             geometry_utils::Trajectory& out_traj) 
{
    int N = waypoints.size() - 1; // 多项式段数
    
    if (N == 0) return false;

    // MINCO_S3NU 需要 headPVA 和 tailPVA
    // start_state 和 end_state 分别是 3x3 矩阵 (Col0: P, Col1: V, Col2: A)
    Eigen::Matrix3d headState = start_state;
    Eigen::Matrix3d tailState = end_state;
    
    // 强制修正起终点位置为抽稀后的起终点（防止微小误差）
    headState.col(0) = waypoints.front();
    tailState.col(0) = waypoints.back();

    // 3. 提取中间点
    Eigen::Matrix3Xd innerPoints;
    if (N > 1) {
        innerPoints.resize(3, N - 1);
        for (int i = 0; i < N - 1; ++i) {
            innerPoints.col(i) = waypoints[i + 1];
        }
    } else {
        innerPoints.resize(3, 0);
    }

    // 4. 初始时间分配
    Eigen::VectorXd times = allocateInitialTimes(waypoints);

    // 5. 迭代求解与时间重分配 (Hard Constraint Loop)
    bool satisfy_constraints = false;
    if (!minco_solver_) {
        std::cerr << "[MincoOptimizer] minco_solver_ is null!" << std::endl;
        return false;
    }
    minco_solver_->setConditions(headState, tailState, N);
    
    for (int iter = 0; iter < config_.time_allocation_iters; ++iter) {
        // A. 闭式求解 Ax=b
        // minco.h 中的 getTrajectory 会自动计算系数并填入 out_traj
        minco_solver_->setParameters(innerPoints, times);
        minco_solver_->getTrajectory(out_traj); 

        // B. 检查约束
        Eigen::VectorXd new_times;
        double max_ratio = checkConstraints(out_traj, times, new_times);

        // C. 判断收敛
        if (max_ratio <= 1.01) { // 允许 1% 的误差
            satisfy_constraints = true;
            break;
        }

        // D. 更新时间
        times = new_times;
    }

    if (!satisfy_constraints) {
        // 如果迭代耗尽仍未完全满足，通常是因为极短的段导致加速度极大
        // 此时 out_traj 依然是一条连续可行的轨迹，只是可能略微超限
        // 在 RM 比赛中，可以直接输出，底盘控制器会有限幅保护
        // 或者在这里做一个 fallback 处理
    }

    return true;
}

Eigen::VectorXd MincoOptimizer::allocateInitialTimes(const std::vector<Eigen::Vector3d>& waypoints) {
    int N = waypoints.size() - 1;
    Eigen::VectorXd times(N);
    for (int i = 0; i < N; ++i) {
        double dist = (waypoints[i+1] - waypoints[i]).norm();
        // 初始猜测：用较慢的速度，留出裕量
        double t = dist / (config_.max_vel * 0.6); 
        times(i) = std::max(0.1, t); // 最小时间保护
    }
    return times;
}

double MincoOptimizer::checkConstraints(const geometry_utils::Trajectory& traj, 
                                        const Eigen::VectorXd& times,
                                        Eigen::VectorXd& new_times) 
{
    double global_max_ratio = 1.0;
    new_times = times;
    int N = traj.getPieceNum(); // Trajectory 类的方法

    // 获取所有段的时长
    Eigen::VectorXd durs = traj.getDurations(); 
    for (int i = 0; i < N; ++i) {
        double duration = durs(i);
        double seg_max_v = 0.0;
        double seg_max_a = 0.0;

        // 采样检查：在当前段内采样
        int samples = 15; 
        for (int k = 0; k <= samples; ++k) {
            double t_local = duration * k / samples;
            
            Eigen::Vector3d v = traj[i].getVel(t_local);
            Eigen::Vector3d a = traj[i].getAcc(t_local);

            if (v.norm() > seg_max_v) seg_max_v = v.norm();
            if (a.norm() > seg_max_a) seg_max_a = a.norm();
        }

        // 计算缩放比例
        double v_ratio = (seg_max_v > config_.max_vel) ? (seg_max_v / config_.max_vel) : 1.0;
        double a_ratio = (seg_max_a > config_.max_acc) ? std::sqrt(seg_max_a / config_.max_acc) : 1.0;
        
        double ratio = std::max(v_ratio, a_ratio);
        
        // 如果违规，拉长时间
        if (ratio > 1.0) {
            new_times(i) = times(i) * ratio * 1.05; // 1.05 倍余量加速收敛
            global_max_ratio = std::max(global_max_ratio, ratio);
        } else {
            new_times(i) = times(i);
        }
    }
    return global_max_ratio;
}

} // namespace minco_planner