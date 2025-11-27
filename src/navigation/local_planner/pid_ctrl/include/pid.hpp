#pragma once
#include <cmath>
namespace dreamchaser_ctrl
{
  struct PIDParams
  {
    double kp;              // 比例增益
    double ki;              // 积分增益
    double kd;              // 微分增益
    double max_output;      // 最大输出限幅
    double min_output;      // 最小输出限幅
    double integral_limit;  // 积分限幅，防止积分风暴
  };
  class PIDSolver
  {
  public:
    PIDSolver(const PIDParams& params);
    ~PIDSolver() = default;
    
    // 设置PID参数
    void setParameters(double kp, double ki, double kd);

    // 设置输出限幅
    void setOutputLimits(double min, double max);

    // 计算控制量
    double compute(double setpoint, double actual_value, double dt);

    // 重置积分项和上一次误差（用于控制器切换或防止积分饱和）
    void reset();
  };

}  // namespace dreamchaser_ctrl