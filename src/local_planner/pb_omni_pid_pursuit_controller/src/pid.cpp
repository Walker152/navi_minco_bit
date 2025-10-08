#include "pb_omni_pid_pursuit_controller/pid.hpp"

/**
 * @brief PID控制器构造函数实现
 * @details 初始化所有PID参数，并将误差和积分项清零
 */
PID::PID(double dt, double max, double min, double kp, double kd, double ki)
: dt_(dt), max_(max), min_(min), kp_(kp), kd_(kd), ki_(ki), pre_error_(0), integral_(0)
{
}

/**
 * @brief PID控制计算的核心实现
 * @param set_point 目标值（期望到达的值）
 * @param pv 当前过程值（实际测量值）
 * @return PID控制器的输出值
 * 
 * @details PID控制算法详解：
 *          1. 比例项(P)：Kp * error - 当前误差的直接响应
 *          2. 积分项(I)：Ki * ∫error*dt - 消除稳态误差
 *          3. 微分项(D)：Kd * d(error)/dt - 预测未来误差趋势
 *          
 *          积分饱和处理：限制积分项在[-1,1]范围内，防止积分饱和
 *          输出限幅：最终输出限制在[min_,max_]范围内
 */
double PID::calculate(double set_point, double pv)
{
  // 计算当前误差：目标值 - 实际值
  double error = set_point - pv;

  // 比例项：与当前误差成正比
  // 作用：提供主要的控制力度，误差越大，输出越大
  double p_out = kp_ * error;

  // 积分项：累积历史误差
  // 作用：消除稳态误差，长期存在的小误差会被积分放大
  integral_ += error * dt_;
  double i_out = ki_ * integral_;

  // 积分限幅：防止积分饱和（windup）
  // 当执行器饱和时，积分项会继续累积，导致系统响应迟缓
  if (integral_ > 1) {
    integral_ = 1;
  } else if (integral_ < -1) {
    integral_ = -1;
  }

  // 微分项：误差的变化率
  // 作用：预测未来趋势，提供阻尼效果，减少超调
  double derivative = (error - pre_error_) / dt_;
  double d_out = kd_ * derivative;

  // PID总输出：三项之和
  double output = p_out + i_out + d_out;

  // 输出限幅：确保控制量在执行器能力范围内
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // 保存当前误差，用于下次计算微分项
  pre_error_ = error;

  return output;
}

/**
 * @brief 设置积分累积值
 * @param sum_error 新的积分值
 * @details 用于重置积分项，通常在系统重启或状态切换时使用
 */
void PID::setSumError(double sum_error) { 
  integral_ = sum_error; 
}

/**
 * @brief 析构函数
 */
PID::~PID() {}
