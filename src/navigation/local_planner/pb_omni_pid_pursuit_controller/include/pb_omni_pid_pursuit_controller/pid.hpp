#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_

/**
 * @class PID
 * @brief 经典PID控制器实现
 * 
 * @details 实现标准的比例-积分-微分控制算法：
 *          output = Kp*error + Ki*integral + Kd*derivative
 *          
 *          特性：
 *          - 支持输出限幅（防止控制量过大）
 *          - 积分项限幅（防止积分饱和）
 *          - 可重置积分累积值
 */
class PID
{
public:
  /**
   * @brief PID控制器构造函数
   * @param dt 控制周期（时间间隔）
   * @param max 输出最大值
   * @param min 输出最小值  
   * @param kp 比例增益（Proportional gain）- 决定对当前误差的响应强度
   * @param kd 微分增益（Derivative gain）- 决定对误差变化率的响应强度
   * @param ki 积分增益（Integral gain）- 决定对累积误差的响应强度
   */
  PID(double dt, double max, double min, double kp, double kd, double ki);

  /**
   * @brief 计算PID控制输出
   * @param set_point 目标值（期望值）
   * @param pv 当前过程值（Process Value，实际值）
   * @return 计算得到的控制量
   */
  double calculate(double set_point, double pv);
  
  /**
   * @brief 设置积分累积值
   * @param sum_error 新的积分累积值
   * @details 用于重置或调整积分项，防止积分饱和
   */
  void setSumError(double sum_error);
  
  /**
   * @brief 析构函数
   */
  ~PID();

private:
  double dt_;        ///< 控制周期
  double max_;       ///< 输出最大值
  double min_;       ///< 输出最小值
  double kp_;        ///< 比例增益
  double kd_;        ///< 微分增益
  double ki_;        ///< 积分增益
  double pre_error_; ///< 上一次的误差值（用于计算微分项）
  double integral_;  ///< 积分累积值
};

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__PID_HPP_
