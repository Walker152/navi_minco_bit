#include <cmath>
class pid
{
public:
    explicit pid(double &kp, double &ki, double &kd, double &dt, double &dead_band, double &min_val, double &max_val);
    double compute(double target, double current);

private:
    inline void limit(double &output)
    {
        output = (output < min_val_) ? min_val_ : (output > max_val_) ? max_val_
                                                                      : output;
    }

    // parameters:
    // 1. 基本参数
    double kp_, ki_, kd_, dt_;
    // 2. 已积分值
    double intetral_;
    // 3. 附属参数
    // 死区阈值
    double dead_band_;
    // 输出限幅阈值
    double min_val_;
    double max_val_;
    // 积分阈值

    // 前馈增益

    // 滤波系数
};
