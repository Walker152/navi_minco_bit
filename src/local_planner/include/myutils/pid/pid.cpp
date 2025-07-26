#include "pid.hpp"

pid::pid(double &kp, double &ki, double &kd, double &dt, double &dead_band, double &min_val, double &max_val)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    dt = dt_;
    dead_band = dead_band_;
    min_val = min_val_;
    max_val = max_val_;
}

