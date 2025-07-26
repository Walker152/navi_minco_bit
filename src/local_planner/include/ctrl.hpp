// C++
#include <iostream>
#include <string.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
// Algorithm
#include "myutils/pid/pid.hpp"
//
namespace local_planner
{
    // using std::string;
    class ctrl
    {
    public:
        explicit ctrl(cosnt std::string &node);
        ~ctrl();

    private:
        pid pid_;

        // parameters
        struct parameters_pid_
        {
            double kp, ki, kd, dt;
            double intetral;
            double dead_band;
        };
    };

}