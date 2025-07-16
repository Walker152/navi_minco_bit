#pragma once
namespace local_planner
{
    class super_control
    {
    private:
    public:
        void get_plan();
        void plan_opt();
    };

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    local_planner::super_control, nav2_core::Controller)