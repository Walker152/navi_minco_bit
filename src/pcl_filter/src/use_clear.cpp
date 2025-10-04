#include "clear.hpp"



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pclfilter::ClearNode>("clear_it");

    int way = 2;
    if (way == 1)
    {
        node->init_odom();
    }
    else if (way == 2)
    {
        node->init_basemap();
    }
    else
    {
        node->init_mapwithodom();
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}