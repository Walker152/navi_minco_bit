#include "../include/clear.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto clear_node = std::make_shared<pclfilter::clear>();
    
    int way = 2;  // 1: init_odom, 2: init_basemap, 3: init_mapwithodom
    
    if (way == 1)
    {
        clear_node->init_odom();
    }
    else if (way == 2)
    {
        clear_node->init_basemap();
    }
    else
    {
        clear_node->init_mapwithodom();
    }
    
    rclcpp::spin(clear_node);
    rclcpp::shutdown();
    return 0;
}
