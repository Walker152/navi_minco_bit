#include "clear.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>

using namespace std::chrono_literals;
namespace pclfilter
{
    void ClearNode::init_basemap()
    {
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 1, std::bind(&ClearNode::basemapCloudCB, this, std::placeholders::_1));//这里的消息队列大小只有1是为什么？
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 1);
        
        if (!loadcubeParam("cube"))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load polygons-Yaml");
        }
    }

    void ClearNode::basemapCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
    {
        std::string source_frame = input_msg->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "sourceID:%s", source_frame.c_str());
        std::string target_frame = "base_link";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *cloud_in);

        // 安全检查
        if(cloud_in->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty cloud after filtering!");
            return;
        }

        for(size_t i = 0; i < min_pts_.size(); ++i)
        {
            size_t before_size = cloud_in->size();
            once_filter(cloud_in, min_pts_[i], max_pts_[i]);
            size_t after_size = cloud_in->size();
              
            RCLCPP_INFO(this->get_logger(), "from %zu to %zu ", before_size, after_size);
        }

        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_in, filtered_msg);
        filtered_msg.header.frame_id = source_frame;
        filtered_msg.header.stamp = input_msg->header.stamp;
        
        // transform
        try
        {
            if (!tf_buffer_->canTransform(target_frame, source_frame, input_msg->header.stamp, 100ms))
            {
                RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s at time %u.%u", 
                    source_frame.c_str(), target_frame.c_str(), 
                    input_msg->header.stamp.sec, input_msg->header.stamp.nanosec);
                return;     
            }

            auto transformed_msg = tf_buffer_->transform(filtered_msg, target_frame, tf2::durationFromSec(0.1));
            cloud_pub_->publish(transformed_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF transform exception: %s", ex.what());
            return;
        }
    }

    inline void printZCoordinates(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
    {
        for (size_t i = 0; i < cloud_in->points.size(); ++i)
        {
            float z = cloud_in->points[i].z;
            std::cout << "Point " << i << " z: " << z << std::endl;
        }
    }

    void ClearNode::once_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt)
    {
        if (cloud_in->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty cloud passed to filter");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(cloud_in);
        crop_box_filter.setMin(min_pt);
        printZCoordinates(cloud_in);
        crop_box_filter.setMax(max_pt);
        crop_box_filter.setNegative(true);

        crop_box_filter.filter(*cloud_filtered);
        *cloud_in = *cloud_filtered;
    }

    void ClearNode::init_odom()
    {
        basemap_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 1, std::bind(&ClearNode::cloudCB, this, std::placeholders::_1));
        
        if (!loadparam("polygons"))                                 //这里读region.yaml
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load polygons-Yaml");
        }
    }
    //还没写

    void ClearNode::init_mapwithodom()
    {
        if_clear_ = false;
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cloud_registered", 1, std::bind(&ClearNode::combineCloudCB, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_topic", 1, std::bind(&ClearNode::odomCB, this, std::placeholders::_1));
        
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_filter_baselink", 1);
        
        if (!loadparam("polygons"))                                 //这里读region.yaml                 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load polygons-Yaml");
        }
    }

    void ClearNode::combineCloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
    {
        (void)input_msg;
        // TODO:结合里程计进行哪多边形进行判断
    }                   

    void ClearNode::odomCB(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        x_ = odom_msg->pose.pose.position.x;
        y_ = odom_msg->pose.pose.position.y;
        if_clear_ = is_non_area(x_, y_);
    }

    void ClearNode::cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input_msg)
    {
        sensor_msgs::msg::PointCloud2 cloud_in_base_link;
        std::string source_frame = input_msg->header.frame_id;
        std::string target_frame = "base_link";

        if (if_clear_)
        {
            sensor_msgs::msg::PointCloud2 empty_cloud;
            empty_cloud.header = cloud_in_base_link.header;
            empty_cloud.height = 0;
            empty_cloud.width = 0;
            empty_cloud.is_dense = false;
            empty_cloud.is_bigendian = false;
            empty_cloud.fields.clear();
            empty_cloud.data.clear();

            cloud_pub_->publish(empty_cloud);
            RCLCPP_INFO(this->get_logger(), "Clear!");
        }
        else
        {
            try
            {
                if (!tf_buffer_->canTransform(target_frame, source_frame, input_msg->header.stamp, 1s))
                {
                    RCLCPP_WARN(this->get_logger(), "Cannot transform from %s to %s at time %u.%u", 
                        source_frame.c_str(), target_frame.c_str(), 
                        input_msg->header.stamp.sec, input_msg->header.stamp.nanosec);
                    return;
                }
                cloud_in_base_link = tf_buffer_->transform(*input_msg, target_frame, tf2::durationFromSec(1.0));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF transform exception: %s", ex.what());
                return;
            }
            cloud_pub_->publish(cloud_in_base_link);
        }
    }

    void ClearNode::once_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt,
        double cubeyaw)
    {
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(cloud_in);
        crop_box_filter.setMin(min_pt);
        crop_box_filter.setMax(max_pt);
        crop_box_filter.setNegative(true);
        crop_box_filter.setRotation(Eigen::Vector3f(0, 0, cubeyaw));
        crop_box_filter.filter(*cloud_in);
    }

    bool ClearNode::is_non_area(double x, double y)
    {
        for (const auto &poly : polygons_)
        {
            int cnt = 0;
            size_t n = poly.size();
            for (size_t i = 0; i < n; i++)
            {
                const Point2D &p1 = poly[i];
                const Point2D &p2 = poly[(i + 1) % n];
                if (((p1.y > y) != (p2.y > y)) &&
                    (x < (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y) + p1.x))
                {
                    cnt++;
                }
            }
            if (cnt % 2 == 1)
            {
                return true;
            }
        }
        return false;
    }


    bool ClearNode::loadparam(const std::string &param_name)
    {
        // 步骤1: 声明并获取字符串参数
        // 在ROS 2参数服务器中声明一个字符串参数，默认值为空数组JSON字符串
        this->declare_parameter<std::string>(param_name, "[]");
    
        std::string param_value;
        try {
        // 从参数服务器获取参数值
            param_value = this->get_parameter(param_name).as_string();
        } catch (const rclcpp::ParameterTypeException& e) {
        // 异常处理：参数类型不匹配（例如参数不是字符串类型）
            RCLCPP_ERROR(this->get_logger(), "Failed to get param %s: %s", 
                        param_name.c_str(), e.what());
            return false;
        }

        // 步骤2: 使用nlohmann/json库解析JSON字符串
        try {
        // 将参数字符串解析为JSON对象
            auto json_data = nlohmann::json::parse(param_value);
        
        // 检查JSON数据是否为数组类型
            if (!json_data.is_array()) {
                RCLCPP_ERROR(this->get_logger(), "Param %s is not a JSON array", param_name.c_str());
                return false;
            }
        
        // 清空现有的多边形数据
            polygons_.clear();
        
        // 步骤3: 遍历JSON数组中的每个多边形
            for (size_t i = 0; i < json_data.size(); ++i) {
                const auto& polygon_json = json_data[i];
            
            // 检查多边形数据是否为数组类型
                if (!polygon_json.is_array()) {
                    RCLCPP_WARN(this->get_logger(), "Polygon %zu is not an array of points", i);
                    continue;  // 跳过无效的多边形数据
                }
            
            // 创建新的多边形对象
                Polygon2D polygon;
            
            // 步骤4: 遍历多边形中的每个点
                for (size_t j = 0; j < polygon_json.size(); ++j) {
                    const auto& point_json = polygon_json[j];
                
                // 检查点数据是否为对象类型（JSON对象）
                    if (!point_json.is_object()) {
                        RCLCPP_WARN(this->get_logger(), "Point %zu in polygon %zu is not an object", j, i);
                        continue;  // 跳过无效的点数据
                    }
                
                // 检查点对象是否包含x和y字段，并且字段值为数字类型
                    if (point_json.contains("x") && point_json.contains("y") &&
                        point_json["x"].is_number() && point_json["y"].is_number()) {
                    
                    // 创建点对象并设置坐标
                        Point2D pt;
                        pt.x = point_json["x"].get<double>();  // 获取x坐标
                        pt.y = point_json["y"].get<double>();  // 获取y坐标
                        polygon.push_back(pt);  // 将点添加到多边形中
                    } else {
                    // 警告：点数据缺少必要字段或字段类型不正确
                        RCLCPP_WARN(this->get_logger(), "Point %zu in polygon %zu missing x or y, or type incorrect", j, i);
                    }
                }
            
            // 只有当多边形包含至少一个点时才添加到列表中
                if (!polygon.empty()) {
                    polygons_.push_back(polygon);
                }
            }
        
        // 成功加载后的日志输出
            RCLCPP_INFO(this->get_logger(), 
                    "Loaded %lu polygons from param %s", 
                    polygons_.size(), param_name.c_str());
            return true;  // 加载成功
        
        } catch (const nlohmann::json::exception& e) {
        // JSON解析异常处理
            RCLCPP_ERROR(this->get_logger(), 
                    "Failed to parse JSON from param %s: %s", 
                    param_name.c_str(), e.what());
            return false;  // 加载失败
        } catch (const std::exception& e) {
        // 其他未知异常处理
            RCLCPP_ERROR(this->get_logger(), 
                    "Unexpected error with param %s: %s", 
                    param_name.c_str(), e.what());
            return false;  // 加载失败
        }
    }

/*   bool ClearNode::loadcubeParam()
    {
        this->declare_parameter<std::float_t>(cube_min_x, 0);
        this->declare_parameter<std::float_t>(cube_min_y, 0);
        this->declare_parameter<std::float_t>(cube_min_z, 0);
        this->declare_parameter<std::float_t>(cube_min_w, 0);
        this->declare_parameter<std::float_t>(cube_max_x, 0);
        this->declare_parameter<std::float_t>(cube_max_y, 0);
        this->declare_parameter<std::float_t>(cube_max_z, 0);
        this->declare_parameter<std::float_t>(cube_max_w, 0);
        
        min_pts_.clear();
        max_pts_.clear();

        auto greeting = this->get_parameter("greeting").as_string();
        auto name = this->get_parameter("name").as_string();
        auto exposure_time = this->get_parameter("ExposureTime").as_int();
    }
*/

    bool ClearNode::loadcubeParam(const std::string &param_name)
    {
        // 步骤1: 声明并获取参数，默认值设置为一个空的JSON数组字符串
        this->declare_parameter("cube", "[]"); // 默认空数组
    
        std::string param_value;
        try {
        // 尝试从参数服务器获取字符串类型的参数值
            param_value = this->get_parameter(param_name).as_string();
        } catch (const rclcpp::ParameterTypeException& e) {
        // 处理参数类型异常（例如，参数存在但不是字符串类型）
            RCLCPP_ERROR(this->get_logger(), "Failed to get param '%s': %s", param_name.c_str(), e.what());
            return false;
        }

    // 步骤2: 使用nlohmann/json库解析获取到的JSON字符串
        try {
            auto cuboid_list = nlohmann::json::parse(param_value); // 解析JSON字符串

        // 检查解析后的数据是否为数组类型
            if (!cuboid_list.is_array()) {
                RCLCPP_ERROR(this->get_logger(), "Param '%s' is not a JSON array", param_name.c_str());
                return false;
            }

        // 清空现有的立方体最小点和最大点列表
            min_pts_.clear();
            max_pts_.clear();

        // 步骤3: 遍历JSON数组中的每个立方体对象
            for (size_t i = 0; i < cuboid_list.size(); ++i) {
                const auto& cuboid = cuboid_list[i];
            
                // 检查当前立方体对象是否包含必需的"min"和"max"字段
                if (!cuboid.contains("min") || !cuboid.contains("max")) {
                    RCLCPP_ERROR(this->get_logger(), "Cuboid element %zu does not have min/max", i);
                    continue; // 跳过这个无效的立方体元素
                }

                const auto& min_val = cuboid["min"];
                const auto& max_val = cuboid["max"];

                // 检查"min"和"max"字段是否为数组且长度是否为4
                if (!min_val.is_array() || !max_val.is_array() || min_val.size() != 4 || max_val.size() != 4) {
                    RCLCPP_ERROR(this->get_logger(), "Cuboid element %zu min/max not 4 elements", i);
                    continue; // 跳过这个无效的立方体元素
                }

                Eigen::Vector4f min_pt, max_pt;
                // 步骤4: 遍历"min"和"max"数组中的每个元素（应为4个）
                for (size_t j = 0; j < 4; ++j) {
                // 处理min_val[j]
                    if (min_val[j].is_number()) {
                        min_pt[j] = min_val[j].get<float>(); // 直接获取float值，JSON库处理类型转换
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Cuboid element %zu min_val[%zu] is not a number", i, j);
                        return false; // 遇到非数字类型，直接返回失败
                    }

                // 处理max_val[j]
                    if (max_val[j].is_number()) {
                        max_pt[j] = max_val[j].get<float>(); // 直接获取float值，JSON库处理类型转换
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Cuboid element %zu max_val[%zu] is not a number", i, j);
                        return false; // 遇到非数字类型，直接返回失败
                    }
                }
                // 将解析成功的最小点和最大点添加到成员变量中
                min_pts_.push_back(min_pt);
                max_pts_.push_back(max_pt);
            }
            // 打印成功加载的立方体数量信息
            RCLCPP_INFO(this->get_logger(), "Loaded %lu cuboids from param '%s'", min_pts_.size(), param_name.c_str());
            return true; // 加载成功

        } catch (const nlohmann::json::exception& e) {
            // 处理JSON解析过程中可能出现的异常
            RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON from param '%s': %s", param_name.c_str(), e.what());
            return false;
        } catch (const std::exception& e) {
            // 处理其他可能的异常
            RCLCPP_ERROR(this->get_logger(), "Unexpected error with param '%s': %s", param_name.c_str(), e.what());
            return false;
        }
    }
}
