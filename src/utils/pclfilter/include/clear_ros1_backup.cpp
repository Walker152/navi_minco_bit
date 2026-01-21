#include "../include/clear.hpp"

namespace pclfilter
{
    using Polygon2D = std::vector<Point2D>;
    clear::clear()
    {
        if_clear_ = false;
    }

    // main
    void clear::init_basemap()
    {
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/cloud_registered", 1, &clear::basemapCloudCB, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filter_baselink", 1);
        if (!loadcubeParam("cube"))
        {
            ROS_ERROR("Failed to load polygons-Yaml");
        }
    }

    void clear::basemapCloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg)
    {
        static tf2_ros::Buffer tf_buffer(ros::Duration(1));
        static tf2_ros::TransformListener tf_listener(tf_buffer);
        // Debug Start!
        // static size_t process_count = 0;
        // static ros::Duration total_duration(0);
        // ros::Time start_time = ros::Time::now();
        // Debug End!
        std::string source_frame = input_msg->header.frame_id;
        ROS_INFO("sourceID:%s", source_frame.c_str());
        std::string target_frame = "base_link";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *cloud_in);

        // 安全检查
        if (cloud_in->empty())
        {
            ROS_WARN("Empty cloud after filtering!");
            return;
        }

        for (size_t i = 0; i < min_pts_.size(); ++i)
        {
            size_t before_size = cloud_in->size();
            once_filter(cloud_in, min_pts_[i], max_pts_[i]);
            size_t after_size = cloud_in->size();
            ROS_INFO("from %zu to %zu ", before_size, after_size);
        }

        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_in, filtered_msg);
        filtered_msg.header.frame_id = source_frame;
        filtered_msg.header.stamp = input_msg->header.stamp;
        // transform
        try
        {
            if (!tf_buffer.canTransform(target_frame, source_frame, input_msg->header.stamp, ros::Duration(0.1)))
            {
                ROS_WARN("Cannot transform from %s to %s at time %u.%u", source_frame.c_str(), target_frame.c_str(), input_msg->header.stamp.sec, input_msg->header.stamp.nsec);
                return;     
            }

            sensor_msgs::PointCloud2 transformed_msg;
            tf_buffer.transform(filtered_msg, transformed_msg, target_frame, ros::Duration(0.1));
            cloud_pub_.publish(transformed_msg);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF transform exception: %s", ex.what());
            return;
        }
        // Debug Start!
        // ros::Duration duration = ros::Time::now() - start_time;
        // process_count++;
        // total_duration += duration;

        // if (process_count % 10 == 0)
        // {
        //     ROS_INFO("cost:%.6f", total_duration.toSec() / process_count);
        // }
        // Debug End!
    }
    inline void printZCoordinates(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in)
    {
        for (size_t i = 0; i < cloud_in->points.size(); ++i)
        {
            float z = cloud_in->points[i].z;
            std::cout << "Point " << i << " z: " << z << std::endl;
        }
    }
    void clear::once_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt)
    {
        // Debug prints to check filter limits
        // ROS_DEBUG("Filter min: [%f, %f, %f, %f]", min_pt[0], min_pt[1], min_pt[2], min_pt[3]);
        // ROS_DEBUG("Filter max: [%f, %f, %f, %f]", max_pt[0], max_pt[1], max_pt[2], max_pt[3]);

        // Check if cloud has points
        if (cloud_in->empty())
        {
            ROS_WARN("Empty cloud passed to filter");
            return;
        }

        // Create a new cloud for output to verify points are removed
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // Create CropBox filter
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(cloud_in);
        crop_box_filter.setMin(min_pt);
        // 假设 cloud_in 已经被正确赋值并包含点云数据
        printZCoordinates(cloud_in);
        crop_box_filter.setMax(max_pt);
        crop_box_filter.setNegative(true); // Remove points inside the box

        // Apply filter to separate cloud first to check results
        crop_box_filter.filter(*cloud_filtered);

        // Update the input cloud if filtering worked
        *cloud_in = *cloud_filtered;
    }
    void clear::init_odom()
    {
        basemap_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/cloud_registered", 1, &clear::cloudCB, this);
        if (!loadparam("polygons")) // 这里传参数名"polygons"
        {
            ROS_ERROR("Failed to load polygons-Yaml");
        }
    }
    // todo
    void clear::init_mapwithodom()
    {
        if_clear_ = false;
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/cloud_registered", 1, &clear::combineCloudCB, this);
        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
            "/odom_topic", 1, &clear::odomCB, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_filter_baselink", 1);
        if (!loadparam("polygons")) // 执行并且判断
        {
            ROS_ERROR("Failed to load polygons-Yaml");
        }
    }

    void clear::odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        x_ = odom_msg->pose.pose.position.x;
        y_ = odom_msg->pose.pose.position.y;
        if_clear_ = is_non_area(x_, y_);
        // todo:判断上坡之后，不使用清除功能
    }
    void clear::cloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg)
    {
        // way1:base_odom:Begin

        static tf2_ros::Buffer tf_buffer(ros::Duration(3));
        static tf2_ros::TransformListener tf_listener(tf_buffer);
        sensor_msgs::PointCloud2 cloud_in_base_link;
        std::string source_frame = input_msg->header.frame_id;
        std::string target_frame = "base_link";

        if (if_clear_)
        {

            // 构造空点云，header用转换后点云的header，发布空点云
            sensor_msgs::PointCloud2 empty_cloud;
            empty_cloud.header = cloud_in_base_link.header;
            empty_cloud.height = 0;
            empty_cloud.width = 0;
            empty_cloud.is_dense = false;
            empty_cloud.is_bigendian = false;
            empty_cloud.fields.clear();
            empty_cloud.data.clear();

            cloud_pub_.publish(empty_cloud);

            ROS_INFO("Clear!");
        }
        else
        {
            try
            {
                if (!tf_buffer.canTransform(target_frame, source_frame, input_msg->header.stamp, ros::Duration(1)))
                {
                    ROS_WARN("Cannot transform from %s to %s at time %u.%u", source_frame.c_str(), target_frame.c_str(), input_msg->header.stamp.sec, input_msg->header.stamp.nsec);
                    return;
                }
                tf_buffer.transform(*input_msg, cloud_in_base_link, target_frame, ros::Duration(1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("TF transform exception: %s", ex.what());
                return;
            }
            cloud_pub_.publish(cloud_in_base_link);
        }
    }

    // 一键过滤0
    void clear::once_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
        const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt,
        double cubeyaw)
    {
        // 创建 CropBox 滤波器对象
        pcl::CropBox<pcl::PointXYZ> crop_box_filter;
        crop_box_filter.setInputCloud(cloud_in);
        // 设置盒子范围
        crop_box_filter.setMin(min_pt);
        crop_box_filter.setMax(max_pt);
        // false 清除盒子内部的点
        crop_box_filter.setNegative(true);
        // 设置过滤盒子旋转
        crop_box_filter.setRotation(Eigen::Vector3f(0, 0, cubeyaw));
        // 执行过滤
        crop_box_filter.filter(*cloud_in);
    }

    bool clear::is_non_area(double x, double y)
    {
        // 判断输入的x，y是否在给定的所有的区域内，若在，则返回true
        // 遍历所有polygons_判断(x,y)是否点在任何一个多边形内
        for (const auto &poly : polygons_)
        {
            int cnt = 0;
            size_t n = poly.size();
            for (size_t i = 0; i < n; i++)
            {
                const Point2D &p1 = poly[i];
                const Point2D &p2 = poly[(i + 1) % n];
                // 射线法判断点是否在多边形内
                if (((p1.y > y) != (p2.y > y)) &&
                    (x < (p2.x - p1.x) * (y - p1.y) / (p2.y - p1.y) + p1.x))
                {
                    cnt++;
                }
            }
            if (cnt % 2 == 1)
            {
                // 点在多边形内，返回true
                return true;
            }
        }
        // 点不在任何多边形内，返回false
        return false;
        // TODO 数据结构优化
        // 或者使用 PCL库的cube函数
    }
    //
    void clear::combineCloudCB(const sensor_msgs::PointCloud2ConstPtr &input_msg)
    {
        // TODO:结合里程计进行哪多边形进行判断
    }

    ///////////////////////////////////////////////////////////////////
    bool clear::loadparam(const std::string &param_name)
    {
        // 载入多边形yaml文件
        // param_name的参数格式是一个包含多个多边形的数组，每个多边形又是一个数组，内含多个{ x, y }
        XmlRpc::XmlRpcValue param_list;
        if (!nh_.getParam(param_name, param_list))
        {
            ROS_ERROR("Failed to get param %s", param_name.c_str());
            return false;
        }
        if (param_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param %s is not an array", param_name.c_str());
            return false;
        }
        polygons_.clear();
        for (int i = 0; i < param_list.size(); ++i)
        {
            // 格式不对！
            if (param_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_WARN("Polygon %d is not an array of points", i);
                continue;
            }
            Polygon2D polygon;
            for (int j = 0; j < param_list[i].size(); ++j)
            {
                // ？？？
                if (param_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                {
                    ROS_WARN("Point %d in polygon %d is not a struct", j, i);
                    continue;
                }
                Point2D pt;
                if (param_list[i][j].hasMember("x") && param_list[i][j].hasMember("y"))
                {
                    pt.x = static_cast<double>(param_list[i][j]["x"]);
                    pt.y = static_cast<double>(param_list[i][j]["y"]);
                    polygon.push_back(pt);
                }
                else
                {
                    ROS_WARN("Point %d in polygon %d missing x or y", j, i);
                }
            }
            if (!polygon.empty())
            {
                polygons_.push_back(polygon);
            }
        }
        ROS_INFO("Loaded %lu polygons from param %s", polygons_.size(), param_name.c_str());
        return true;
    }

    bool clear::loadcubeParam(const std::string &param_name)
    {
        XmlRpc::XmlRpcValue cuboid_list;
        if (!nh_.getParam(param_name, cuboid_list))
        {
            ROS_ERROR("Failed to get param '%s'", param_name.c_str());
            return false;
        }
        if (cuboid_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            ROS_ERROR("Param '%s' is not an array", param_name.c_str());
            return false;
        }

        min_pts_.clear();
        max_pts_.clear();

        for (int i = 0; i < cuboid_list.size(); ++i)
        {
            if (!cuboid_list[i].hasMember("min") || !cuboid_list[i].hasMember("max"))
            {
                ROS_ERROR("cuboid element %d does not have min/max", i);
                continue;
            }

            XmlRpc::XmlRpcValue &min_val = cuboid_list[i]["min"];
            XmlRpc::XmlRpcValue &max_val = cuboid_list[i]["max"];

            if (min_val.getType() != XmlRpc::XmlRpcValue::TypeArray || max_val.getType() != XmlRpc::XmlRpcValue::TypeArray ||
                min_val.size() != 4 || max_val.size() != 4)
            {
                ROS_ERROR("cuboid element %d min/max not 4 elements", i);
                continue;
            }

            Eigen::Vector4f min_pt, max_pt;
            for (int j = 0; j < 4; ++j)
            {
                // min_val[j]
                if (min_val[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    min_pt[j] = static_cast<float>(static_cast<double>(min_val[j]));
                }
                else if (min_val[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    min_pt[j] = static_cast<float>(static_cast<int>(min_val[j]));
                }
                else
                {
                    ROS_ERROR("cuboid element %d min_val[%d] has unexpected type", i, j);
                    return false;
                }

                // max_val[j]
                if (max_val[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                {
                    max_pt[j] = static_cast<float>(static_cast<double>(max_val[j]));
                }
                else if (max_val[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    max_pt[j] = static_cast<float>(static_cast<int>(max_val[j]));
                }
                else
                {
                    ROS_ERROR("cuboid element %d max_val[%d] has unexpected type", i, j);
                    return false;
                }
            }
            min_pts_.push_back(min_pt);
            max_pts_.push_back(max_pt);
        }
        ROS_INFO("Loaded %lu cuboids from param '%s'", min_pts_.size(), param_name.c_str());
        return true;
    }
}
