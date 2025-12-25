#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <vector>
#include <string>
#include <cmath>
#include <filesystem>

/**
 * @brief 从 Nav2 PGM 地图生成 ESDF
 * 逻辑：
 * 1. 读取 YAML 获取地图元数据（分辨率、原点）。
 * 2. 读取 PGM 像素，识别占据格（像素值 < occupied_thresh）。
 * 3. 运行 2D Meijster 距离变换。
 * 4. 转换回世界坐标并存储在 PCD Intensity 字段。
 */

class Pgm2EsdfGenerator : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;

    // 地图参数
    std::string map_yaml_path_;
    std::string output_pcd_path_;
    double max_dist_;
    int occupied_thresh_;

    // YAML 中的数据
    std::string image_name_;
    double resolution_;
    std::vector<double> origin_; // [x, y, yaw]
    int width_, height_;

public:
    Pgm2EsdfGenerator() : Node("pgm2esdf_generator") {
        this->declare_parameter("map_yaml", "map.yaml");
        this->declare_parameter("output_pcd", "map_esdf.pcd");
        this->declare_parameter("max_dist", 3.0);
        this->declare_parameter("occupied_thresh", 10); // 接近 0 为障碍物

        map_yaml_path_ = this->get_parameter("map_yaml").as_string();
        output_pcd_path_ = this->get_parameter("output_pcd").as_string();
        max_dist_ = this->get_parameter("max_dist").as_double();
        occupied_thresh_ = this->get_parameter("occupied_thresh").as_int();

        esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("esdf_result", rclcpp::QoS(1).transient_local());

        process();
    }

    /**
     * @brief 1D 距离变换核心逻辑
     */
    void fillESDF1D(const std::vector<double>& src, std::vector<double>& dst, int n, int step, int offset) {
        int first_obs = -1;
        for (int i = 0; i < n; ++i) {
            if (src[i * step + offset] < 1e9) { first_obs = i; break; }
        }
        if (first_obs == -1) {
            for (int i = 0; i < n; ++i) dst[i * step + offset] = 1e10;
            return;
        }

        std::vector<int> v(n);
        std::vector<double> z(n + 1);
        int k = 0;
        v[0] = first_obs;
        z[0] = -1e10;
        z[1] = 1e10;

        for (int q = first_obs + 1; q < n; ++q) {
            if (src[q * step + offset] > 1e9) continue;
            double s;
            while (k >= 0) {
                int p = v[k];
                s = ((src[q * step + offset] + (double)q * q) - (src[p * step + offset] + (double)p * p)) / (2.0 * (double)q - 2.0 * (double)p);
                if (s <= z[k]) k--;
                else break;
            }
            k++;
            v[k] = q;
            z[k] = s;
            z[k + 1] = 1e10;
        }

        int cur_seg = 0;
        for (int q = 0; q < n; ++q) {
            while (z[cur_seg + 1] < (double)q) cur_seg++;
            dst[q * step + offset] = (double)(q - v[cur_seg]) * (q - v[cur_seg]) + src[v[cur_seg] * step + offset];
        }
    }

    void process() {
        // 1. 解析 YAML
        try {
            YAML::Node config = YAML::LoadFile(map_yaml_path_);
            image_name_ = config["image"].as<std::string>();
            resolution_ = config["resolution"].as<double>();
            origin_ = config["origin"].as<std::vector<double>>();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML Load Error: %s", e.what());
            return;
        }

        // 获取 PGM 的完整路径（与 YAML 在同一目录下）
        std::filesystem::path yaml_p(map_yaml_path_);
        std::string image_path = (yaml_p.parent_path() / image_name_).string();

        // 2. 读取 PGM
        cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not load image: %s", image_path.c_str());
            return;
        }
        width_ = img.cols;
        height_ = img.rows;
        RCLCPP_INFO(this->get_logger(), "Map Loaded: %dx%d, Res: %.3f", width_, height_, resolution_);

        // 3. 初始化距离场缓冲区
        std::vector<double> dist_buffer(width_ * height_, 1e10);
        for (int r = 0; r < height_; ++r) {
            for (int c = 0; c < width_; ++c) {
                // Nav2 PGM: 0 是障碍物，254 是空地，255 是未知
                // 我们取像素值小于阈值的作为障碍物
                if (img.at<uchar>(r, c) <= occupied_thresh_) {
                    dist_buffer[r * width_ + c] = 0;
                }
            }
        }

        // 4. Meijster 算法计算 ESDF
        std::vector<double> tmp_buffer(width_ * height_, 1e10);
        // 第一遍：行扫描 (Column-wise in logic, but Row in data)
        for (int r = 0; r < height_; ++r) {
            fillESDF1D(dist_buffer, tmp_buffer, width_, 1, r * width_);
        }
        // 第二遍：列扫描
        for (int c = 0; c < width_; ++c) {
            fillESDF1D(tmp_buffer, dist_buffer, height_, width_, c);
        }

        // 5. 转换为世界坐标系点云
        pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        double max_px_dist_sq = std::pow(max_dist_ / resolution_, 2);

        for (int r = 0; r < height_; ++r) {
            for (int c = 0; c < width_; ++c) {
                double d2 = dist_buffer[r * width_ + c];
                if (d2 > max_px_dist_sq) continue;

                pcl::PointXYZI pt;
                // 注意：PGM 的 (0,0) 是左上角，行代表 Y 轴减方向。
                // Nav2 的逻辑通常是：World_X = Origin_X + col * Res
                // World_Y = Origin_Y + (height - row - 1) * Res (如果原点在左下角)
                // 具体的转换取决于 YAML 的定义，通常 Nav2 是 col -> X, (height-row) -> Y
                pt.x = origin_[0] + (double)c * resolution_;
                pt.y = origin_[1] + (double)(height_ - r - 1) * resolution_;
                pt.z = 0.0;
                pt.intensity = std::sqrt(d2) * resolution_; // 物理距离 (米)
                out_cloud->push_back(pt);
            }
        }

        if (out_cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "ESDF result is empty!");
            return;
        }

        // 6. 保存与发布
        out_cloud->width = out_cloud->size();
        out_cloud->height = 1;
        out_cloud->is_dense = true;
        pcl::io::savePCDFileBinary(output_pcd_path_, *out_cloud);
        RCLCPP_INFO(this->get_logger(), "ESDF saved to %s (%lu points)", output_pcd_path_.c_str(), out_cloud->size());

        sensor_msgs::msg::PointCloud2 ros_msg;
        pcl::toROSMsg(*out_cloud, ros_msg);
        ros_msg.header.frame_id = "map";
        ros_msg.header.stamp = this->now();
        esdf_pub_->publish(ros_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pgm2EsdfGenerator>());
    rclcpp::shutdown();
    return 0;
}