#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_set>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @brief 从 Nav2 PGM 地图生成 ESDF
 * 逻辑：
 * 1. 读取 YAML 获取地图元数据（分辨率、原点）。
 * 2. 读取 PGM 像素，识别占据格（像素值 < occupied_thresh）。
 * 3. 运行 2D Meijster 距离变换。
 * 4. 转换回世界坐标并存储在 PCD Intensity 字段。
 */

class Pgm2EsdfGenerator : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;

  // 地图参数
  std::string map_yaml_path_;
  std::string output_pcd_path_;
  double max_dist_;
  double occupied_thresh_;
  double free_thresh_;
  bool treat_unknown_as_obstacle_;
  std::unordered_set<int> unknown_gray_values_;

  // YAML 中的数据
  std::string image_name_;
  double resolution_;
  std::vector<double> origin_;  // [x, y, yaw]
  int negate_;
  int width_, height_;

public:
  Pgm2EsdfGenerator() : Node("pgm2esdf_generator")
  {
    this->declare_parameter("map_yaml", "map.yaml");
    this->declare_parameter("output_pcd", "map_esdf.pcd");
    this->declare_parameter("max_dist", 3.0);
    // Nav2 map_server 语义：占据/空闲阈值为概率(0~1)，并受 negate 影响。
    this->declare_parameter("occupied_thresh", 0.65);
    this->declare_parameter("free_thresh", 0.25);
    this->declare_parameter("treat_unknown_as_obstacle", true);
    this->declare_parameter("unknown_gray_values", std::vector<int64_t>{205});

    map_yaml_path_ = this->get_parameter("map_yaml").as_string();
    output_pcd_path_ = this->get_parameter("output_pcd").as_string();
    max_dist_ = this->get_parameter("max_dist").as_double();
    occupied_thresh_ = this->get_parameter("occupied_thresh").as_double();
    free_thresh_ = this->get_parameter("free_thresh").as_double();
    treat_unknown_as_obstacle_ = this->get_parameter("treat_unknown_as_obstacle").as_bool();
    const auto unknown_vals = this->get_parameter("unknown_gray_values").as_integer_array();
    for (const auto v : unknown_vals) {
      if (v >= 0 && v <= 255) {
        unknown_gray_values_.insert(static_cast<int>(v));
      }
    }
    if (unknown_gray_values_.empty()) {
      unknown_gray_values_.insert(205);
      RCLCPP_WARN(this->get_logger(), "unknown_gray_values is empty, fallback to [205]");
    }

    esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "esdf_result", rclcpp::QoS(1).transient_local());

    process();
  }

  /**
   * @brief 1D 距离变换核心逻辑
   */
  void fillESDF1D(const std::vector<double> & src, std::vector<double> & dst, int n, int step, int offset)
  {
    int first_obs = -1;
    for (int i = 0; i < n; ++i) {
      if (src[i * step + offset] < 1e9) {
        first_obs = i;
        break;
      }
    }
    if (first_obs == -1) {
      for (int i = 0; i < n; ++i)
        dst[i * step + offset] = 1e10;
      return;
    }

    std::vector<int> v(n);
    std::vector<double> z(n + 1);
    int k = 0;
    v[0] = first_obs;
    z[0] = -1e10;
    z[1] = 1e10;

    for (int q = first_obs + 1; q < n; ++q) {
      if (src[q * step + offset] > 1e9)
        continue;
      double s;
      while (k >= 0) {
        int p = v[k];
        s = ((src[q * step + offset] + (double)q * q) - (src[p * step + offset] + (double)p * p)) /
            (2.0 * (double)q - 2.0 * (double)p);
        if (s <= z[k])
          k--;
        else
          break;
      }
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = 1e10;
    }

    int cur_seg = 0;
    for (int q = 0; q < n; ++q) {
      while (z[cur_seg + 1] < (double)q)
        cur_seg++;
      dst[q * step + offset] =
        (double)(q - v[cur_seg]) * (q - v[cur_seg]) + src[v[cur_seg] * step + offset];
    }
  }

  void process()
  {
    // 1. 解析 YAML
    try {
      YAML::Node config = YAML::LoadFile(map_yaml_path_);
      image_name_ = config["image"].as<std::string>();
      resolution_ = config["resolution"].as<double>();
      origin_ = config["origin"].as<std::vector<double>>();
      negate_ = config["negate"] ? config["negate"].as<int>() : 0;
    } catch (const std::exception & e) {
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
    RCLCPP_INFO(this->get_logger(),
      "Map Loaded: %dx%d, Res: %.3f, Negate: %d",
      width_,
      height_,
      resolution_,
      negate_);

    if (occupied_thresh_ < 0.0 || occupied_thresh_ > 1.0 || free_thresh_ < 0.0 || free_thresh_ > 1.0 ||
        free_thresh_ >= occupied_thresh_) {
      RCLCPP_ERROR(this->get_logger(), "Invalid thresholds: require 0<=free_thresh<occupied_thresh<=1.");
      return;
    }

    // 3. 初始化双向距离场缓冲区
    // dist_buffer_out: 计算外部距离（空闲区到障碍物的距离）
    std::vector<double> dist_buffer_out(width_ * height_, 1e10);
    // dist_buffer_in: 计算内部距离（障碍物到空闲区的距离）
    std::vector<double> dist_buffer_in(width_ * height_, 1e10);
    std::vector<uint8_t> occupied_mask(width_ * height_, 0);

    size_t occupied_cnt = 0;
    size_t unknown_cnt = 0;
    size_t free_cnt = 0;

    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        const auto pix = img.at<uchar>(r, c);
        const double occ_prob = (negate_ == 0) ? (255.0 - (double)pix) / 255.0 : (double)pix / 255.0;

        const bool is_unknown_pixel =
          (unknown_gray_values_.find(static_cast<int>(pix)) != unknown_gray_values_.end());

        bool is_occupied = (occ_prob >= occupied_thresh_);
        bool is_unknown = is_unknown_pixel;
        if (!is_unknown_pixel) {
          is_unknown = (!is_occupied && occ_prob > free_thresh_);
        }

        if (!is_occupied && treat_unknown_as_obstacle_ && is_unknown) {
          is_occupied = true;
        }

        if (is_occupied) {
          // 占据栅格：对于外部距离场是零点，对于内部距离场是待求区域
          dist_buffer_out[r * width_ + c] = 0;
          dist_buffer_in[r * width_ + c] = 1e10;
          occupied_mask[r * width_ + c] = 1;
          occupied_cnt++;
        } else {
          // 空闲栅格：对于外部距离场是待求区域，对于内部距离场是零点
          dist_buffer_out[r * width_ + c] = 1e10;
          dist_buffer_in[r * width_ + c] = 0;
          if (is_unknown) {
            unknown_cnt++;
          } else {
            free_cnt++;
          }
        }
      }
    }

    // 4. Meijster 算法计算双向 ESDF
    std::vector<double> tmp_buffer(width_ * height_, 1e10);

    // --- 4.1 计算外部距离 (Outward ESDF) ---
    for (int r = 0; r < height_; ++r) {
      fillESDF1D(dist_buffer_out, tmp_buffer, width_, 1, r * width_);
    }
    for (int c = 0; c < width_; ++c) {
      fillESDF1D(tmp_buffer, dist_buffer_out, height_, width_, c);
    }

    // --- 4.2 计算内部距离 (Inward ESDF) ---
    std::fill(tmp_buffer.begin(), tmp_buffer.end(), 1e10);
    for (int r = 0; r < height_; ++r) {
      fillESDF1D(dist_buffer_in, tmp_buffer, width_, 1, r * width_);
    }
    for (int c = 0; c < width_; ++c) {
      fillESDF1D(tmp_buffer, dist_buffer_in, height_, width_, c);
    }

    // 5. 转换为世界坐标系点云 (融合为真正的 SEDF)
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    double max_px_dist_sq = std::pow(max_dist_ / resolution_, 2);

    double min_intensity = std::numeric_limits<double>::infinity();
    double max_intensity = 0.0;

    for (int r = 0; r < height_; ++r) {
      for (int c = 0; c < width_; ++c) {
        const int idx = r * width_ + c;

        pcl::PointXYZI pt;
        pt.x = origin_[0] + (double)c * resolution_;
        pt.y = origin_[1] + (double)(height_ - r - 1) * resolution_;
        pt.z = 0.0;

        if (occupied_mask[idx]) {
          // 障碍物内部：读取内部距离场，取负值
          double d2 = dist_buffer_in[idx];
          if (d2 > max_px_dist_sq)
            d2 = max_px_dist_sq;  // 限制内部最大穿透距离
          pt.intensity = -static_cast<float>(std::sqrt(d2) * resolution_);
        } else {
          // 空闲区域：读取外部距离场，取正值
          double d2 = dist_buffer_out[idx];
          if (d2 > max_px_dist_sq)
            d2 = max_px_dist_sq;
          pt.intensity = static_cast<float>(std::sqrt(d2) * resolution_);
        }

        min_intensity = std::min(min_intensity, (double)pt.intensity);
        max_intensity = std::max(max_intensity, (double)pt.intensity);

        out_cloud->push_back(pt);
      }
    }

    if (out_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "ESDF result is empty!");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "SEDF intensity range: [%.3f, %.3f] (meters).", min_intensity, max_intensity);

    // 6. 保存与发布
    out_cloud->width = out_cloud->size();
    out_cloud->height = 1;
    out_cloud->is_dense = true;
    pcl::io::savePCDFileBinary(output_pcd_path_, *out_cloud);
    RCLCPP_INFO(
      this->get_logger(), "SEDF saved to %s (%lu points)", output_pcd_path_.c_str(), out_cloud->size());

    sensor_msgs::msg::PointCloud2 ros_msg;
    pcl::toROSMsg(*out_cloud, ros_msg);
    ros_msg.header.frame_id = "map";
    ros_msg.header.stamp = this->now();
    esdf_pub_->publish(ros_msg);
  }
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pgm2EsdfGenerator>());
  rclcpp::shutdown();
  return 0;
}