#ifndef STATIC_ESDF_MAP_HPP
#define STATIC_ESDF_MAP_HPP

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

#include "utils/header/color_text.hpp"

namespace minco_planner {
using namespace color_text;
class StaticESDFMap
{
public:
  using Ptr = std::shared_ptr<StaticESDFMap>;

  StaticESDFMap() = default;

  /**
   * @brief 加载 PCD 并重建栅格地图
   * @param pcd_path PCD文件路径
   * @param resolution 地图分辨率 (必须与生成 PCD 时的一致，通常从 yaml 读取)
   */
  bool loadMap(const std::string & pcd_path, double resolution)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1) {
      std::cout << YELLOW << "[StaticESDFMap] "
                << "Failed to load PCD file: " << pcd_path << RESET << std::endl;
      return false;
    }

    if (cloud->empty()) {
      std::cout << YELLOW << "[StaticESDFMap] "
                << "Loaded PCD file is empty: " << pcd_path << RESET << std::endl;
      return false;
    }

    resolution_ = resolution;

    // 1. 自动计算地图边界
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();

    for (const auto & pt : cloud->points) {
      if (pt.x < min_x)
        min_x = pt.x;
      if (pt.x > max_x)
        max_x = pt.x;
      if (pt.y < min_y)
        min_y = pt.y;
      if (pt.y > max_y)
        max_y = pt.y;
    }

    // 添加一点缓冲，防止边界溢出
    origin_ << min_x, min_y;

    // 计算栅格尺寸 (+1 保证覆盖最大边界)
    width_ = std::ceil((max_x - min_x) / resolution_) + 1;
    height_ = std::ceil((max_y - min_y) / resolution_) + 1;

    // 2. 初始化地图数据（默认填充一个较大的安全距离，比如 10m）
    // 这样如果有点缺失，默认是无障碍的
    data_.assign(width_ * height_, 10.0);

    // 3. 填充数据
    for (const auto & pt : cloud->points) {
      int ix = std::round((pt.x - min_x) / resolution_);
      int iy = std::round((pt.y - min_y) / resolution_);

      if (ix >= 0 && ix < width_ && iy >= 0 && iy < height_) {
        // intensity 字段即为 ESDF 距离
        data_[iy * width_ + ix] = pt.intensity;
      }
    }

    std::cout << GREEN << "[StaticESDFMap] "
              << "Static ESDF loaded successfully. Size: " << width_ << " x " << height_
              << ", Resolution: " << resolution_ << ", Origin: (" << origin_.x() << ", " << origin_.y()
              << ")" << RESET << std::endl;

    return true;
  }

  /**
   * @brief 查询距离和梯度 (双线性插值)
   * @param pos 世界坐标 (x, y)
   * @param dist [输出] 距离障碍物的距离
   * @param grad [输出] 距离场的梯度 (方向指向远离障碍物的方向)
   */
  void evaluate(const Eigen::Vector3d & pos, double & dist, Eigen::Vector3d & grad) const
  {
    // 1. 转到栅格坐标
    double px = (pos.x() - origin_.x()) / resolution_;
    double py = (pos.y() - origin_.y()) / resolution_;

    int ix = std::floor(px);
    int iy = std::floor(py);

    // 2. 边界检查 (越界视为极度安全，梯度为0)
    if (ix < 0 || ix >= width_ - 1 || iy < 0 || iy >= height_ - 1) {
      dist = 10.0;
      grad.setZero();
      return;
    }

    // 3. 计算插值系数
    double fx = px - ix;
    double fy = py - iy;

    // 4. 获取四个邻居的值
    // indices: (ix, iy), (ix+1, iy), (ix, iy+1), (ix+1, iy+1)
    double d00 = data_[iy * width_ + ix];
    double d10 = data_[iy * width_ + (ix + 1)];
    double d01 = data_[(iy + 1) * width_ + ix];
    double d11 = data_[(iy + 1) * width_ + (ix + 1)];

    // 5. 双线性插值计算距离
    double lerp_y0 = (1.0 - fx) * d00 + fx * d10;
    double lerp_y1 = (1.0 - fx) * d01 + fx * d11;
    dist = (1.0 - fy) * lerp_y0 + fy * lerp_y1;

    // 6. 计算梯度 (有限差分法)
    // d_dist / d_px (像素梯度)
    double dd_dx = (1.0 - fy) * (d10 - d00) + fy * (d11 - d01);
    // d_dist / d_py (像素梯度)
    double dd_dy = (1.0 - fx) * (d01 - d00) + fx * (d11 - d10);

    // 像素梯度 -> 物理梯度: grad_world = grad_pixel / resolution
    grad.x() = dd_dx / resolution_;
    grad.y() = dd_dy / resolution_;
    grad.z() = 0.0;  // 2D ESDF, Z轴梯度为0
  }

  double getDistance(const Eigen::Vector3d & pos) const
  {
    double dist;
    Eigen::Vector3d grad;
    evaluate(pos, dist, grad);
    return dist;
  }

  int getWidth() const { return width_; }
  int getHeight() const { return height_; }
  double getResolution() const { return resolution_; }
  Eigen::Vector2d getOrigin() const { return origin_; }
  const std::vector<double> & getData() const { return data_; }

private:
  std::vector<double> data_;  // 一维数组存储栅格数据
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;      // 默认值，会被 loadMap 覆盖
  Eigen::Vector2d origin_{0, 0};  // 地图左下角物理坐标
};
}  // namespace minco_planner
#endif  // STATIC_ESDF_MAP_HPP