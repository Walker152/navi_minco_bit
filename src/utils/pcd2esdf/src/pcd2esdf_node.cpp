#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <vector>
#include <algorithm>

class Pcd2EsdfNode : public rclcpp::Node
{
public:
  Pcd2EsdfNode() : Node("pcd2esdf_node")
  {
    this->declare_parameter("pcd_path", "");
    this->declare_parameter("output_path", "");
    this->declare_parameter("resolution", 0.1);
    this->declare_parameter("downsample_resolution", 0.05);
    this->declare_parameter("padding", 1.0); // meters
    this->declare_parameter("visualize", true);

    std::string pcd_path = this->get_parameter("pcd_path").as_string();
    std::string output_path = this->get_parameter("output_path").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    double downsample_res = this->get_parameter("downsample_resolution").as_double();
    padding_ = this->get_parameter("padding").as_double();
    bool visualize = this->get_parameter("visualize").as_bool();

    if (pcd_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Please provide pcd_path parameter");
      return;
    }

    if (output_path.empty()) {
      size_t last_dot = pcd_path.find_last_of(".");
      if (last_dot != std::string::npos && pcd_path.substr(last_dot) == ".pcd") {
        output_path = pcd_path.substr(0, last_dot) + ".esdf";
      } else {
        output_path = pcd_path + ".esdf";
      }
      RCLCPP_WARN(this->get_logger(), "output_path not provided, using %s", output_path.c_str());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", pcd_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %lu points", cloud->points.size());

    if (downsample_res > 1e-4) {
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud);
      sor.setLeafSize(downsample_res, downsample_res, downsample_res);
      sor.filter(*cloud);
      RCLCPP_INFO(this->get_logger(), "Downsampled to %lu points", cloud->points.size());
    }

    generateESDF(cloud, output_path);

    if (visualize) {
      pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("esdf_cloud", 1);
      publishESDF();
    }
  }

private:
  void generateESDF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string& output_path)
  {
    // 1. Compute bounds
    double x_min = 1e9, y_min = 1e9, z_min = 1e9;
    double x_max = -1e9, y_max = -1e9, z_max = -1e9;

    for (const auto& pt : cloud->points) {
      if (pt.x < x_min) x_min = pt.x;
      if (pt.x > x_max) x_max = pt.x;
      if (pt.y < y_min) y_min = pt.y;
      if (pt.y > y_max) y_max = pt.y;
      if (pt.z < z_min) z_min = pt.z;
      if (pt.z > z_max) z_max = pt.z;
    }

    x_min -= padding_; y_min -= padding_; z_min -= padding_;
    x_max += padding_; y_max += padding_; z_max += padding_;

    int x_size = std::ceil((x_max - x_min) / resolution_);
    int y_size = std::ceil((y_max - y_min) / resolution_);
    int z_size = std::ceil((z_max - z_min) / resolution_);

    RCLCPP_INFO(this->get_logger(), "Map bounds: [%f, %f] x [%f, %f] x [%f, %f]", x_min, x_max, y_min, y_max, z_min, z_max);
    RCLCPP_INFO(this->get_logger(), "Grid size: %d x %d x %d", x_size, y_size, z_size);

    // 2. Build KD-Tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 3. Compute ESDF
    esdf_data_.resize(x_size * y_size * z_size);
    
    // Store metadata for visualization
    min_bound_ = Eigen::Vector3d(x_min, y_min, z_min);
    grid_size_ = Eigen::Vector3i(x_size, y_size, z_size);

    RCLCPP_INFO(this->get_logger(), "Computing ESDF... This might take a while.");
    
    #pragma omp parallel for collapse(3)
    for (int x = 0; x < x_size; ++x) {
      for (int y = 0; y < y_size; ++y) {
        for (int z = 0; z < z_size; ++z) {
          pcl::PointXYZ searchPoint;
          searchPoint.x = x_min + (x + 0.5) * resolution_;
          searchPoint.y = y_min + (y + 0.5) * resolution_;
          searchPoint.z = z_min + (z + 0.5) * resolution_;

          std::vector<int> pointIdxNKNSearch(1);
          std::vector<float> pointNKNSquaredDistance(1);

          if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            esdf_data_[x * y_size * z_size + y * z_size + z] = std::sqrt(pointNKNSquaredDistance[0]);
          } else {
            esdf_data_[x * y_size * z_size + y * z_size + z] = -1.0; // Should not happen
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "ESDF computed.");

    // 4. Save to file
    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open output file %s", output_path.c_str());
      return;
    }

    out.write(reinterpret_cast<const char*>(&resolution_), sizeof(double));
    out.write(reinterpret_cast<const char*>(&x_min), sizeof(double));
    out.write(reinterpret_cast<const char*>(&y_min), sizeof(double));
    out.write(reinterpret_cast<const char*>(&z_min), sizeof(double));
    out.write(reinterpret_cast<const char*>(&x_size), sizeof(int));
    out.write(reinterpret_cast<const char*>(&y_size), sizeof(int));
    out.write(reinterpret_cast<const char*>(&z_size), sizeof(int));
    out.write(reinterpret_cast<const char*>(esdf_data_.data()), esdf_data_.size() * sizeof(double));
    out.close();

    RCLCPP_INFO(this->get_logger(), "Saved ESDF to %s", output_path.c_str());
  }

  void publishESDF()
  {
    pcl::PointCloud<pcl::PointXYZI> cloud_out;
    
    // Downsample for visualization if needed, or just publish points with dist < threshold
    // Here we publish a slice or just points close to obstacles for verification
    
    for (int x = 0; x < grid_size_.x(); ++x) {
      for (int y = 0; y < grid_size_.y(); ++y) {
        for (int z = 0; z < grid_size_.z(); ++z) {
           double dist = esdf_data_[x * grid_size_.y() * grid_size_.z() + y * grid_size_.z() + z];
           // Visualize points with distance < 2.0m to see the field
           if (dist < 2.0) {
             pcl::PointXYZI pt;
             pt.x = min_bound_.x() + (x + 0.5) * resolution_;
             pt.y = min_bound_.y() + (y + 0.5) * resolution_;
             pt.z = min_bound_.z() + (z + 0.5) * resolution_;
             pt.intensity = dist;
             cloud_out.points.push_back(pt);
           }
        }
      }
    }

    cloud_out.width = cloud_out.points.size();
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    cloud_out.header.frame_id = "map";

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud_out, msg);
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published ESDF visualization cloud.");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<double> esdf_data_;
  double resolution_;
  double padding_;
  Eigen::Vector3d min_bound_;
  Eigen::Vector3i grid_size_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pcd2EsdfNode>());
  rclcpp::shutdown();
  return 0;
}
