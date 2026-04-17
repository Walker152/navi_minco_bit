#include "pcd2ele/pcd2ele.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <limits>

namespace pcd2ele {

PCD2ELE::PCD2ELE(const rclcpp::NodeOptions & options) : Node("pcd2ele_node", options)
{
  declareParameters();
  getParameters();
  process();
}

PCD2ELE::~PCD2ELE()
{
}

void PCD2ELE::declareParameters()
{
  declare_parameter("pcd_file_path", "");
  declare_parameter("output_folder", "./maps");
  declare_parameter("output_name", "elevation_map");
  declare_parameter("z_min", -100.0);
  declare_parameter("z_max", 100.0);
  declare_parameter("filter_radius", 0.5);
  declare_parameter("filter_min_neighbors", 10);
  declare_parameter("resolution", 0.05);
  declare_parameter("use_radius_filter", true);
  declare_parameter("use_pass_through_filter", true);
  declare_parameter("use_dilation", true);
  declare_parameter("dilation_radius", 1);
}

void PCD2ELE::getParameters()
{
  get_parameter("pcd_file_path", pcd_file_path_);
  get_parameter("output_folder", output_folder_);
  get_parameter("output_name", output_name_);
  get_parameter("z_min", z_min_);
  get_parameter("z_max", z_max_);
  get_parameter("filter_radius", filter_radius_);
  get_parameter("filter_min_neighbors", filter_min_neighbors_);
  get_parameter("resolution", resolution_);
  get_parameter("use_radius_filter", use_radius_filter_);
  get_parameter("use_pass_through_filter", use_pass_through_filter_);
  get_parameter("use_dilation", use_dilation_);
  get_parameter("dilation_radius", dilation_radius_);
}

void PCD2ELE::process()
{
  if (pcd_file_path_.empty()) {
    RCLCPP_ERROR(get_logger(), "PCD file path is empty!");
    return;
  }

  cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_path_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Loaded point cloud with %lu points", cloud_->points.size());

  // Display initial coordinate system info (Bounds)
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  RCLCPP_INFO(get_logger(),
    "Original Cloud Bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
    min_pt.x,
    max_pt.x,
    min_pt.y,
    max_pt.y,
    min_pt.z,
    max_pt.z);

  // PassThrough Filter
  if (use_pass_through_filter_) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min_, z_max_);
    pass.filter(*cloud_);
    RCLCPP_INFO(get_logger(), "After PassThrough: %lu points", cloud_->points.size());
  }

  // Radius Outlier Removal
  if (use_radius_filter_) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_);
    outrem.setRadiusSearch(filter_radius_);
    outrem.setMinNeighborsInRadius(filter_min_neighbors_);
    outrem.filter(*cloud_);
    RCLCPP_INFO(get_logger(), "After RadiusOutlier: %lu points", cloud_->points.size());
  }

  if (cloud_->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty after filtering!");
    return;
  }

  // Re-calculate bounds after filtering
  pcl::getMinMax3D(*cloud_, min_pt, max_pt);
  RCLCPP_INFO(get_logger(),
    "Filtered Cloud Bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
    min_pt.x,
    max_pt.x,
    min_pt.y,
    max_pt.y,
    min_pt.z,
    max_pt.z);

  // Create Elevation Grid
  int width = std::ceil((max_pt.x - min_pt.x) / resolution_);
  int height = std::ceil((max_pt.y - min_pt.y) / resolution_);

  // Use a vector to store max z in each cell. Initialize with lowest possible double.
  std::vector<double> elevation_grid(width * height, -std::numeric_limits<double>::infinity());

  for (const auto & pt : cloud_->points) {
    int x_idx = std::floor((pt.x - min_pt.x) / resolution_);
    int y_idx = std::floor((pt.y - min_pt.y) / resolution_);

    if (x_idx >= 0 && x_idx < width && y_idx >= 0 && y_idx < height) {
      int index = y_idx * width + x_idx;
      if (pt.z > elevation_grid[index]) {
        elevation_grid[index] = pt.z;
      }
    }
  }

  RCLCPP_INFO(get_logger(),
    "PGM Map Info: Origin[%.2f, %.2f], Resolution[%.2f], Size[%d, %d]",
    min_pt.x,
    min_pt.y,
    resolution_,
    width,
    height);

  if (use_dilation_) {
    dilateMap(elevation_grid, width, height);
  }

  saveMap(elevation_grid, width, height, min_pt.z, max_pt.z);
}

void PCD2ELE::dilateMap(std::vector<double> & elevation_grid, int width, int height)
{
  std::vector<double> dilated_grid = elevation_grid;
  int radius = dilation_radius_;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * width + x;
      double max_z = elevation_grid[idx];

      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            int nidx = ny * width + nx;
            double val = elevation_grid[nidx];
            if (val > max_z) {
              max_z = val;
            }
          }
        }
      }
      dilated_grid[idx] = max_z;
    }
  }
  elevation_grid = dilated_grid;
  RCLCPP_INFO(get_logger(), "Dilated map with radius %d", radius);
}

void PCD2ELE::saveMap(
  const std::vector<double> & elevation_grid, int width, int height, double min_z, double max_z)
{
  // Create output directory if it doesn't exist
  std::filesystem::path output_dir(output_folder_);
  if (!std::filesystem::exists(output_dir)) {
    std::filesystem::create_directories(output_dir);
  }

  std::string pgm_filename = output_name_ + ".pgm";
  std::string yaml_filename = output_name_ + ".yaml";
  std::filesystem::path pgm_path = output_dir / pgm_filename;
  std::filesystem::path yaml_path = output_dir / yaml_filename;

  // Normalize and write PGM
  std::ofstream pgm_file(pgm_path, std::ios::binary);
  if (!pgm_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open PGM file for writing: %s", pgm_path.c_str());
    return;
  }

  pgm_file << "P5\n" << width << " " << height << "\n255\n";

  // We need to flip Y because PGM stores top-to-bottom, but map coordinate usually implies bottom-to-top (y
  // increases upwards) Standard map_server loads PGM such that (0,0) is bottom-left if we don't flip?
  // Actually, map_server reads image: (0,0) is top-left.
  // But the map origin in YAML is the position of the bottom-left pixel.
  // So we should write the image such that the bottom row of the map corresponds to the last row of the
  // image (or first row if we flip). Let's write it so that index 0 (bottom-left in our grid logic)
  // corresponds to the bottom-left pixel in the image. PGM stores top row first. So we should write the
  // last row of our grid (max y) first.

  std::vector<unsigned char> pgm_data(width * height);

  double range = max_z - min_z;
  if (range <= 0)
    range = 1.0;  // Avoid division by zero

  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      int grid_idx = y * width + x;
      double z = elevation_grid[grid_idx];
      unsigned char val = 0;
      if (z > -std::numeric_limits<double>::infinity()) {
        // Normalize to 0-255
        // 0 is lowest, 255 is highest
        val = static_cast<unsigned char>(255.0 * (z - min_z) / range);
      } else {
        // Empty cell. What value?
        // If we want it to be "unknown", maybe 0? Or 128?
        // Let's use 0 for now.
        val = 0;
      }
      pgm_file << val;
    }
  }
  pgm_file.close();

  // Write YAML
  std::ofstream yaml_file(yaml_path);
  if (!yaml_file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open YAML file for writing: %s", yaml_path.c_str());
    return;
  }

  // Get origin from cloud bounds (min_x, min_y)
  // We need to pass min_x and min_y to this function or store them.
  // I'll re-calculate or pass them.
  // Wait, I didn't pass min_x, min_y to saveMap.
  // I'll fix this by adding arguments or using member variables.
  // I'll assume the caller passed the correct min_z/max_z, but I need min_x/min_y for the origin.
  // I'll modify the function signature in the next step or just fix it now.
  // I'll assume I can get them from the cloud if I didn't clear it, but I should pass them.

  // Let's fix the function signature in the .cpp file content I'm writing.
  // But I already defined it in .hpp without min_x, min_y.
  // I should update .hpp as well or just use the cloud_ member to get min_x/min_y again (it's fast).

  pcl::PointXYZ min_pt_c, max_pt_c;
  pcl::getMinMax3D(*cloud_, min_pt_c, max_pt_c);

  yaml_file << "image: " << pgm_filename << "\n";
  yaml_file << "resolution: " << resolution_ << "\n";
  yaml_file << "origin: [" << min_pt_c.x << ", " << min_pt_c.y << ", 0.0]\n";
  yaml_file << "negate: 0\n";
  yaml_file << "occupied_thresh: 0.65\n";
  yaml_file << "free_thresh: 0.196\n";
  yaml_file.close();

  RCLCPP_INFO(get_logger(), "Saved map to %s and %s", pgm_path.c_str(), yaml_path.c_str());
}

}  // namespace pcd2ele

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2ele::PCD2ELE)
