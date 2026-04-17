#ifndef PCD2ELE_HPP
#define PCD2ELE_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

namespace pcd2ele {
class PCD2ELE : public rclcpp::Node
{
public:
  explicit PCD2ELE(const rclcpp::NodeOptions & options);
  ~PCD2ELE();

private:
  void declareParameters();
  void getParameters();
  void process();
  void dilateMap(std::vector<double> & elevation_grid, int width, int height);
  void saveMap(
    const std::vector<double> & elevation_grid, int width, int height, double min_z, double max_z);

  // Parameters
  std::string pcd_file_path_;
  std::string output_folder_;
  std::string output_name_;
  double z_min_, z_max_;
  double filter_radius_;
  int filter_min_neighbors_;
  double resolution_;
  bool use_radius_filter_;
  bool use_pass_through_filter_;
  bool use_dilation_;
  int dilation_radius_;

  // Data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};
}  // namespace pcd2ele

#endif  // PCD2ELE_HPP
