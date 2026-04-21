#pragma once
#include <Eigen/Eigen>
#include <ctime>
#include <deque>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <set>
#include <unordered_map>

using namespace std;
using namespace pcl;

namespace pclfilter {
struct PointInfo
{
  vector<int> indices;
  float depth_;

  PointInfo() : depth_(-1) {}
  PointInfo(int index, float depth)
  {
    indices.push_back(index);
    depth_ = depth;
  }
};

struct FrameData
{
  vector<int> ground_indices;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  double timestamp;
};

class DepthCluster
{
public:
  DepthCluster(float vertcal_resolution,
    float horizontal_resolution,
    int lidar_lines,
    int min_cluster_size,
    float ground_max_slope_angle,
    int normal_k,
    float depth_threshold,
    bool use_euclidean,
    float euclidean_tolerance,
    int euclidean_min_size,
    int euclidean_max_size,
    float normal_curvature_threshold,
    bool auto_estimate_reference_normal,
    float sensor_tilt_angle_x,
    float sensor_tilt_angle_y,
    bool use_adaptive_radius,
    bool use_temporal_filter,
    int temporal_window_size);

  void initParams();
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & msg);

  vector<int> exactGroundCloudIndicesByLocalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr & msg,
    vector<vector<int>> & label_image,
    const vector<vector<PointInfo>> & depth_image);

  vector<pcl::PointIndices> euclideanClustering(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, const vector<int> & indices_to_cluster);

  vector<vector<PointInfo>> generateDepthImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused_ptr);
  void labelComponents(const vector<vector<PointInfo>> & depth_image,
    vector<vector<int>> & label_image,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_msg);
  bool judgmentCondition(const vector<vector<PointInfo>> & depth_image,
    const pair<int, int> & target_point,
    const pair<int, int> & neigh_point);
  bool calculateCoordinate(const pcl::PointXYZ & point, int & row, int & col);
  bool warpPoint(pair<int, int> & pt);

  vector<vector<int>> getClustersIndex();
  vector<int> getMergedClustersIndex();
  vector<int> getGroundCloudIndices();
  void paramsReset();

  void setMaxSlopeAngle(float angle_degrees) { max_slope_angle_rad_ = angle_degrees * M_PI / 180.0f; }
  void setNormalEstimationK(int k) { normal_estimation_k_ = k; }
  void setAutoEstimateReferenceNormal(bool enable) { auto_estimate_reference_normal_ = enable; }
  void setSensorTilt(float angle_x, float angle_y);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_Pointcloud_;
  vector<int> ground_points_indices_;
  float sensor_height_;
  float max_slope_angle_rad_;
  int normal_estimation_k_;
  float normal_curvature_threshold_;
  bool auto_estimate_reference_normal_;
  Eigen::Vector3f reference_normal_;

  float vertcal_resolution_;
  float horizontal_resolution_;
  int lidar_lines_;
  int min_cluster_size_;
  float depth_threshold_;
  bool use_euclidean_;
  float euclidean_tolerance_;
  int euclidean_min_cluster_size_;
  int euclidean_max_cluster_size_;

  bool use_adaptive_radius_;
  bool use_temporal_filter_;
  int temporal_window_size_;
  deque<FrameData> frame_history_;

  int image_rows_;
  int image_cols_;
  int vertical_angle_min_;
  int vertical_angle_max_;
  vector<vector<int>> clusters_indices_vec_;

  bool computeLocalNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    const std::vector<int> & indices,
    Eigen::Vector3f & normal,
    float & curvature);
  void estimateReferenceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  float computeAdaptiveTolerance(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
  vector<int> temporalFilterGround(const vector<int> & current_ground,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & current_cloud,
    double current_time);
};
}  // namespace pclfilter