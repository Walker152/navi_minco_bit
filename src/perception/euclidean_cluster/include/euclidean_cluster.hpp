#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "data_types.hpp"

namespace EuclideanCluster
{

struct ClusterConfig
{
  float z_min = 0.150;
  float z_max = 0.6f;
  float max_detection_range = 10.0f;
  float leaf_size = 0.05f;
  float cluster_tolerance = 0.2f;
  int min_cluster_size = 15;
  int max_cluster_size = 500;
};

class EuclideanClusterAlg
{
public:
  EuclideanClusterAlg() = default;

  void configure(const ClusterConfig & config);
  void processCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
    std::vector<Detected_Obj> & obj_list) const;

private:
  void clusterByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
    std::vector<Detected_Obj> & obj_list) const;

  void clusterSegment(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
    float max_cluster_distance,
    std::vector<Detected_Obj> & obj_list) const;

  ClusterConfig config_;
};

}  // namespace EuclideanCluster
