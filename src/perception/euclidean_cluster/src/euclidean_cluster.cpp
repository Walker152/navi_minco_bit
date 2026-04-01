#include "euclidean_cluster.hpp"

#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace EuclideanCluster
{

void EuclideanClusterAlg::configure(const ClusterConfig & config)
{
  config_ = config;
}

void EuclideanClusterAlg::processCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
  std::vector<Detected_Obj> & obj_list) const
{
  obj_list.clear();
  if (!in_cloud || in_cloud->empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(in_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(config_.z_min, config_.z_max);
  pass.filter(*z_filtered);

  if (z_filtered->empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(z_filtered);
  voxel.setLeafSize(config_.leaf_size, config_.leaf_size, config_.leaf_size);
  voxel.filter(*downsampled);

  if (downsampled->empty()) {
    return;
  }

  clusterByDistance(downsampled, obj_list);
}

void EuclideanClusterAlg::clusterByDistance(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
  std::vector<Detected_Obj> & obj_list) const
{
  if (!in_cloud || in_cloud->empty()) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  near_cloud->reserve(in_cloud->size());

  for (const auto & p : in_cloud->points) {
    const float dist = std::sqrt(p.x * p.x + p.y * p.y);
    if (dist <= config_.max_detection_range) {
      near_cloud->points.push_back(p);
    }
  }

  if (near_cloud->empty()) {
    return;
  }

  clusterSegment(near_cloud, config_.cluster_tolerance, obj_list);
}

void EuclideanClusterAlg::clusterSegment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud,
  float max_cluster_distance,
  std::vector<Detected_Obj> & obj_list) const
{
  if (!in_cloud || in_cloud->empty()) {
    return;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud, *cloud_2d);
  for (auto & p : cloud_2d->points) {
    p.z = 0.0f;
  }
  tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
  euclid.setInputCloud(cloud_2d);
  euclid.setSearchMethod(tree);
  euclid.setClusterTolerance(max_cluster_distance);
  euclid.setMinClusterSize(config_.min_cluster_size);
  euclid.setMaxClusterSize(config_.max_cluster_size);
  euclid.extract(indices);

  for (const auto & cluster_idx : indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    cluster->reserve(cluster_idx.indices.size());
    for (const int idx : cluster_idx.indices) {
      cluster->points.push_back(in_cloud->points[static_cast<size_t>(idx)]);
    }
    if (cluster->empty()) {
      continue;
    }

    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*cluster, centroid4f);
    const Eigen::Vector3f centroid = centroid4f.head<3>();

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, centroid4f, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance, Eigen::ComputeEigenvectors);
    if (solver.info() != Eigen::Success) {
      continue;
    }

    Eigen::Matrix3f basis;
    basis.col(0) = solver.eigenvectors().col(2).normalized();
    basis.col(1) = solver.eigenvectors().col(1).normalized();
    basis.col(2) = basis.col(0).cross(basis.col(1)).normalized();
    basis.col(1) = basis.col(2).cross(basis.col(0)).normalized();

    Eigen::Vector3f proj_min(
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max());
    Eigen::Vector3f proj_max(
      -std::numeric_limits<float>::max(),
      -std::numeric_limits<float>::max(),
      -std::numeric_limits<float>::max());

    for (const auto & p : cluster->points) {
      const Eigen::Vector3f pt(p.x, p.y, p.z);
      const Eigen::Vector3f local = basis.transpose() * (pt - centroid);
      proj_min = proj_min.cwiseMin(local);
      proj_max = proj_max.cwiseMax(local);
    }

    const Eigen::Vector3f local_center = 0.5f * (proj_min + proj_max);

    Detected_Obj obj;
    obj.centroid = centroid + basis * local_center;
    obj.size = (proj_max - proj_min).cwiseAbs().cwiseMax(Eigen::Vector3f(0.01f, 0.01f, 0.01f));
    obj.orientation = Eigen::Quaternionf(basis);
    obj.track_id = -1;
    obj.vx = 0.0f;
    obj.vy = 0.0f;
    obj.speed = 0.0f;

    obj_list.push_back(obj);
  }
}

}  // namespace EuclideanCluster
