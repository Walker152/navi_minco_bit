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

    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    Eigen::Vector2f mean_xy = Eigen::Vector2f::Zero();
    for (const auto & p : cluster->points) {
      mean_xy.x() += p.x;
      mean_xy.y() += p.y;
      min_z = std::min(min_z, p.z);
      max_z = std::max(max_z, p.z);
    }
    mean_xy /= static_cast<float>(cluster->size());

    Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
    for (const auto & p : cluster->points) {
      const Eigen::Vector2f d(p.x - mean_xy.x(), p.y - mean_xy.y());
      covariance += d * d.transpose();
    }
    covariance /= static_cast<float>(cluster->size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig_solver(covariance, Eigen::ComputeEigenvectors);
    if (eig_solver.info() != Eigen::Success) {
      continue;
    }

    Eigen::Vector2f primary = eig_solver.eigenvectors().col(1).normalized();
    Eigen::Vector2f secondary(-primary.y(), primary.x());

    float min_p = std::numeric_limits<float>::max();
    float max_p = -std::numeric_limits<float>::max();
    float min_s = std::numeric_limits<float>::max();
    float max_s = -std::numeric_limits<float>::max();
    for (const auto & p : cluster->points) {
      const Eigen::Vector2f d(p.x - mean_xy.x(), p.y - mean_xy.y());
      const float proj_p = d.dot(primary);
      const float proj_s = d.dot(secondary);
      min_p = std::min(min_p, proj_p);
      max_p = std::max(max_p, proj_p);
      min_s = std::min(min_s, proj_s);
      max_s = std::max(max_s, proj_s);
    }

    float length = max_p - min_p;
    float width = max_s - min_s;
    if (width > length) {
      std::swap(length, width);
      std::swap(primary, secondary);
      if (primary.x() * secondary.y() - primary.y() * secondary.x() < 0.0f) {
        secondary = -secondary;
      }
      // Keep projection center consistent after axis swap.
      std::swap(min_p, min_s);
      std::swap(max_p, max_s);
    }

    const float c_p = 0.5f * (min_p + max_p);
    const float c_s = 0.5f * (min_s + max_s);
    const Eigen::Vector2f center_xy = mean_xy + primary * c_p + secondary * c_s;

    Eigen::Matrix3f basis = Eigen::Matrix3f::Identity();
    basis.col(0) = Eigen::Vector3f(primary.x(), primary.y(), 0.0f);
    basis.col(1) = Eigen::Vector3f(secondary.x(), secondary.y(), 0.0f);
    basis.col(2) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    const float height = std::max(0.01f, max_z - min_z);

    Detected_Obj obj;
    obj.centroid = Eigen::Vector3f(center_xy.x(), center_xy.y(), 0.5f * (min_z + max_z));
    obj.size = Eigen::Vector3f(std::max(0.01f, length), std::max(0.01f, width), height);
    obj.orientation = Eigen::Quaternionf(basis);
    obj.track_id = -1;
    obj.vx = 0.0f;
    obj.vy = 0.0f;
    obj.speed = 0.0f;
    obj.dynamic_confirmed = false;

    obj_list.push_back(obj);
  }
}

}  // namespace EuclideanCluster
