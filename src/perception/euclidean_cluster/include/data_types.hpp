#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace EuclideanCluster
{

struct Detected_Obj
{
  int track_id = -1;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f size = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

  float vx = 0.0f;
  float vy = 0.0f;
  float speed = 0.0f;
};

}  // namespace EuclideanCluster
