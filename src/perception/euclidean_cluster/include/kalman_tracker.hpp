#pragma once

#include <vector>

#include <Eigen/Core>

#include "data_types.hpp"

namespace EuclideanCluster
{

struct TrackerConfig
{
  float match_distance_threshold = 0.5f;
  int max_missed_frames = 3;
  float dynamic_speed_threshold = 0.2f;
};

class KalmanTracker
{
public:
  KalmanTracker() = default;

  void configure(const TrackerConfig & config);
  void update(std::vector<Detected_Obj> & current_objects, float dt);

private:
  struct TrackedObject
  {
    int id = -1;
    Eigen::Vector2f position = Eigen::Vector2f::Zero();
    Eigen::Matrix<float, 4, 1> state = Eigen::Matrix<float, 4, 1>::Zero();
    Eigen::Matrix4f covariance = Eigen::Matrix4f::Identity();
    int missed_frames = 0;
  };

  TrackerConfig config_;
  std::vector<TrackedObject> tracked_objects_;
  int next_track_id_ = 0;
};

}  // namespace EuclideanCluster
