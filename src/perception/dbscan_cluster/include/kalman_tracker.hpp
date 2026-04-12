#pragma once

#include <vector>

#include <Eigen/Core>

#include "data_types.hpp"

namespace DBSCANCluster
{

struct TrackerConfig
{
  float match_distance_threshold = 0.5f;
  int max_missed_frames = 3;
  float dynamic_speed_threshold = 0.2f;
  float alpha_size = 0.2f;
  float alpha_orientation = 0.2f;
  int class_confirm_frames = 3;
  float dt_default = 0.1f;
  float q_pos_x = 0.01f;
  float q_pos_y = 0.01f;
  float q_vel_x = 0.25f;
  float q_vel_y = 0.25f;
  float q_acc_x = 0.5f;
  float q_acc_y = 0.5f;
  float r_pos_x = 0.04f;
  float r_pos_y = 0.04f;
  float r_vel_x = 0.05f;
  float r_vel_y = 0.05f;
  float association_spatial_weight = 0.7f;
  float association_shape_weight = 0.3f;
  float association_gate_scale = 1.5f;
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
    Eigen::Matrix<float, 6, 1> state = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 6> covariance = Eigen::Matrix<float, 6, 6>::Identity();
    int missed_frames = 0;
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    int dynamic_match_frames = 0;
    bool dynamic_confirmed = false;
  };

  TrackerConfig config_;
  std::vector<TrackedObject> tracked_objects_;
  int next_track_id_ = 0;
};

}  // namespace DBSCANCluster
