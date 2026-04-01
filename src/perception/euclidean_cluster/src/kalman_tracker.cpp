#include "kalman_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace EuclideanCluster
{

void KalmanTracker::configure(const TrackerConfig & config)
{
  config_ = config;
}

void KalmanTracker::update(std::vector<Detected_Obj> & current_objects, float dt)
{
  const float dt_default = std::max(1e-3f, config_.dt_default);
  const float q_pos_x = std::max(1e-6f, config_.q_pos_x);
  const float q_pos_y = std::max(1e-6f, config_.q_pos_y);
  const float q_vel_x = std::max(1e-6f, config_.q_vel_x);
  const float q_vel_y = std::max(1e-6f, config_.q_vel_y);
  const float r_pos_x = std::max(1e-6f, config_.r_pos_x);
  const float r_pos_y = std::max(1e-6f, config_.r_pos_y);

  if (!(std::isfinite(dt) && dt > 1e-3f)) {
    dt = dt_default;
  }
  const int confirm_frames = std::max(1, config_.class_confirm_frames);

  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;

  Eigen::Matrix<float, 2, 4> H;
  H.setZero();
  H(0, 0) = 1.0f;
  H(1, 1) = 1.0f;

  Eigen::Matrix4f Q = Eigen::Matrix4f::Identity();
  Q(0, 0) = q_pos_x;
  Q(1, 1) = q_pos_y;
  Q(2, 2) = q_vel_x;
  Q(3, 3) = q_vel_y;

  Eigen::Matrix2f R = Eigen::Matrix2f::Zero();
  R(0, 0) = r_pos_x;
  R(1, 1) = r_pos_y;

  const size_t prev_track_count = tracked_objects_.size();
  for (size_t i = 0; i < prev_track_count; ++i) {
    tracked_objects_[i].state = F * tracked_objects_[i].state;
    tracked_objects_[i].covariance = F * tracked_objects_[i].covariance * F.transpose() + Q;
    tracked_objects_[i].position = tracked_objects_[i].state.head<2>();
  }

  std::vector<std::tuple<float, int, int>> candidates;
  candidates.reserve(current_objects.size() * std::max<size_t>(prev_track_count, 1));
  for (size_t obj_idx = 0; obj_idx < current_objects.size(); ++obj_idx) {
    const Eigen::Vector2f z(current_objects[obj_idx].centroid.x(), current_objects[obj_idx].centroid.y());
    for (size_t trk_idx = 0; trk_idx < prev_track_count; ++trk_idx) {
      const float dist = (z - tracked_objects_[trk_idx].position).norm();
      if (dist <= config_.match_distance_threshold) {
        candidates.emplace_back(dist, static_cast<int>(obj_idx), static_cast<int>(trk_idx));
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
    [](const auto & a, const auto & b) { return std::get<0>(a) < std::get<0>(b); });

  std::vector<bool> object_assigned(current_objects.size(), false);
  std::vector<bool> track_assigned(prev_track_count, false);

  for (const auto & c : candidates) {
    const int obj_idx = std::get<1>(c);
    const int trk_idx = std::get<2>(c);
    if (object_assigned[static_cast<size_t>(obj_idx)] || track_assigned[static_cast<size_t>(trk_idx)]) {
      continue;
    }

    auto & track = tracked_objects_[static_cast<size_t>(trk_idx)];
    const Eigen::Vector2f z(
      current_objects[static_cast<size_t>(obj_idx)].centroid.x(),
      current_objects[static_cast<size_t>(obj_idx)].centroid.y());

    const Eigen::Vector2f innovation = z - H * track.state;
    const Eigen::Matrix2f S = H * track.covariance * H.transpose() + R;
    const Eigen::Matrix<float, 4, 2> K = track.covariance * H.transpose() * S.inverse();

    track.state = track.state + K * innovation;
    track.covariance = (Eigen::Matrix4f::Identity() - K * H) * track.covariance;
    track.position = track.state.head<2>();
    track.missed_frames = 0;

    auto & obj = current_objects[static_cast<size_t>(obj_idx)];
    obj.track_id = track.id;
    obj.vx = track.state(2);
    obj.vy = track.state(3);
    obj.speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);

    if (obj.speed > config_.dynamic_speed_threshold) {
      track.dynamic_match_frames++;
    } else {
      track.dynamic_match_frames = 0;
      track.dynamic_confirmed = false;
    }

    if (track.dynamic_match_frames > confirm_frames) {
      track.dynamic_confirmed = true;
    }
    obj.dynamic_confirmed = track.dynamic_confirmed;

    object_assigned[static_cast<size_t>(obj_idx)] = true;
    track_assigned[static_cast<size_t>(trk_idx)] = true;
  }

  for (size_t obj_idx = 0; obj_idx < current_objects.size(); ++obj_idx) {
    if (object_assigned[obj_idx]) {
      continue;
    }

    TrackedObject new_track;
    new_track.id = next_track_id_++;
    new_track.state << current_objects[obj_idx].centroid.x(), current_objects[obj_idx].centroid.y(), 0.0f, 0.0f;
    new_track.position = new_track.state.head<2>();
    new_track.covariance = Eigen::Matrix4f::Identity();
    new_track.covariance(2, 2) = 4.0f;
    new_track.covariance(3, 3) = 4.0f;
    new_track.missed_frames = 0;
    new_track.dynamic_match_frames = 0;
    new_track.dynamic_confirmed = false;
    tracked_objects_.push_back(new_track);

    current_objects[obj_idx].track_id = new_track.id;
    current_objects[obj_idx].vx = 0.0f;
    current_objects[obj_idx].vy = 0.0f;
    current_objects[obj_idx].speed = 0.0f;
    current_objects[obj_idx].dynamic_confirmed = false;
  }

  for (size_t trk_idx = 0; trk_idx < prev_track_count; ++trk_idx) {
    if (!track_assigned[trk_idx]) {
      tracked_objects_[trk_idx].missed_frames++;
    }
  }

  tracked_objects_.erase(
    std::remove_if(
      tracked_objects_.begin(), tracked_objects_.end(),
      [this](const TrackedObject & t) { return t.missed_frames > config_.max_missed_frames; }),
    tracked_objects_.end());
}

}  // namespace EuclideanCluster
