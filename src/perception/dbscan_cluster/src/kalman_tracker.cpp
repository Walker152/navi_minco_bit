#include "kalman_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>

#include <Eigen/Geometry>

namespace DBSCANCluster
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
  const float q_acc_x = std::max(1e-6f, config_.q_acc_x);
  const float q_acc_y = std::max(1e-6f, config_.q_acc_y);
  const float r_pos_x = std::max(1e-6f, config_.r_pos_x);
  const float r_pos_y = std::max(1e-6f, config_.r_pos_y);
  const float gate_scale = std::max(1.0f, config_.association_gate_scale);
  const float spatial_w_raw = std::max(0.0f, config_.association_spatial_weight);
  const float shape_w_raw = std::max(0.0f, config_.association_shape_weight);
  const float w_sum = spatial_w_raw + shape_w_raw;
  const float spatial_w = (w_sum > 1e-6f) ? (spatial_w_raw / w_sum) : 0.7f;
  const float shape_w = (w_sum > 1e-6f) ? (shape_w_raw / w_sum) : 0.3f;
  const float alpha_size = std::clamp(config_.alpha_size, 0.0f, 1.0f);
  const float alpha_orientation = std::clamp(config_.alpha_orientation, 0.0f, 1.0f);

  if (!(std::isfinite(dt) && dt > 1e-3f)) {
    dt = dt_default;
  }
  const int confirm_frames = std::max(1, config_.class_confirm_frames);

  const float dt2_half = 0.5f * dt * dt;

  Eigen::Matrix<float, 6, 6> F = Eigen::Matrix<float, 6, 6>::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  F(0, 4) = dt2_half;
  F(1, 5) = dt2_half;
  F(2, 4) = dt;
  F(3, 5) = dt;

  Eigen::Matrix<float, 2, 6> H;
  H.setZero();
  H(0, 0) = 1.0f;
  H(1, 1) = 1.0f;

  Eigen::Matrix<float, 6, 6> Q = Eigen::Matrix<float, 6, 6>::Identity();
  Q(0, 0) = q_pos_x;
  Q(1, 1) = q_pos_y;
  Q(2, 2) = q_vel_x;
  Q(3, 3) = q_vel_y;
  Q(4, 4) = q_acc_x;
  Q(5, 5) = q_acc_y;

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

  auto compute_bbox_iou_2d = [](const Eigen::Vector2f & center_a, const Eigen::Vector3f & size_a,
                               const Eigen::Vector2f & center_b, const Eigen::Vector3f & size_b) {
      const float a_half_x = 0.5f * std::max(0.01f, size_a.x());
      const float a_half_y = 0.5f * std::max(0.01f, size_a.y());
      const float b_half_x = 0.5f * std::max(0.01f, size_b.x());
      const float b_half_y = 0.5f * std::max(0.01f, size_b.y());

      const float a_min_x = center_a.x() - a_half_x;
      const float a_max_x = center_a.x() + a_half_x;
      const float a_min_y = center_a.y() - a_half_y;
      const float a_max_y = center_a.y() + a_half_y;

      const float b_min_x = center_b.x() - b_half_x;
      const float b_max_x = center_b.x() + b_half_x;
      const float b_min_y = center_b.y() - b_half_y;
      const float b_max_y = center_b.y() + b_half_y;

      const float inter_w = std::max(0.0f, std::min(a_max_x, b_max_x) - std::max(a_min_x, b_min_x));
      const float inter_h = std::max(0.0f, std::min(a_max_y, b_max_y) - std::max(a_min_y, b_min_y));
      const float inter_area = inter_w * inter_h;

      const float area_a = (a_max_x - a_min_x) * (a_max_y - a_min_y);
      const float area_b = (b_max_x - b_min_x) * (b_max_y - b_min_y);
      const float union_area = area_a + area_b - inter_area;

      if (union_area <= 1e-6f) {
        return 0.0f;
      }
      return inter_area / union_area;
    };

  for (size_t obj_idx = 0; obj_idx < current_objects.size(); ++obj_idx) {
    const Eigen::Vector2f z(current_objects[obj_idx].centroid.x(), current_objects[obj_idx].centroid.y());
    for (size_t trk_idx = 0; trk_idx < prev_track_count; ++trk_idx) {
      const float dist = (z - tracked_objects_[trk_idx].position).norm();
      if (dist > config_.match_distance_threshold * gate_scale) {
        continue;
      }

      const float spatial_cost = dist / std::max(1e-3f, config_.match_distance_threshold);
      const float iou = compute_bbox_iou_2d(
        z, current_objects[obj_idx].size,
        tracked_objects_[trk_idx].position, tracked_objects_[trk_idx].size);
      const float shape_cost = 1.0f - std::clamp(iou, 0.0f, 1.0f);
      const float association_cost = spatial_w * spatial_cost + shape_w * shape_cost;

      candidates.emplace_back(association_cost, static_cast<int>(obj_idx), static_cast<int>(trk_idx));
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
    const Eigen::Matrix<float, 6, 2> K = track.covariance * H.transpose() * S.inverse();

    track.state = track.state + K * innovation;
    track.covariance = (Eigen::Matrix<float, 6, 6>::Identity() - K * H) * track.covariance;
    track.position = track.state.head<2>();
    track.missed_frames = 0;

    auto & obj = current_objects[static_cast<size_t>(obj_idx)];
    obj.track_id = track.id;
    obj.vx = track.state(2);
    obj.vy = track.state(3);
    obj.speed = std::sqrt(obj.vx * obj.vx + obj.vy * obj.vy);

    // --- 1. PCA 边长 90度 防翻转对齐 ---
    if (track.size.squaredNorm() > 1e-6f) {
      float diff_no_swap = std::abs(obj.size.x() - track.size.x()) + std::abs(obj.size.y() - track.size.y());
      float diff_swap = std::abs(obj.size.x() - track.size.y()) + std::abs(obj.size.y() - track.size.x());

      // 如果交换长宽后与历史尺寸更匹配，说明 PCA 发生了 90 度主轴跳变
      if (diff_swap < diff_no_swap) {
        std::swap(obj.size.x(), obj.size.y());
        // 补偿 90 度旋转，以对齐历史坐标系
        Eigen::Quaternionf rot90(Eigen::AngleAxisf(static_cast<float>(M_PI) / 2.0f, Eigen::Vector3f::UnitZ()));
        obj.orientation = obj.orientation * rot90;
      }
    }

    // --- 2. 尺寸与角度的低通平滑 ---
    if (track.size.squaredNorm() < 1e-6f) {
      track.size = obj.size;
    } else {
      track.size = (1.0f - alpha_size) * track.size + alpha_size * obj.size;
    }

    if (track.orientation.norm() < 1e-6f) {
      track.orientation = obj.orientation;
    } else {
      if (track.orientation.dot(obj.orientation) < 0.0f) {
        obj.orientation.coeffs() *= -1.0f;
      }
      track.orientation = track.orientation.slerp(alpha_orientation, obj.orientation).normalized();
    }

    // --- 3. 漏桶算法 (Leaky Bucket)：防闪烁与防滞后 ---
    if (obj.speed > config_.dynamic_speed_threshold) {
      track.dynamic_match_frames += 2; // 移动时：+2 快速增加信任 (彻底解决滞后)
    } else if (obj.speed < config_.dynamic_speed_threshold * 0.5f) {
      track.dynamic_match_frames -= 1; // 停止/跳变时：-1 缓慢扣除信任 (彻底解决单帧跳变引起的闪烁)
    }

    // 将计数器限制在 0 到 confirm_frames * 3 之间（设置上限防止溢出）
    int max_frames = confirm_frames * 3;
    track.dynamic_match_frames = std::clamp(track.dynamic_match_frames, 0, max_frames);

    // 状态确认逻辑
    if (track.dynamic_match_frames >= confirm_frames) {
      track.dynamic_confirmed = true;
    } else if (track.dynamic_match_frames == 0) {
      track.dynamic_confirmed = false;
    }

    obj.size = track.size;
    obj.orientation = track.orientation;
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
    new_track.state <<
      current_objects[obj_idx].centroid.x(), current_objects[obj_idx].centroid.y(),
      0.0f, 0.0f,
      0.0f, 0.0f;
    new_track.position = new_track.state.head<2>();
    new_track.covariance = Eigen::Matrix<float, 6, 6>::Identity();
    new_track.covariance(2, 2) = 4.0f;
    new_track.covariance(3, 3) = 4.0f;
    new_track.covariance(4, 4) = 4.0f;
    new_track.covariance(5, 5) = 4.0f;
    new_track.missed_frames = 0;
    new_track.size = current_objects[obj_idx].size;
    new_track.orientation = current_objects[obj_idx].orientation;
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

  for (size_t trk_idx = 0; trk_idx < prev_track_count; ++trk_idx) {
    if (track_assigned[trk_idx]) {
      continue;
    }
    const auto & track = tracked_objects_[trk_idx];
    if (track.missed_frames > config_.max_missed_frames) {
      continue;
    }

    Detected_Obj predicted_obj;
    predicted_obj.track_id = track.id;
    predicted_obj.centroid = Eigen::Vector3f(track.state(0), track.state(1), 0.2f);
    predicted_obj.size = track.size;
    predicted_obj.orientation = track.orientation;
    predicted_obj.vx = track.state(2);
    predicted_obj.vy = track.state(3);
    predicted_obj.speed = std::sqrt(predicted_obj.vx * predicted_obj.vx + predicted_obj.vy * predicted_obj.vy);
    predicted_obj.dynamic_confirmed = track.dynamic_confirmed;
    current_objects.push_back(predicted_obj);
  }

  tracked_objects_.erase(
    std::remove_if(
      tracked_objects_.begin(), tracked_objects_.end(),
      [this](const TrackedObject & t) { return t.missed_frames > config_.max_missed_frames; }),
    tracked_objects_.end());
}

}  // namespace DBSCANCluster
