#pragma once

#include <Eigen/Dense>
#include <deque>

namespace tracking {

class Track {
 public:
  Track() = default;

  void push(const Eigen::Vector4d& state) {
    states_.push_back(state);
    if (states_.size() > max_track_size_) {
      states_.pop_front();
    }
  }

  void push(const Eigen::Matrix4d& state_cov) {
    states_cov_.push_back(state_cov);
    if (states_cov_.size() > max_track_size_) {
      states_cov_.pop_front();
    }
  }

  void setPredictedObservation(const Eigen::Vector2d& predicted_observation) {
    pred_observation_ = predicted_observation;
  }

  const Eigen::Vector2d getPredictedObservation() const {
    return pred_observation_;
  }

  const Eigen::Vector4d& getLastPose() const { return states_.back(); }
  void setLastPose(const Eigen::Vector4d& state) { states_.back() = state; }

  const Eigen::Matrix4d& getLastPoseCov() const { return states_cov_.back(); }
  void setLastPoseCov(const Eigen::Matrix4d& state_cov) {
    states_cov_.back() = state_cov;
  }

  void setMatched(const bool matched) { matched_ = matched; }
  bool getMatched() const { return matched_; }

  void setObservationIdx(const int idx) { matched_observation_idx_ = idx; }
  int getObservationIdx() const { return matched_observation_idx_; }

  void setId(const int id) { id_ = id; }
  int getId() const { return id_; }

 private:
  int id_;
  size_t max_track_size_{200};
  // Track's states
  std::deque<Eigen::Vector4d> states_;
  std::deque<Eigen::Matrix4d> states_cov_;

  Eigen::Vector2d pred_observation_;
  int matched_observation_idx_;
  // Whether or not the track is matched with an observation
  bool matched_{false};
};
}  // namespace tracking
