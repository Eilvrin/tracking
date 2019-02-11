#pragma once

#include <Eigen/Dense>
#include <array>
#include <boost/array.hpp>
#include <vector>

namespace tracking {

class Observation {
 public:
  Observation() = default;

  Observation(const int64_t id, const double x, const double y,
              const boost::array<double, 4> noise_cov, const bool matched)
      : id_{id}, matched_{matched} {
    mu_(0) = x;
    mu_(1) = y;

    cov_(0, 0) = noise_cov[0];
    cov_(0, 1) = noise_cov[1];
    cov_(1, 0) = noise_cov[2];
    cov_(1, 1) = noise_cov[3];
  }

  void setMatched(const bool matched) { matched_ = matched; }
  bool getMatched() const { return matched_; }

  const Eigen::Vector2d& getObservation() const { return mu_; }
  const Eigen::Matrix2d& getObservationCov() const { return cov_; }

 private:
  int64_t id_;
  Eigen::Vector2d mu_;
  Eigen::Matrix2d cov_;
  // Whether or not the observation is matched with track
  bool matched_{false};
};
}  // namespace tracking
