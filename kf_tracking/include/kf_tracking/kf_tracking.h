#pragma once

#include <ros/ros.h>
#include <simple_data_publisher/Observation2D.h>
#include <simple_data_publisher/Observation2DArray.h>
#include <boost/math/distributions/chi_squared.hpp>

#include <vector>
#include "kf_tracking/observation.h"
#include "kf_tracking/track.h"

namespace tracking {
class KF_Tracking {
 public:
  KF_Tracking(ros::NodeHandle& node, ros::NodeHandle& private_nh);
  ~KF_Tracking();
  void cbTracking(const simple_data_publisher::Observation2DArray::ConstPtr&
                      observations_msg);

 private:
  void predict(const double dt);
  void associateObservations(std::vector<Observation>& valid_observations);
  void update(std::vector<Observation>& valid_observations);
  void initializeNewTracks(std::vector<Observation>& valid_observations);

  ros::Publisher pub_;
  ros::Subscriber sub_;

  std::vector<Track> tracks_;

  // Process noise covariance
  Eigen::Matrix4d Q_;
  // Initial state covariance
  Eigen::Matrix4d P_0_;
  // Observation matrix
  Eigen::Matrix<double, 2, 4> H_;

  float alpha_{0.99f};
  double treshold_;
};
}  // namespace tracking
