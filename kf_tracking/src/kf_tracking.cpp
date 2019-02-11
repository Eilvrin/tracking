#include "kf_tracking/kf_tracking.h"

namespace {
namespace sdp = simple_data_publisher;
}

namespace tracking {
KF_Tracking::KF_Tracking(ros::NodeHandle& node, ros::NodeHandle& private_nh) {
  sub_ = node.subscribe("/observations2D", 100, &KF_Tracking::cbTracking, this);

  // TODO load matrices from parameters
  // TODO load alpha from parameters
  Q_ = Eigen::Matrix4d::Identity() * 0.001;
  Q_(2, 2) = 0.1;
  Q_(3, 3) = 0.1;

  H_(0, 0) = 1;
  H_(1, 1) = 1;

  P_0_ = Eigen::Matrix4d::Identity() * 0.01;
  P_0_(2, 2) = 0.1;
  P_0_(3, 3) = 0.1;

  boost::math::chi_squared_distribution<double> dist_chi(2);
  treshold_ = quantile(dist_chi, alpha_);
}

KF_Tracking::~KF_Tracking() {}

void KF_Tracking::cbTracking(
    const sdp::Observation2DArray::ConstPtr& observations_msg) {
  ROS_INFO_STREAM("Received an observation message with timestamp "
                  << observations_msg->header.stamp << "and seq "
                  << observations_msg->header.seq);
  // TODO reset
  predict(observations_msg->dt);

  std::vector<Observation> valid_observations;
  for (auto& observation : observations_msg->observations) {
    if (!observation.valid) continue;
    valid_observations.emplace_back(observation.id, observation.x,
                                    observation.y, observation.noise_cov,
                                    false);
  }

  associateObservations(valid_observations);
  update(valid_observations);
  initializeNewTracks(valid_observations);
}

void KF_Tracking::predict(const double dt) {
  // Constant velocity MM
  Eigen::Matrix4d F(Eigen::Matrix4d::Identity());
  F(0, 2) = dt;
  F(1, 3) = dt;

  for (auto& track : tracks_) {
    Eigen::Vector4d mu = track.getLastPose();
    mu = F * mu;
    track.push(mu);
    Eigen::Matrix4d cov = track.getLastPoseCov();
    cov = F * cov * F.transpose() + Q_;
    track.push(cov);
    Eigen::Vector2d pred_observation = H_ * mu;
    track.setPredictedObservation(pred_observation);
    track.setMatched(false);
  }
}

void KF_Tracking::associateObservations(
    std::vector<Observation>& valid_observations) {
  if (valid_observations.empty() || tracks_.empty()) return;

  Eigen::MatrixXd D(tracks_.size(), valid_observations.size());
  D.setZero();
  for (size_t i = 0; i < tracks_.size(); ++i) {
    for (size_t j = 0; j < valid_observations.size(); ++j) {
      auto v = valid_observations[j].getObservation() -
               tracks_[i].getPredictedObservation();
      auto S = H_ * tracks_[i].getLastPoseCov() * H_.transpose() +
               valid_observations[j].getObservationCov();
      D(i, j) = v.transpose() * S.inverse() * v;
    }
  }
  bool terminated = false;

  while (!terminated) {
    int i, j;
    double d_min = D.minCoeff(&i, &j);
    if ((d_min < std::numeric_limits<double>::infinity()) &&
        (d_min < treshold_)) {
      tracks_[i].setMatched(true);
      valid_observations[j].setMatched(true);
      tracks_[i].setObservationIdx(j);
      D.row(i).setConstant(std::numeric_limits<double>::infinity());
      D.col(j).setConstant(std::numeric_limits<double>::infinity());
    } else {
      terminated = true;
    }
  }
}

void KF_Tracking::update(std::vector<Observation>& valid_observations) {
  for (auto& track : tracks_) {
    if (!track.getMatched()) return;
    int observation_idx = track.getObservationIdx();
    Eigen::Matrix4d P = track.getLastPoseCov();
    auto v = valid_observations[observation_idx].getObservation() -
             track.getPredictedObservation();
    auto S = H_ * P * H_.transpose() +
             valid_observations[observation_idx].getObservationCov();
    // Update
    auto K = P * H_.transpose() * S.inverse();
    const Eigen::Vector4d x = track.getLastPose() + K * v;
    P = P - K * H_ * P;
    track.setLastPose(x);
    track.setLastPoseCov(P);
    std::cout << "Update mu " << x << std::endl;
    std::cout << "Update cov " << P << std::endl;
  }
}

void KF_Tracking::initializeNewTracks(
    std::vector<Observation>& valid_observations) {
  for (auto& observation : valid_observations) {
    if (observation.getMatched()) return;
    Track new_track;
    new_track.setId(tracks_.size());
    Eigen::Vector4d state = Eigen::Vector4d::Zero();
    state.head(2) = observation.getObservation();
    new_track.push(state);
    new_track.push(P_0_);
    tracks_.push_back(new_track);
  }
}
}  // namespace tracking
