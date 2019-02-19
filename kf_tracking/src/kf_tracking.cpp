#include "kf_tracking/kf_tracking.h"
#include <kf_tracking/Tracklet.h>

namespace {
namespace sdp = simple_data_publisher;
}

namespace tracking {

KF_Tracking::~KF_Tracking() {}

bool KF_Tracking::init(ros::NodeHandle& node, ros::NodeHandle& private_nh) {
  sub_ = node.subscribe("/observations2D", 100, &KF_Tracking::cbTracking, this);
  pub_ = node.advertise<kf_tracking::Tracklet>("/tracks", 100);

  std::vector<double> process_noise;
  if (private_nh.getParam("process_noise", process_noise)) {
    if (process_noise.size() != Q_.cols() * Q_.rows()) {
      ROS_ERROR(
          "Provided wrong size of process noise matrix. Matrix should be of "
          "size %ldx%ld.",
          Q_.rows(), Q_.cols());
      return false;
    }
    Q_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
        process_noise.data());
  } else {
    Q_ = Eigen::Matrix4d::Identity() * 0.001;
    Q_(2, 2) = 0.1;
    Q_(3, 3) = 0.1;
  }

  std::vector<double> observation_matrix;
  if (private_nh.getParam("observation_matrix", observation_matrix)) {
    if (observation_matrix.size() != H_.cols() * H_.rows()) {
      ROS_ERROR(
          "Provided wrong size of observation matrix. Matrix should be of "
          "size %ldx%ld.",
          H_.rows(), H_.cols());
      return false;
    }
    H_ = Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>>(
        observation_matrix.data());
  } else {
    H_.setZero();
    H_(0, 0) = 1;
    H_(1, 1) = 1;
  }

  std::vector<double> initial_state_covariance;
  if (private_nh.getParam("initial_state_covariance",
                          initial_state_covariance)) {
    if (initial_state_covariance.size() != P_0_.cols() * P_0_.rows()) {
      ROS_ERROR(
          "Provided wrong size of initial state covariance matrix. Matrix "
          "should be of "
          "size %ldx%ld.",
          P_0_.rows(), P_0_.cols());
      return false;
    }
    P_0_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
        initial_state_covariance.data());
  } else {
    P_0_ = Eigen::Matrix4d::Identity() * 0.01;
    P_0_(2, 2) = 0.1;
    P_0_(3, 3) = 0.1;
  }

  private_nh.param("alpha", alpha_, 0.99f);

  boost::math::chi_squared_distribution<double> dist_chi(2);
  treshold_ = quantile(dist_chi, alpha_);
  return true;
}

void KF_Tracking::cbTracking(
    const sdp::Observation2DArray::ConstPtr& observations_msg) {
  ROS_INFO_STREAM("Received an observation message with timestamp "
                  << observations_msg->header.stamp << " and seq "
                  << observations_msg->header.seq);

  if (int(observations_msg->header.seq) - last_observation_seq_ != 1) {
    ROS_WARN_STREAM(
        "Calling reset! Detected jump in observation messages: previous seq"
        << last_observation_seq_ << " new msg seq "
        << observations_msg->header.seq);
    reset();
  }
  last_observation_seq_ = observations_msg->header.seq;

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

  for (auto& track : tracks_) {
    kf_tracking::Tracklet tracklet;
    tracklet.header = observations_msg->header;
    tracklet.id = track.getId();
    tracklet.pose.position.x = track.getLastPose()[0];
    tracklet.pose.position.y = track.getLastPose()[1];
    tracklet.pose.position.z = 0;

    const Eigen::Matrix4d cov = track.getLastPoseCov();
    tracklet.covariance[0] = cov(0, 0);
    tracklet.covariance[1] = cov(0, 1);
    tracklet.covariance[3] = cov(1, 0);
    tracklet.covariance[4] = cov(1, 1);
    pub_.publish(tracklet);
  }
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
    if (!track.getMatched()) continue;
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
    std::cout << "Update mu " << std::endl << x << std::endl;
    std::cout << "Update cov " << std::endl << P << std::endl;
  }
}

void KF_Tracking::initializeNewTracks(
    std::vector<Observation>& valid_observations) {
  for (auto& observation : valid_observations) {
    if (observation.getMatched()) continue;
    Track new_track;
    new_track.setId(tracks_.size());
    Eigen::Vector4d state = Eigen::Vector4d::Zero();
    state.head(2) = observation.getObservation();
    new_track.push(state);
    new_track.push(P_0_);
    tracks_.push_back(new_track);
  }
}

void KF_Tracking::reset() {
  tracks_.clear();
  last_observation_seq_ = -1;
}

}  // namespace tracking
