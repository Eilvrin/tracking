#include <visualization/tracklet_visual.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

namespace visualization {

TrackletVisual::TrackletVisual(Ogre::SceneManager* scene_manager,
                               Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

TrackletVisual::~TrackletVisual() {
  scene_manager_->destroySceneNode(frame_node_);
}

void TrackletVisual::setMessage(const kf_tracking::TrackletConstPtr& msg) {
  track_position_.reset(
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
  track_covariance_.reset(
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
  Ogre::Vector3 position(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  Ogre::Vector3 scale(0.05, 0.05, 0.05);
  track_position_->setScale(scale);
  track_position_->setPosition(position);

  setCovariance(msg, *track_covariance_);

  track_position_->setColor(colour_);
  auto covariance_colour = colour_;
  covariance_colour.a = 0.3;
  track_covariance_->setColor(covariance_colour);

  history_.push_back(track_position_);
  history_covar_.push_back(track_covariance_);

  while (history_.size() > max_history_size_) {
    history_.pop_front();
    history_covar_.pop_front();
  }
}

// Position and orientation are passed through to the SceneNode.
void TrackletVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void TrackletVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void TrackletVisual::setRandomColour() {
  Ogre::Real r = Ogre::Math::RangeRandom(0, 1);
  Ogre::Real g = Ogre::Math::RangeRandom(0, 1);
  Ogre::Real b = Ogre::Math::RangeRandom(0, 1);
  colour_ = Ogre::ColourValue(r, g, b, 1);
  track_position_->setColor(colour_);
  auto covariance_colour = colour_;
  covariance_colour.a = 0.3;
  track_covariance_->setColor(covariance_colour);
}

bool TrackletVisual::setCovariance(const kf_tracking::TrackletConstPtr& msg,
                                   rviz::Shape& sphere) {
  Ogre::Vector3 position(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);
  track_covariance_->setPosition(position);
  Eigen::Matrix3d covariance;
  if (msg->covariance.size() != covariance.cols() * covariance.rows()) {
    ROS_WARN(
        "Provided wrong size of covariance matrix. Matrix "
        "should be of "
        "size %ldx%ld. Setting covariance scale to 0.",
        covariance.rows(), covariance.cols());
    Ogre::Vector3 covariance_scale(0.0, 0.0, 0.0);
    sphere.setScale(covariance_scale);
    return false;
  }
  covariance(0, 0) = msg->covariance[0];
  covariance(0, 1) = msg->covariance[1];
  covariance(0, 2) = msg->covariance[2];
  covariance(1, 0) = msg->covariance[3];
  covariance(1, 1) = msg->covariance[4];
  covariance(1, 2) = msg->covariance[5];
  covariance(2, 0) = msg->covariance[6];
  covariance(2, 1) = msg->covariance[7];
  covariance(2, 2) = msg->covariance[8];

  Eigen::EigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  if (eigensolver.info() != Eigen::Success) {
    ROS_WARN(
        "Failed to compute eigen vectors/values. "
        "Setting covariance scale to 0.");
    Ogre::Vector3 covariance_scale(0.0, 0.0, 0.0);
    sphere.setScale(covariance_scale);
    return false;
  }
  // 2, not 1 because it's a diameter (+- sigma)
  Ogre::Vector3 covariance_scale(
      covariance_scaling_ * 2 * sqrt(eigensolver.eigenvalues().real()[0]),
      covariance_scaling_ * 2 * sqrt(eigensolver.eigenvalues().real()[1]),
      std::max(0.01, 2 * sqrt(eigensolver.eigenvalues().real()[2])));
  sphere.setScale(covariance_scale);
  Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors().real();
  Eigen::Vector3d c0 = eigenvectors.block<3, 1>(0, 0);
  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3, 1>(0, 1);
  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3, 1>(0, 2);
  c2.normalize();
  eigenvectors << c0, c1, c2;
  Ogre::Quaternion orientation;
  orientation.FromRotationMatrix(Ogre::Matrix3(
      eigenvectors(0, 0), eigenvectors(0, 1), eigenvectors(0, 2),
      eigenvectors(1, 0), eigenvectors(1, 1), eigenvectors(1, 2),
      eigenvectors(2, 0), eigenvectors(2, 1), eigenvectors(2, 2)));
  sphere.setOrientation(orientation);
  return true;
}

}  // namespace visualization
