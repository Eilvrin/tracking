#include <visualization/tracking_display.h>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/visualization_manager.h>

namespace visualization {

TrackingDisplay::TrackingDisplay() {
  covariance_property_ = new rviz::FloatProperty(
      "Covariance scaling", 2.0,
      "Scale covariance by N sigma (default 2 sigma, 95% of distribution)",
      this, SLOT(updateCovarianceScale()));

  history_length_property_ = new rviz::IntProperty(
      "History Length", 1, "Number of messages to keep displaying.", this,
      SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

void TrackingDisplay::onInitialize() { MFDClass::onInitialize(); }
void TrackingDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void TrackingDisplay::updateCovarianceScale() {
  if (!std::isfinite(covariance_property_->getFloat()) &&
      covariance_property_->getFloat() >= 0.)
    return;

  for (auto& visual : visuals_) {
    visual.second->setCovarianceScaling(covariance_property_->getFloat());
  }
}

void TrackingDisplay::updateHistoryLength() {
  const size_t history_size =
      static_cast<size_t>(history_length_property_->getInt());

  for (auto& visual : visuals_) {
    visual.second->setMaxHistorySize(history_size);
  }
}

void TrackingDisplay::processMessage(const kf_tracking::TrackletConstPtr& msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  const auto covariance_scale = std::fmax(0, covariance_property_->getFloat());

  boost::shared_ptr<TrackletVisual> visual;
  if (visuals_.count(msg->id)) {
    visual = visuals_[msg->id];
    visual->setMessage(msg);
  } else {
    visual.reset(new TrackletVisual(context_->getSceneManager(), scene_node_));
    visual->setMessage(msg);
    visual->setRandomColour();
    visuals_.emplace(msg->id, visual);
  }
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  visual->setCovarianceScaling(covariance_scale);
  visual->setMaxHistorySize(
      static_cast<size_t>(history_length_property_->getInt()));
}
}  // namespace visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualization::TrackingDisplay, rviz::Display)
