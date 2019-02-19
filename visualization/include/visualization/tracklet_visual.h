#pragma once
#include <kf_tracking/Tracklet.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/shape.h>

namespace Ogre {
class Vector3;
class Quaternion;
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace visualization {
class TrackletVisual {
 public:
  TrackletVisual(Ogre::SceneManager* scene_manager,
                 Ogre::SceneNode* parent_node);

  virtual ~TrackletVisual();

  void setMessage(const kf_tracking::TrackletConstPtr& msg);

  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setMaxHistorySize(const size_t history_size) {
    max_history_size_ = history_size;
  }

  void setRandomColour();

  void setCovarianceScaling(const float scaling) {
    covariance_scaling_ = scaling;
  }
  bool setCovariance(const kf_tracking::TrackletConstPtr& msg,
                     rviz::Shape& sphere);

 private:
  boost::shared_ptr<rviz::Shape> track_position_;
  boost::shared_ptr<rviz::Shape> track_covariance_;

  std::deque<boost::shared_ptr<rviz::Shape>> history_;
  std::deque<boost::shared_ptr<rviz::Shape>> history_covar_;

  size_t max_history_size_{1};

  float covariance_scaling_{1.};

  Ogre::ColourValue colour_;

  Ogre::SceneNode* frame_node_;
  Ogre::SceneManager* scene_manager_;
};

}  // namespace visualization
