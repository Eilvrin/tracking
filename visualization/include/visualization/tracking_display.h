#pragma once

#include <kf_tracking/Tracklet.h>
#include <rviz/message_filter_display.h>
#include <visualization/tracklet_visual.h>

namespace rviz {
class FloatProperty;
class IntProperty;
}  // namespace rviz

namespace visualization {
class TrackingDisplay
    : public rviz::MessageFilterDisplay<kf_tracking::Tracklet> {
  Q_OBJECT
 public:
  TrackingDisplay();
  virtual ~TrackingDisplay() = default;

 protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

 private Q_SLOTS:
  void updateCovarianceScale();
  void updateHistoryLength();

 private:
  void processMessage(const kf_tracking::TrackletConstPtr& msg);

  std::map<int, boost::shared_ptr<TrackletVisual>> visuals_;
  // User-editable property variables.
  rviz::FloatProperty* covariance_property_;
  rviz::IntProperty* history_length_property_;
};
}  // namespace visualization
