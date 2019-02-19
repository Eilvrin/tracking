#pragma once

#include <ros/ros.h>
#include <simple_data_publisher/Observation2D.h>
#include <simple_data_publisher/Observation2DArray.h>
#include <visualization_msgs/Marker.h>

namespace tracking {
class Visualization {
 public:
  Visualization(ros::NodeHandle& node, ros::NodeHandle& private_nh);
  ~Visualization();

  void cbVis(const simple_data_publisher::Observation2DArray::ConstPtr&
                 observations_msg);

 private:
  ros::Publisher pub_;
  ros::Subscriber sub_;
};
}  // namespace tracking
