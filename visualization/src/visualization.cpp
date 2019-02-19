#include "visualization/visualization.h"

namespace {
namespace sdp = simple_data_publisher;
}

namespace tracking {
Visualization::Visualization(ros::NodeHandle& node,
                             ros::NodeHandle& private_nh) {
  sub_ = node.subscribe("/observations2D", 100, &Visualization::cbVis, this);
  pub_ = node.advertise<visualization_msgs::Marker>(
      "/observation_visualization", 100);
}

Visualization::~Visualization() {}
void Visualization::cbVis(
    const sdp::Observation2DArray::ConstPtr& observations_msg) {
  for (size_t i = 0; i < observations_msg->observations.size(); ++i) {
    const sdp::Observation2D& observation = observations_msg->observations[i];

    if (!observation.valid) continue;

    visualization_msgs::Marker marker;
    marker.header = observations_msg->header;
    marker.ns = "world";
    marker.id = observation.id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = observation.x;
    marker.pose.position.y = observation.y;
    marker.pose.position.z = 0.05;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = observation.color[0];
    marker.color.g = observation.color[1];
    marker.color.b = observation.color[2];
    pub_.publish(marker);
  }
}

}  // namespace tracking
