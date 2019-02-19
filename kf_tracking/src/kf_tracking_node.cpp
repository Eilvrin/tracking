#include <ros/ros.h>
#include "kf_tracking/kf_tracking.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "kf_tracking");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  tracking::KF_Tracking track;
  if (!track.init(n, private_nh)) {
    ROS_ERROR("Error initializing KF tracking.");
    n.shutdown();
    return EXIT_FAILURE;
  }
  ros::spin();

  return 0;
}
