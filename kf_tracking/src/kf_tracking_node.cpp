#include <ros/ros.h>
#include "kf_tracking/kf_tracking.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "kf_tracking");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");
  
  tracking::KF_Tracking track(n, private_nh);
  ros::spin();

  return 0;
}
