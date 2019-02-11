#include <ros/ros.h>
#include "visualization/visualization.h"

int main(int argc, char* argv[]) {
  
	ros::init(argc, argv, "visualization");
  	ros::NodeHandle n;
  	ros::NodeHandle private_nh("~");

  	tracking::Visualization viz(n, private_nh);
  	ros::spin();

    return 0;
}