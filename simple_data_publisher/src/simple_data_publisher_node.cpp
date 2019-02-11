#include <ros/ros.h>
#include <simple_data_publisher/Observation2DArray.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>

namespace {
struct Colour {
  double r, g, b;
};

struct Noise {
  double r1, r1r2, r2r1, r2;
};

struct Measurement {
  double x, y;
};
}  // namespace

namespace {
namespace sdp = simple_data_publisher;

sdp::Observation2D createObservation(int id, Colour c, Noise r, bool valid,
                                     Measurement m) {
  sdp::Observation2D observation;
  observation.id = id;
  observation.noise_cov[0] = r.r1;
  observation.noise_cov[1] = r.r1r2;
  observation.noise_cov[2] = r.r2r1;
  observation.noise_cov[3] = r.r2;
  observation.valid = valid;
  observation.x = m.x;
  observation.y = m.y;
  observation.color[0] = c.r;
  observation.color[1] = c.g;
  observation.color[2] = c.b;
  return observation;
}

}  // namespace

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "simple_data_publisher");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  auto pub =
      node.advertise<sdp::Observation2DArray>("/observations2D", 100, true);

  // Read parameters
  std::string data_filename;
  private_nh.param("data_filename", data_filename, std::string("~/"));

  if (!boost::filesystem::exists(data_filename)) {
    ROS_ERROR("Data file %s does not exist.", data_filename.c_str());
    node.shutdown();
    return EXIT_FAILURE;
  }

  std::ifstream file(data_filename);
  if (!file.is_open()) {
    ROS_ERROR("Error openning a file %s", data_filename.c_str());
    node.shutdown();
    return EXIT_FAILURE;
  }

  while (pub.getNumSubscribers() == 0) {
    ros::Duration(1).sleep();
  }

  double dt;
  int id;
  Colour c;
  Noise r;
  bool valid;
  Measurement m;
  ros::Time timestamp;

  sdp::Observation2D observation;
  sdp::Observation2DArray observations_msg;

  bool is_initialized = false;
  while (file >> dt >> id >> c.r >> c.g >> c.b >> r.r1 >> r.r1r2 >> r.r2r1 >>
         r.r2 >> valid >> m.x >> m.y) {
    if (id == 1) {
      if (is_initialized) pub.publish(observations_msg);

      observations_msg.observations.clear();
      observations_msg.header.stamp = ros::Time::now();
      observations_msg.header.frame_id = "fixed_frame";
      observations_msg.dt = dt;
      is_initialized = true;

      ros::spinOnce();
      ros::Rate(1. / dt).sleep();
    }
    const auto observation = createObservation(id, c, r, valid, m);
    observations_msg.observations.push_back(observation);

    if (!ros::ok()) break;
  }

  // Publish last message
  if (!observations_msg.observations.empty()) pub.publish(observations_msg);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
