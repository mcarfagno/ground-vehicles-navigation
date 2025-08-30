#include <mppi_control/mppi_node.hpp>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mppi_node");
  mppi::MppiNode node;
  node.run();

  return 0;
}
