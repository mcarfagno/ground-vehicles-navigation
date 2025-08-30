#include <ros/ros.h>
#include <mppi_control/mppi_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"mppi_node");
  mppi::MppiNode node;
  node.run();

  return 0;
}
