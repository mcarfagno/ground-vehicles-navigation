#include <ros/ros.h>
#include <mpc_control/mpc_node.hpp>

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"mpc_node");
  mpc::MpcNode node;
  node.run();

  return 0;
}
