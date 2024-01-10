#ifndef MPC_CONTROL__MPC_NODE_HPP_
#define MPC_CONTROL__MPC_NODE_HPP_

#include "mpc_control/mpc.hpp"
#include <memory>
#include <ros/ros.h>

namespace mpc {
// TODO(marcello): mpc ROS node
class MpcNode {
public:
  MpcNode(){};
  ~MpcNode(){};
  void run();

private:
  ros::NodeHandle nh_;
};

} // namespace mpc
#endif
