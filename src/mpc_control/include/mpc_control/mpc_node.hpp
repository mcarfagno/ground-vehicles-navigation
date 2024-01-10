#ifndef MPC_CONTROL__MPC_NODE_HPP_
#define MPC_CONTROL__MPC_NODE_HPP_

#include "mpc_control/mpc.hpp"
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <vector>
#include <vision_msgs/Detection3DArray.h>

namespace mpc {
class MpcNode {
public:
  MpcNode();
  ~MpcNode(){};
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher ctrl_pub_;
  ros::Publisher mpc_viz_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber obstacles_sub_;

  float rate_;
  std::optional<KinematicMpc> mpc_;
  std::optional<nav_msgs::Odometry> latest_odom_;
  std::optional<nav_msgs::Path> path_;
  std::optional<vision_msgs::Detection3DArray> obstacles_;

  KinematicMpc setup_mpc() const;
};

} // namespace mpc
#endif
