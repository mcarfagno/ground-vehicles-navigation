#ifndef MPC_CONTROL__MPC_NODE_HPP_
#define MPC_CONTROL__MPC_NODE_HPP_

#include "mpc_control/mpc.hpp"
#include <memory>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <vector>
#include <vision_msgs/Detection3DArray.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace mpc {
class MpcNode {
public:
  MpcNode();
  ~MpcNode(){};
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher viz_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber obstacles_sub_;

  float rate_;
  float obs_safety_dist_;
  int mpc_horizon_steps_;
  MpcCmd prev_cmd_;
  std::optional<KinematicMpc> mpc_;
  std::optional<nav_msgs::Odometry> latest_odom_;
  std::optional<nav_msgs::Path> path_;
  std::optional<vision_msgs::Detection3DArray> obstacles_;

  casadi::DM odometry_to_casadi(const nav_msgs::Odometry &odom) const;
  casadi::DM cmd_to_casadi(const MpcCmd &cmd) const;
  casadi::DM path_to_casadi(const nav_msgs::Path &path) const;
  casadi::DM
  obstacles_to_casadi(const vision_msgs::Detection3DArray &obs) const;
  MpcCmd casadi_to_cmd(casadi::DM &in) const;

  void publish_mpc_cmd(double speed, double steer);
  void publish_rviz_markers(const casadi::DM &predicted_state_traj);
};

} // namespace mpc
#endif
