#include "mpc_control/mpc_node.hpp"

namespace mpc {
// TODO(marcello): implement node
MpcNode::MpcNode() : private_nh_("~") {

  // variables
  path_ = std::nullopt;
  obstacles_ = std::nullopt;
  mpc_ = std::nullopt;
  latest_odom_ = std::nullopt;

  // params
  private_nh_.param("rate", rate_, double(10.0));

  // publishers
  ctrl_pub_ =
      nh_.advertise<ackerman_msg::AckermannDrive>("/gem/ackermann_cmd", 10);
  mpc_viz_pub_ = nh_.advertise<visualization_msg::VisualizationMarkerArray>(
      "/mpc/markers", 10);

  // subscribers
  odom_sub_ = nh_.subscribe(
      "/gem/base_footprint/odom",
      [&](const nav_msgs::OdometryPtr &msg) { latest_odom_ = *msg; });
  path_sub_ = nh_.subscribe(
      "/mpc/path", [&](const nav_msgs::PathPtr &msg) { path_ = *msg; });
  obstacles_sub_ = nh_.subscribe(
      "/mpc/obstacles",
      [&](const vision_msgs::Detection3DArrayPtr &msg) { obstacles_ = *msg; });
}

KinematicMpc MpcNode::setup_mpc() const {
  // TODO: create MPC
}

void MpcNode::run() {
  ROS_INFO("MPC controller node start");
  ros::Rate loop_rate(rate_);
  while (ros::ok()) {
    ros::spinOnce();

    // check for topics and create mpc instance
    if (!obstacles_ || !path_ || !latest_odom_) {
      ROS_WARN("MPC waiting for necessary topics.");
      loop_rate.sleep();
      continue;
    }

    if (!mpc) {
      ROS_INFO("Creating CasADi problem instance");
      mpc = setup_mpc();
    }

    // control loop
    // TODO: MPC LOOP
  }
}
} // namespace mpc
