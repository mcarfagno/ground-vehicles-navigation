#include "mpc_control/mpc_node.hpp"

namespace mpc {
// TODO(marcello): implement node
MpcNode::MpcNode() : private_nh_("~") {

  // variables
  path_ = std::nullopt;
  obstacles_ = std::nullopt;
  mpc_ = std::nullopt;
  latest_odom_ = std::nullopt;
  prev_cmd_ = {0.0, 0.0};

  // params
  private_nh_.param("rate", rate_, double(10.0));
  private_nh_.param("control_horizon_len", mpc_horizon_steps_, std::size_t(10));

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

void MpcNode::run() {
  ROS_INFO("MPC controller node start");
  ros::Rate loop_rate(rate_);
  while (ros::ok()) {
    ros::spinOnce();

    // check for topics
    if (!obstacles_.has_value() || !path_.has_value() ||
        !latest_odom_.has_value()) {
      ROS_WARN("MPC waiting for necessary topics.");
      loop_rate.sleep();
      continue;
    }

    // create mpc instance
    if (!mpc_.has_value()) {
      ROS_INFO("Creating CasADi problem instance");
      model = KinematicModel();
      params = MpcParameters();
      params.DT = rate_;
      params.N = mpc_horizon_steps_;

      // NOTE: I am assuming path and obstacles not changing
      // this is mostly due to having pre-fixed sizes for the obstacles
      // if the number of obstacles changes the problem must be rebuilt
      mpc_ = KinematicMpc(model, params, path_to_casadi(path_.value()),
                          obstacles_to_casadi(obstacles));
    }

    // control loop
    auto mpc_dict_in = casadi::DMDict();
    mpc_dict_in[INITIAL_STATE_DICT_KEY] = odom_to_casadi(latest_odom_.value());
    mpc_dict_in[INITIAL_CONTROL_DICT_KEY] = cmd_to_casadi(prev_cmd_);

    auto result = mpc_->solve(mpc_dict_in);
    if (result.has_value()) {
      auto ctrl =
          mpc_->casadi_to_cmd(result.value()[OPTIMIZED_CONTROL_DICT_KEY]);
      publish_mpc_cmd(latest_state_.value().speed +
                          ctrl.acceleration * control_time_step_,
                      ctrl.steer);
      publish_rviz_markers(result.value()[OPTIMIZED_TRAJECTORY_DICT_KEY]);
      prev_cmd_ = ctrl;
    } else {
      publish_mpc_cmd(0.0, 0.0);
      prev_cmd_ = {0.0, 0.0};
    }
  }
}

void MpcNode::publish_mpc_cmd(double speed, double steer) {
  auto cmd_msg = ackermann_msgs::AckermannDrive();
  cmd_msg->header.stamp = this->get_clock()->now();
  cmd_msg->speed = speed;
  cmd_msg->steering_angle = steer;
  command_pub_.publish(cmd_msg);
}

void MpcNode::publish_rviz_markers(const casadi::DM &predicted_state_traj) {
  visualization_msgs::MarkerArray marker_arr;

  // 1- publish optimized state x and y
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = this->get_clock()->now();
  marker.ns = "mpc_path_marker";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.frame_locked = true;

  for (std::size_t i = 0;
       i < static_cast<std::size_t>(predicted_state_traj.size1()); i++) {
    geometry_msgs::msg::Point point;
    point.x = predicted_state_traj(i, 0).scalar();
    point.y = predicted_state_traj(i, 1).scalar();
    marker.points.push_back(point);
  }
  marker_arr.markers.push_back(marker);
  viz_pub_.publish(std::move(marker_arr));
}

casadi::DM MpcNode::odometry_to_casadi(const nav_msgs::Odometry &odom) const {
  return casadi::DM(
      {odom.pose.pose.position.x, odom.pose.pose.position.y,
       tf::getYaw(odom.pose.pose.orientation),
       std::hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)});
}

casadi::DM MpcNode::cmd_to_casadi(const MpcCmd &cmd) const {
  return casadi::DM({cmd.acceleration, cmd.steer});
}

casadi::DM MpcNode::path_to_casadi(const nav_msgs::Path &path) const {
  auto tmp = casadi::DM(path.poses.size(), nx_);
  for (std::size_t i = 0; i < path.poses.size(); i++) {
    tmp(i, casadi::Slice()) = {
        path.poses[i].pose.position.x, path.poses[i].pose.position.y,
        tf::getYaw(path.poses[i].pose.orientation), MPC_REF_SPEED};
  }
  return tmp;
}

casadi::DM
MpcNode::obstacles_to_casadi(const vision_msgs::Detection3DArray &obs) const {
  auto tmp = casadi::DM(obs.detections.size(), 2);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp(i, casadi::Slice()) = {
        obs.detections[i].results.front().pose.pose.position.x,
        obs.detections[i].results.front().pose.pose.position.y};
  }
  return tmp;
}

MpcCmd casadi_to_cmd(casadi::DM &in) const {
  return MpcCmd(in(0, 0).scalar(), in(0, 1).scalar());
}

} // namespace mpc
