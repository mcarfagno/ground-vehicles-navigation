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
  private_nh_.param("rate", rate_, float(10.0));
  private_nh_.param("control_horizon_len", mpc_horizon_steps_, int(10));
  private_nh_.param("obstacles_safety_distance", obs_safety_dist_, float(0.25));

  // publishers
  cmd_pub_ =
      nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 10);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mpc/markers", 10);

  // subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "/gem/base_footprint/odom", 1,
      [this](const nav_msgs::OdometryConstPtr &msg) { latest_odom_ = *msg; });
  path_sub_ = nh_.subscribe<nav_msgs::Path>(
      "/mpc/path", 1,
      [this](const nav_msgs::PathConstPtr &msg) { path_ = *msg; });
  obstacles_sub_ = nh_.subscribe<vision_msgs::Detection3DArray>(
      "/mpc/obstacles", 1,
      [this](const vision_msgs::Detection3DArrayConstPtr &msg) {
        obstacles_ = *msg;
      });
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
      auto model = KinematicModel();
      auto params = MpcParameters();
      params.DT = 1./rate_;
      params.N = mpc_horizon_steps_;
      params.min_obstacle_margin  = obs_safety_dist_;

      // NOTE: I am assuming path and obstacles not changing
      // this is mostly due to having pre-fixed sizes for the obstacles
      // if the number of obstacles changes the problem must be rebuilt
      mpc_ = KinematicMpc(model, params, path_to_casadi(path_.value()),
                          obstacles_to_casadi(obstacles_.value()));
    }

    // control loop
    auto mpc_dict_in = casadi::DMDict();
    mpc_dict_in[INITIAL_STATE_DICT_KEY] =
        odometry_to_casadi(latest_odom_.value());
    mpc_dict_in[INITIAL_CONTROL_DICT_KEY] = cmd_to_casadi(prev_cmd_);

    auto result = mpc_->solve(mpc_dict_in);
    if (result.has_value()) {
      auto ctrl = casadi_to_cmd(result.value()[OPTIMIZED_CONTROL_DICT_KEY]);
      auto speed = std::hypot(latest_odom_.value().twist.twist.linear.x,
                              latest_odom_.value().twist.twist.linear.y) +
                   ctrl.acceleration * 1. / rate_;
      publish_mpc_cmd(speed, ctrl.steer);
      publish_rviz_markers(result.value()[OPTIMIZED_TRAJECTORY_DICT_KEY]);
      prev_cmd_ = ctrl;
    } else {
      publish_mpc_cmd(0.0, 0.0);
      prev_cmd_ = {0.0, 0.0};
    }

    loop_rate.sleep();
  }
}

void MpcNode::publish_mpc_cmd(double speed, double steer) {
  auto cmd_msg = ackermann_msgs::AckermannDrive();
  cmd_msg.speed = speed;
  cmd_msg.steering_angle = steer;
  cmd_pub_.publish(cmd_msg);
}

void MpcNode::publish_rviz_markers(const casadi::DM &predicted_state_traj) {
  visualization_msgs::MarkerArray marker_arr;

  // 1- publish optimized state x and y
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mpc_path_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.frame_locked = true;

  for (std::size_t i = 0;
       i < static_cast<std::size_t>(predicted_state_traj.size1()); i++) {
    geometry_msgs::Point point;
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
  auto tmp = casadi::DM(path.poses.size(), 4);
  for (std::size_t i = 0; i < path.poses.size(); i++) {
    tmp(i, casadi::Slice()) = {
        path.poses[i].pose.position.x, path.poses[i].pose.position.y,
        tf::getYaw(path.poses[i].pose.orientation), MPC_REF_SPEED};
  }

  //Stop at last point
  tmp(tmp.size1()-1, 3) = 0.0;
  return tmp;
}

casadi::DM
MpcNode::obstacles_to_casadi(const vision_msgs::Detection3DArray &obs) const {
  auto tmp = casadi::DM(obs.detections.size(), 3);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp(i, casadi::Slice()) = {
        obs.detections[i].bbox.center.position.x,
        obs.detections[i].bbox.center.position.y,
	obs.detections[i].bbox.size.x
    };
  }
  return tmp;
}

MpcCmd MpcNode::casadi_to_cmd(casadi::DM &in) const {
  return MpcCmd(in(0, 0).scalar(), in(0, 1).scalar());
}

} // namespace mpc
