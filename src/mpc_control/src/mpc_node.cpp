#include "mpc_control/mpc_node.hpp"

namespace mpc {
MpcNode::MpcNode() : private_nh_("~") {

  // variables
  path_ = std::nullopt;
  obstacles_ = std::nullopt;
  mpc_ = std::nullopt;
  latest_odom_ = std::nullopt;

  prev_mpc_traj_ = std::nullopt;
  prev_mpc_cmd_ = std::nullopt;
  prev_cmd_ = {0.0, 0.0};

  // params
  private_nh_.param("rate", rate_, float(10.0));
  private_nh_.param("control_horizon_len", mpc_horizon_steps_, int(10));
  private_nh_.param("obstacles_safety_distance", obs_safety_dist_, float(0.25));

  // weights of the cost function terms
  private_nh_.param("x_pos_error_weight", x_weight_, float(1.0));
  private_nh_.param("y_pos_error_weight", y_weight_, float(1.0));
  private_nh_.param("heading_pos_error_weight", yaw_weight_, float(0.1));
  private_nh_.param("speed_error_weight", speed_weight_, float(1.0));
  private_nh_.param("acceleration_rate_weight", acc_rate_weight_, float(10.0));
  private_nh_.param("steer_rate_weight", steer_rate_weight_, float(100.0));
  private_nh_.param("obstacle_distance_weight", dist_weight_, float(5.0));

  // publishers
  cmd_pub_ =
      nh_.advertise<ackermann_msgs::AckermannDrive>("/gem/ackermann_cmd", 10);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/mpc/markers", 10);

  // subscribers
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      "/gem/base_footprint/odom", 1,
      [this](const nav_msgs::OdometryConstPtr &msg) { latest_odom_ = *msg; });
  path_sub_ = nh_.subscribe<nav_msgs::Path>(
      "/mpc/path", 1, [this](const nav_msgs::PathConstPtr &msg) {
        path_ = *msg;

        // convert from GPS to World
        std::for_each(path_.value().poses.begin(), path_.value().poses.end(),
                      [&](auto &p) {
                        auto xy = latlon_to_XY(
                            GPS_WORLD_ORIGIN_LAT, GPS_WORLD_ORIGIN_LON,
                            p.pose.position.x, p.pose.position.y);
                        p.pose.position.x = xy.first;
                        p.pose.position.y = xy.second;
                      });
      });
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
      params.DT = 1. / rate_;
      params.N = mpc_horizon_steps_;
      params.obstacle_margin = obs_safety_dist_;
      params.state_error_weights = {x_weight_, y_weight_, yaw_weight_,
                                    speed_weight_};
      params.control_rate_weights = {acc_rate_weight_, steer_rate_weight_};
      params.obstacle_avoidance_weight = dist_weight_;
      auto obs = obstacles_to_casadi(obstacles_.value());
      auto path = path_to_casadi(path_.value());
      // NOTE: I am assuming path and obstacles not changing
      // this is mostly due to having pre-fixed sizes for the obstacles
      // if the number of obstacles changes the problem must be rebuilt
      mpc_ = KinematicMpc(model, params, path, obs);
    }

    // check for goal
    if (std::hypot(path_.value().poses.back().pose.position.x -
                       latest_odom_.value().pose.pose.position.x,
                   path_.value().poses.back().pose.position.y -
                       latest_odom_.value().pose.pose.position.y) <= 1.0) {
      ROS_INFO("Goal Reached. Resetting Controller");
      path_ = std::nullopt;
      obstacles_ = std::nullopt;
      latest_odom_ = std::nullopt;
      publish_mpc_cmd(0.0, 0.0);

      prev_cmd_ = {0.0, 0.0};
      prev_mpc_traj_ = std::nullopt;
      prev_mpc_cmd_ = std::nullopt;
      continue;
    }

    // control loop
    auto mpc_dict_in = casadi::DMDict();
    mpc_dict_in[INITIAL_STATE_DICT_KEY] =
        odometry_to_casadi(latest_odom_.value());
    mpc_dict_in[INITIAL_CONTROL_DICT_KEY] = cmd_to_casadi(prev_cmd_);

    // use the previous solution to seed the solution
    // for the next step
    if (prev_mpc_cmd_.has_value()) {
      mpc_dict_in[CONTROL_GUESS_DICT_KEY] = prev_mpc_cmd_.value();
    }
    if (prev_mpc_traj_.has_value()) {
      mpc_dict_in[TRAJECTORY_GUESS_DICT_KEY] = prev_mpc_traj_.value();
    }

    auto result = mpc_->solve(mpc_dict_in);
    if (result.has_value()) {
      auto ctrl = casadi_to_cmd(result.value()[OPTIMIZED_CONTROL_DICT_KEY]);
      auto speed = std::hypot(latest_odom_.value().twist.twist.linear.x,
                              latest_odom_.value().twist.twist.linear.y) +
                   ctrl.acceleration * 1. / rate_;
      publish_mpc_cmd(speed, ctrl.steer);
      publish_rviz_markers(result.value()[OPTIMIZED_TRAJECTORY_DICT_KEY]);
      prev_mpc_cmd_ = result.value()[OPTIMIZED_CONTROL_DICT_KEY];
      prev_mpc_traj_ = result.value()[OPTIMIZED_TRAJECTORY_DICT_KEY];
      prev_cmd_ = ctrl;
    } else {
      publish_mpc_cmd(0.0, 0.0);
      prev_cmd_ = {0.0, 0.0};
      prev_mpc_traj_ = std::nullopt;
      prev_mpc_cmd_ = std::nullopt;
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

  // workaround for lack of heading from GPS path
  for (std::size_t i = 1; i < tmp.size1(); i++) {
    tmp(i, 2) = std::atan2(tmp(i, 1).scalar() - tmp(i - 1, 1).scalar(),
                           tmp(i, 0).scalar() - tmp(i - 1, 0).scalar());
  }

  // Decelerate and stop at end of Path
  tmp(tmp.size1() - 1, 3) = 0.0;
  return tmp;
}

casadi::DM
MpcNode::obstacles_to_casadi(const vision_msgs::Detection3DArray &obs) const {
  auto tmp = casadi::DM(obs.detections.size(), 3);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp(i, casadi::Slice()) = {obs.detections[i].bbox.center.position.x,
                               obs.detections[i].bbox.center.position.y,
                               obs.detections[i].bbox.size.x};
  }
  return tmp;
}

MpcCmd MpcNode::casadi_to_cmd(casadi::DM &in) const {
  return MpcCmd(in(0, 0).scalar(), in(0, 1).scalar());
}

} // namespace mpc

std::pair<double, double> latlon_to_XY(double lat0, double lon0, double lat1,
                                       double lon1) {
  auto R_earth = 6371000; // meters
  auto delta_lat = (lat1 - lat0) * (M_PI / 180);

  auto delta_lon = (lon1 - lon0) * (M_PI / 180);

  auto lat_avg = 0.5 * (lat1 * (M_PI / 180) + lat0 * (M_PI / 180));
  auto X = R_earth * delta_lon * std::cos(lat_avg);
  auto Y = R_earth * delta_lat;

  return std::make_pair(X, Y);
}
