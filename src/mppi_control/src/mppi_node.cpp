#include "mppi_control/mppi_node.hpp"

namespace mppi {
MppiNode :: MppiNode() : private_nh_("~") {

  // variables
  path_ = std::nullopt;
  obstacles_ = std::nullopt;
  mppi_ = std::nullopt;
  latest_odom_ = std::nullopt;

  // params
  private_nh_.param("rate", rate_, float(20.0));
  private_nh_.param("control_horizon_len", mpc_horizon_steps_, int(30));
  private_nh_.param("number_of_samples", mpc_rollouts_, int(1000));
  private_nh_.param("obstacles_safety_distance", obs_safety_dist_, float(0.25));

  // weights of the cost function terms
  private_nh_.param("x_pos_error_weight", x_weight_, float(50.0));
  private_nh_.param("y_pos_error_weight", y_weight_, float(50.0));
  private_nh_.param("heading_pos_error_weight", yaw_weight_, float(1.0));
  private_nh_.param("speed_error_weight", speed_weight_, float(20.0));
  private_nh_.param("steer_noise", steer_noise_, float(0.5));
  private_nh_.param("acceleration_noise", acc_noise_, float(0.1));

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

void MppiNode::run() {
  ROS_INFO("MPPI controller node start");
  ros::Rate loop_rate(rate_);
  while (ros::ok()) {
    ros::spinOnce();

    // check for tOpics
    if (!obstacles_.has_value() || !path_.has_value() ||
        !latest_odom_.has_value()) {
      ROS_WARN("MPPI waiting for necessary topics.");
      loop_rate.sleep();
      continue;
    }

    // create mppi instance
    if (!mppi_.has_value()) {
      auto mppi = MPPI(1. / rate_, mpc_horizon_steps_,
                   mpc_rollouts_,0.0,50.0,1.0);
      mppi_.emplace(mppi);
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
      mppi_->reset();
      publish_mpc_cmd(0.0, 0.0);

      continue;
    }

    auto opt =
        mppi_->compute_optimal_input(path_to_matrix(path_.value()),
                                     odometry_to_matrix(latest_odom_.value()));
    const auto ctrl = std::get<0>(opt); // (steer,acc)
    const Eigen::MatrixXf x_opt = std::get<1>(opt);
    const auto x_sampled = std::get<2>(opt);
    auto speed = std::hypot(latest_odom_.value().twist.twist.linear.x,
                            latest_odom_.value().twist.twist.linear.y) +
                 ctrl.second * 1. / rate_;
    publish_mpc_cmd(speed, ctrl.first);
    publish_rviz_markers(x_opt, x_sampled);

    loop_rate.sleep();
  }
}

void MppiNode::publish_mpc_cmd(double speed, double steer) {
  auto cmd_msg = ackermann_msgs::AckermannDrive();
  cmd_msg.speed = speed;
  cmd_msg.steering_angle = steer;
  cmd_pub_.publish(cmd_msg);
}

void MppiNode::publish_rviz_markers(
    const Eigen::MatrixXf &optimal_traj,
    const std::vector<Eigen::MatrixXf> sampled_traj_list) {
  visualization_msgs::MarkerArray marker_arr;

  // 1- publish optimized trajectory
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "mppi_path_marker";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.frame_locked = true;
  marker.lifetime = ros::Duration(1./rate_);

  marker.points.resize(optimal_traj.rows());
  for (std::size_t i = 0; i < optimal_traj.rows(); i++) {
    marker.points[i].x = optimal_traj(i, 0);
    marker.points[i].y = optimal_traj(i, 1);
  }
  marker_arr.markers.push_back(marker);

  // 2- publish sampled trajectories
  for (std::size_t i = 0; i < sampled_traj_list.size(); i++) {
    const auto &sample = sampled_traj_list[i];
    visualization_msgs::Marker s;
    s.header.frame_id = "world";
    s.header.stamp = ros::Time::now();
    s.ns = "mppi_path_marker";
    s.id = i + 1;
    s.type = visualization_msgs::Marker::LINE_STRIP;
    s.action = visualization_msgs::Marker::ADD;
    s.scale.x = 0.2;
    s.color.r = 0.5;
    s.color.g = 0.5;
    s.color.b = 0.5;
    s.color.a = 0.35;
    s.frame_locked = true;
    s.lifetime = ros::Duration(1./rate_);

    s.points.resize(sample.rows());
    for (std::size_t i = 0; i < sample.rows(); i++) {
      s.points[i].x = sample(i, 0);
      s.points[i].y = sample(i, 1);
    }
    marker_arr.markers.push_back(s);
  }

  viz_pub_.publish(std::move(marker_arr));
}

Eigen::Vector4f
MppiNode::odometry_to_matrix(const nav_msgs::Odometry &odom) const {
  Eigen::Vector4f state(
      odom.pose.pose.position.x, odom.pose.pose.position.y,
      tf::getYaw(odom.pose.pose.orientation),
      std::hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y));
  return state;
}

Eigen::MatrixXf MppiNode::path_to_matrix(const nav_msgs::Path &path) const {

  Eigen::MatrixXf tmp;
  tmp.resize(path.poses.size(), 4);
  for (std::size_t i = 0; i < path.poses.size(); i++) {
    tmp.row(i) = Eigen::Vector4f(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                  tf::getYaw(path.poses[i].pose.orientation), MPPI_REF_SPEED);
  }

  // workaround for lack of heading from GPS path
  for (std::size_t i = 1; i < tmp.rows(); i++) {
    tmp(i, 2) =
        std::atan2(tmp(i, 1) - tmp(i - 1, 1), tmp(i, 0) - tmp(i - 1, 0));
  }

  // Decelerate and stop at end of Path
  tmp(tmp.rows() - 1, 3) = 0.0;
  return tmp;
}

Eigen::MatrixXf
MppiNode::obstacles_to_matrix(const vision_msgs::Detection3DArray &obs) const {
  Eigen::MatrixXf tmp;
  tmp.resize(obs.detections.size(), 3);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp.row(i) = Eigen::Vector3f(obs.detections[i].bbox.center.position.x,
                  obs.detections[i].bbox.center.position.y,
                  obs.detections[i].bbox.size.x);
  }
  return tmp;
}

} // namespace mppi


/**
 * @brief Converts latitude and longitude to global X, Y coordinates,
 *        using an equirectangular projection.
 *
 *  @returns pair(meters east of lon0, meters north of lat0)
 *
 *  Sources: http://www.movable-type.co.uk/scripts/latlong.html
 *           https://github.com/MPC-Car/StochasticLC/blob/master/controller.py
 */
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
