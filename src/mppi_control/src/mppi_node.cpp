#include "mppi_control/mppi_node.hpp"

namespace mppi {
MppiNode : MppiNode() : private_nh_("~") {

  // variables
  path_ = std::nullopt;
  obstacles_ = std::nullopt;
  mpc_ = std::nullopt;
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

    // check for topics
    if (!obstacles_.has_value() || !path_.has_value() ||
        !latest_odom_.has_value()) {
      ROS_WARN("MPPI waiting for necessary topics.");
      loop_rate.sleep();
      continue;
    }

    // create mppi instance
    if (!mppi_.has_value()) {
      mppi_ = MPPI(delta_t = 1. / rate_, horizon_step_T = mpc_horizon_steps_,
                   number_of_samples_K = mpc_rollouts_;
                   sigma = Eigen::Matrix2f(steer_noise_, 0.0, 0.0, acc_noise_),
                   stage_cost_weight = Eigen::Vector4f(
                       x_weight_, y_weight_, yaw_weight_, speed_weight_),
                   terminal_cost_weight = Eigen::Vector4f(
                       x_weight_, y_weight_, yaw_weight_, speed_weight_));
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
    Eigen::MatrixXf ctrl = std::get<0>(opt);
    Eigen::MatrixXf x_opt = std::get<1>(opt);
    Eigen::MatrixXf x_sampled = std::get<2>(opt);
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

  marker.points.resize(optimal_traj.rows());
  for (std::size_t i = 0; i < optimal_traj.rows(); i++) {
    marker.points[i].x = predicted_state_traj(i, 0);
    marker.points[i].y = predicted_state_traj(i, 1);
  }

  marker_arr.markers.push_back(marker);

  // 2- publish sampled trajectories
  for (std::size_t i = 0; i < sampled_traj_list.size(); i++) {
    const auto &sample = sampled_traj_list[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "mppi_path_marker";
    marker.id = i + 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.35;
    marker.frame_locked = true;

    marker.points.resize(sample.rows());
    for (std::size_t i = 0; i < sample.rows(); i++) {
      sample.points[i].x = predicted_state_traj(i, 0);
      sample.points[i].y = predicted_state_traj(i, 1);
    }
    marker_arr.markers.push_back(marker);
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
    tmp.row(i) = {path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                  tf::getYaw(path.poses[i].pose.orientation), MPPI_REF_SPEED};
  }

  // workaround for lack of heading from GPS path
  for (std::size_t i = 1; i < tmp.rows(); i++) {
    tmp(i, 2) =
        std::atan2(tmp(i, 1) - tmp(i - 1, 1), tmp(i, 0) - tmp(i - 1, 0));
  }

  // Decelerate and stop at end of Path
  tmp(tmp.size1() - 1, 3) = 0.0;
  return tmp;
}

Eigen::MatrixXf
MppiNode::obstacles_to_matrix(const vision_msgs::Detection3DArray &obs) const {
  Eigen::MatrixXf tmp;
  tmp.resize(obs.detections.size(), 3);
  for (std::size_t i = 0; i < obs.detections.size(); i++) {
    tmp.row(i) = {obs.detections[i].bbox.center.position.x,
                  obs.detections[i].bbox.center.position.y,
                  obs.detections[i].bbox.size.x};
  }
  return tmp;
}

} // namespace mppi
