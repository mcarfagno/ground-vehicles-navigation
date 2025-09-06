#ifndef MPPI_CONTROL__MPPI_NODE_HPP_
#define MPPI_CONTROL__MPPI_NODE_HPP_

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Point.h>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <utility>
#include <vector>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "mppi_control/mppi.hpp"
#include "mppi_control/utils.hpp"
namespace mppi {

// World origin in Lat/Lon
// TODO: this should be a rosparam
static const float GPS_WORLD_ORIGIN_LAT = 40.09302492080515;
static const float GPS_WORLD_ORIGIN_LON = -88.2357551253083;
static const double MPPI_REF_SPEED = 20 / 3.6; // [m/s] -> 20km/h

class MppiNode {
public:
  MppiNode();
  ~MppiNode() {};
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher cmd_pub_;
  ros::Publisher viz_pub_;

  ros::Subscriber odom_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber obstacles_sub_;

  // TODO: redo these ?
  int mpc_horizon_steps_;
  int mpc_rollouts_;
  float rate_;
  float obs_safety_dist_;
  float x_weight_;
  float y_weight_;
  float yaw_weight_;
  float speed_weight_;
  float steer_noise_;
  float acc_noise_;

  std::optional<MPPI> mppi_;

  std::optional<nav_msgs::Odometry> latest_odom_;
  std::optional<nav_msgs::Path> path_;
  std::optional<vision_msgs::Detection3DArray> obstacles_;

  Eigen::Vector4f odometry_to_matrix(const nav_msgs::Odometry &odom) const;
  Eigen::MatrixXf
  obstacles_to_matrix(const vision_msgs::Detection3DArray &obs) const;
Eigen::MatrixXf path_to_matrix(const nav_msgs::Path &path) const;

  void publish_mpc_cmd(double speed, double steer);
void publish_rviz_markers(
    const Eigen::MatrixXf &optimal_traj,
    const std::vector<Eigen::MatrixXf> sampled_traj_list);
};

} // namespace mppi

std::pair<double, double> latlon_to_XY(double lat0, double lon0, double lat1,
                                       double lon1);
#endif
