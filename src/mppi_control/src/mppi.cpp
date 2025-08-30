#include "mppi_control/mppi.hpp"
#include <stdexcept>
#include <unsupported/Eigen/Splines>

namespace mppi {

KinematicMpc::KinematicMpc(const KinematicModel &m, const MpcParameters &p,
                           const casadi::DM &trajectory,
                           const casadi::DM &obstacles) {
}


// TODO: test this standalone...
Eigen::MatrixXf
MPPI::reinterpolate_reference_trajectory(const Eigen::MatrixXf  &traj,
                                                 const Eigen::Vector4d  &x) const {
  using Spline1D = Eigen::Spline<float, 1, 2> ;
  using  SplineFitting1D = Eigen::SplineFitting<Spline1D>;

  Eigen::Matrix<float, N_, nx_> waypoints ;

  // Find the index of the closest trajectory point to the vehicle.
  std::vector<float> distances(traj.rows());
  for (std::size_t i = 0; i < traj.rows(); i++) {
    distances[i] =
        std::hypot(x(0) - traj(i, 0), x(1) - traj(i, 1));
  }

  auto min_element = std::min_element(distances.begin(), distances.end());
  std::size_t closest_idx = std::distance(distances.begin(), min_element);

  // find target states by interpolating along trajectory length.
  // compute first the distance along the trajectory for each traj point
  // these will be the interpolation knot points
  Eigen::RowVectorXf cdist(traj.rows());
  cdist(0) = 0.0;
  for (std::size_t i = 1; i < traj.rows(); i++) {
    cdist(i) =
        cdist(i - 1) + std::hypot(traj(i, 0) - traj(i - 1, 0),
                                  traj(i, 1) - traj(i - 1, 1));
  }

  auto start_dist = cdist[closest_idx];

  // interpolate the trajectory at these points
  // NOTE: the interpolation points are equally
  // spaced given the average speed
  float v = traj.col(3).norm();
  v /= traj.col(3).size();

  Eigen::VectorXf intp_pts(N_);
  for (std::size_t i = 0; i < N_; i++) {
    intp_pts(i) =
        std::clamp(start_dist + (i + 1) * v * dt_, cdist.head(1)[0], cdist.tail(1)[0]);
  }

  const auto fit_x = SplineFitting1D::Interpolate(traj.col(0).transpose(), 2, cdist);
  Spline1D x_intp(fit_x);

  const auto fit_y = SplineFitting1D::Interpolate(traj.col(1).transpose(), 2, cdist);
  Spline1D y_intp(fit_y);
  
  const auto fit_theta = SplineFitting1D::Interpolate(traj.col(2).transpose(), 2, cdist);
  Spline1D t_intp(fit_theta);
  
  const auto fit_v = SplineFitting1D::Interpolate(traj.col(3).transpose(), 2, cdist);
  Spline1D v_intp(fit_v);

  std::cout<<"ok so far?"<<std::endl;
  
  // interpolate at target points
  for (std::size_t i = 0; i < N_; i++) {
    waypoints(i, 0) = x_intp(intp_pts(i)).coeff(0);;
    waypoints(i, 1) = y_intp(intp_pts(i)).coeff(0);;
    waypoints(i, 2) = t_intp(intp_pts(i)).coeff(0);;
    waypoints(i, 3) = v_intp(intp_pts(i)).coeff(0);;
  }

  // NOTE: equivalent of MATLAB unwrap, removes jumps from heading
  auto unwrap = [](float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;
  };

  waypoints(0, 2) = unwrap(x(2), waypoints(0, 2));
  for (std::size_t i = 1; i < N_; i++) {
    waypoints(i, 2) =
        unwrap(waypoints(i - 1, 2), waypoints(i, 2));
  }

  return waypoints;
}

} // namespace mppi
