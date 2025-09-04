#include <stdexcept>
#include <unsupported/Eigen/Splines>

#include "mppi_control/mppi.hpp"
namespace mppi {

std::tuple<MppiCmd, MatrixXf, std::vector<MatrixXf>>
MPPI::compute_optimal_input(const MatrixXf &trajectory, const Vector4f &x0) {

  // nominal control sequence
  ArrayXXf u = prev_u_;

  // N points -> 1 for each horizon step
  const MatrixXf reference = reinterpolate_reference_trajectory(trajectory, x0);

  // buffer for rollout costs
  ArrayXf S = ArrayXf::Zero(K_);
  std::vector<ArrayXXf> epsilon_buff(K_);
  std::vector<MatrixXf> sampled_buff(K_);

  // loop for 0 ~ K-1 samples
  for (std::size_t k = 0; k < K_; k++) {
    // start state of this rollout
    auto x = x0;

    // sample disturbance vector
    const auto epsilon = compute_epsilon_();
    epsilon_buff[k] = epsilon;

    // buffer for sampled control sequence
    MatrixXf v = MatrixXf::Zero(u.rows(), u.cols());
    MatrixXf sampled_trajectory = MatrixXf::Zero(T_, dim_x_);

    // loop for time step t = 1 ~ T
    for (std::size_t t = 1; t < T_ + 1; t++) {

      // noisy control input for this step
      if (k < (1.0 - param_exploration_) * K_) {
        // exploit
        v.row(t - 1) = u.row(t - 1) + epsilon.row(t - 1);
      } else {
        // explore
        v.row(t - 1) = epsilon.row(t - 1);
      }

      // update x
      x = F_(x, g_(v.row(t - 1)));
      sampled_trajectory.row(t - 1) = x;

      // accumulate stage cost
      S(k) += c_(x, reference.row(t - 1)) + param_gamma_ * u.row(t - 1) *
                                                sigma_.inverse() *
                                                v.row(t - 1).transpose();
    }

    // terminal cost
    S(k) += phi_(x, reference.row(T_ - 1));
    sampled_buff[k] = sampled_trajectory;
  }

  // compute information theoretic weights for each sample
  VectorXf w = compute_weights_(S);

  // update control input sequence
  ArrayXXf w_epsilon = ArrayXXf::Zero(T_, dim_u_);
  for (std::size_t k = 0; k < K_; k++) {
    w_epsilon += w(k) * epsilon_buff[k];
  }
  u = u + w_epsilon;

  // set up for next iteration
  u_prev_ = u;

  // shift inputs by 1 timestep
  u_prev_.block(0, 0, u_prev_.rows() - 1, u_prev_.cols()) =
      u_prev_.block(1, 0, u_prev_.rows() - 1, u_prev_.cols());

  // calculate optimal trajectory
  MatrixXf optimal_trajectory = MatrixXf::Zero(T_, dim_x_);
  Vector4f x = x0;
  for (std::size_t t = 0; t < T_; t++) {
    x = F_(x, g_(u.row(t)));
    optimal_trajectory.row(t) = x;
  }

  return std::make_tuple(std::make_pair(u(0, 0), u(0, 1)), optimal_trajectory,
                         sampled_buff);
}

Vector4f MPPI::F_(const Vector4f &x_t, const Vector2f &u_t) const {
  const auto x = x_t(0);
  const auto y = x_t(1);
  const auto yaw = x_t(2);
  const auto v = x_t(3);

  const auto steer = u_t(0);
  const auto accel = u_t(1);

  return Vector4f(x + v * std::cos(yaw) * dt_, y + v * std::sin(yaw) * dt_,
                  yaw + v / wheel_base_ * std::tan(steer) * dt_,
                  v + accel * dt_);
}

Vector2f MPPI::g_(const Vector2f &u_t) const {
  return Vector2f(std::clamp(u_t(0), -steer_max_abs_, steer_max_abs_),
                  std::clamp(u_t(1), -a_max_abs_, a_max_abs_));
}

float MPPI::c_(const Vector4f &x_t, const Vector4f &x_ref) const {

  // Compute the cost
  Vector4f x_err = x_t - x_ref;
  float stage_cost =
      x_err.transpose() * stage_cost_weight_.asDiagonal() * x_err;

  // TODO add penalty for collision with obstacles
  return stage_cost;
}

float MPPI::phi_(const Vector4f &x_t, const Vector4f &x_ref) const {

  // Compute the cost
  Vector4f x_err = x_t - x_ref;
  float stage_cost =
      x_err.transpose() * stage_cost_weight_.asDiagonal() * x_err;

  // TODO add penalty for collision with obstacles
  return stage_cost;
}

ArrayXXf MPPI::compute_epsilon_() const {
  ArrayXXf epsilon = ArrayXXf::Zero(T_, dim_u_);
  normal_random_variable sample{sigma_};

  for (std::size_t i = 0; i < epsilon.rows(); i++) {
    epsilon.row(i) = sample();
  }

  return epsilon;
}

VectorXf MPPI::compute_weights_(const ArrayXf &S) const {
  const float rho = S.minCoeff();
  const float eta = (-1.0 / param_lambda_ * (S - rho)).exp().sum();

  VectorXf w = VectorXf::Zero(K_);
  w = (1.0 / eta) * ((-1.0 / param_lambda_) * (S - rho)).exp();
  return w;
}

MatrixXf MPPI::reinterpolate_reference_trajectory(const MatrixXf &traj,
                                                  const Vector4f &x) const {
  using Spline1D = Eigen::Spline<float, 1, 2>;
  using SplineFitting1D = Eigen::SplineFitting<Spline1D>;

  // find target states by interpolating along trajectory length.
  // compute first the distance along the trajectory for each traj point
  // these will be the interpolation knot points
  Eigen::RowVectorXf cdist(traj.rows());
  cdist(0) = 0.0;
  for (std::size_t i = 1; i < traj.rows(); i++) {
    cdist(i) = cdist(i - 1) + std::hypot(traj(i, 0) - traj(i - 1, 0),
                                         traj(i, 1) - traj(i - 1, 1));
  }

  // Find the index of the closest trajectory point to the vehicle.
  std::vector<float> distances(traj.rows());
  for (std::size_t i = 0; i < traj.rows(); i++) {
    distances[i] = std::hypot(x(0) - traj(i, 0), x(1) - traj(i, 1));
  }

  auto min_element = std::min_element(distances.begin(), distances.end());
  std::size_t closest_idx = std::distance(distances.begin(), min_element);
  auto start_dist = cdist[closest_idx];

  // interpolate the trajectory at these points
  // NOTE: the interpolation points are equally
  // spaced given the average speed
  const float v = traj.col(3).mean();

  VectorXf intp_pts(T_);
  for (std::size_t i = 0; i < T_; i++) {
    intp_pts(i) = std::clamp(start_dist + (i + 1) * v * dt_, cdist.head(1)[0],
                             cdist.tail(1)[0]);
  }

  const auto fit_x =
      SplineFitting1D::Interpolate(traj.col(0).transpose(), 2, cdist);
  Spline1D x_intp(fit_x);

  const auto fit_y =
      SplineFitting1D::Interpolate(traj.col(1).transpose(), 2, cdist);
  Spline1D y_intp(fit_y);

  const auto fit_theta =
      SplineFitting1D::Interpolate(traj.col(2).transpose(), 2, cdist);
  Spline1D t_intp(fit_theta);

  const auto fit_v =
      SplineFitting1D::Interpolate(traj.col(3).transpose(), 2, cdist);
  Spline1D v_intp(fit_v);

  // interpolate at target points
  Eigen::Matrix<float, T_, dim_x_> waypoints;
  for (std::size_t i = 0; i < waypoints.rows(); i++) {
    waypoints(i, 0) = x_intp(intp_pts(i)).coeff(0);
    waypoints(i, 1) = y_intp(intp_pts(i)).coeff(0);
    waypoints(i, 2) = t_intp(intp_pts(i)).coeff(0);
    waypoints(i, 3) = v_intp(intp_pts(i)).coeff(0);
  }

  // NOTE: equivalent of MATLAB unwrap, removes jumps from heading
  auto unwrap = [](float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;
  };

  waypoints(0, 2) = unwrap(x(2), waypoints(0, 2));
  for (std::size_t i = 1; i < waypoints.rows(); i++) {
    waypoints(i, 2) = unwrap(waypoints(i - 1, 2), waypoints(i, 2));
  }

  return waypoints;
}

} // namespace mppi
