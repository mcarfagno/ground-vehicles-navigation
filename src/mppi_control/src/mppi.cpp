#include <stdexcept>
#include <cmath>

#include "mppi_control/mppi.hpp"
namespace mppi {

std::tuple<MppiCmd, MatrixXf, std::vector<MatrixXf>>
MPPI::compute_optimal_input(const MatrixXf &trajectory, const Vector4f &x0) {
  // N points -> 1 for each horizon step
  const Eigen::MatrixXf reference =
      reinterpolate_reference_trajectory(trajectory, x0);

  // sample noisy trajectories (K rollouts)
  // x holds -> sampled trajecories and noisy controls for all batches
  State x;
  x.reset(K_, T_);

  // sample disturbance vector epsilon (for each rollout)
  auto epsilon_steer = sample_noise(sigma_(0, 0), K_, T_);
  auto epsilon_a = sample_noise(sigma_(1, 1), K_, T_);

  // compute noisy imput v (for each rollout)
  x.a = epsilon_a.rowwise() + u_.a.transpose();
  x.steer = epsilon_steer.rowwise() + u_.steer.transpose();

  // set initials
  x.x.col(0) = x0(0);
  x.y.col(0) = x0(1);
  x.yaw.col(0) = x0(2);
  x.v.col(0) = x0(3);

  // predict trajectory (for each rollout)
  // loop for time step t = 1 ~ T
  for (std::size_t t = 1; t < T_; t++) {

    const auto x_t_minus = x.x.col(t - 1);
    const auto y_t_minus = x.y.col(t - 1);
    const auto yaw_t_minus = x.yaw.col(t - 1);
    const auto v_t_minus = x.v.col(t - 1);

    const auto steer_t =
        x.steer.col(t - 1).array().min(steer_max_abs_).max(-steer_max_abs_);
    const auto accel_t =
        x.a.col(t - 1).array().min(a_max_abs_).max(-a_max_abs_);

    x.x.col(t) = x_t_minus + v_t_minus * yaw_t_minus.cos() * dt_;
    x.y.col(t) = y_t_minus + v_t_minus * yaw_t_minus.sin() * dt_;
    x.yaw.col(t) = yaw_t_minus + v_t_minus / wheel_base_ * steer_t.tan() * dt_;
    x.v.col(t) = v_t_minus + accel_t * dt_;
  }

  Eigen::ArrayXf costs_;
  costs_.setZero(K_);

  // ============================================================
  // Frenet Frame Cost Computation
  // ============================================================
  // Transform position errors from global (Cartesian) frame to path-relative
  // (Frenet) frame. This separates cross-track error (perpendicular to path)
  // from along-track error (along path direction).
  //
  //          FRENET FRAME
  //          (along path, perpendicular to path)
  //
  //               ↑ perpendicular (cross-track)
  //               │
  //               │    ● Predicted
  //               │   ╱│
  //               │  ╱ │ e_cross
  //               │ ╱  │
  //          ─────●────┼───────────→ along path
  //                 e_along
  //
  // where:
  // along_track  = dx * cos(θ) + dy * sin(θ)     // Project onto path direction vector
  // cross_track  = -dx * sin(θ) + dy * cos(θ)    // Project onto perpendicular to path direction vector


  // Compute position errors in global frame
  auto dx = x.x.rowwise() - reference.col(0).transpose().array();
  auto dy = x.y.rowwise() - reference.col(1).transpose().array();

  // Transform to Frenet frame for each timestep
  Eigen::ArrayXXf cross_track_errors(K_, T_);
  Eigen::ArrayXXf along_track_errors(K_, T_);

  for (std::size_t t = 0; t < T_; t++) {
    float path_tangent = reference(t, 4);  // Path tangent angle
    float sin_theta = std::sin(path_tangent);
    float cos_theta = std::cos(path_tangent);

    // Rotate errors into path frame
    // Cross-track: perpendicular to path (lateral deviation)
    cross_track_errors.col(t) = -dx.col(t) * sin_theta + dy.col(t) * cos_theta;

    // Along-track: along path direction (longitudinal deviation)
    along_track_errors.col(t) = dx.col(t) * cos_theta + dy.col(t) * sin_theta;
  }

  // Compute heading error relative to path tangent
  Eigen::ArrayXXf heading_errors = x.yaw.array() - reference.col(4).array().transpose().replicate(K_, 1);

  // Normalize angles to [-pi, pi] to handle wrapping
  for (std::size_t k = 0; k < K_; k++) {
    for (std::size_t t = 0; t < T_; t++) {
      float error = heading_errors(k, t);
      // Wrap to [-pi, pi]
      while (error > M_PI) error -= 2.0f * M_PI;
      while (error < -M_PI) error += 2.0f * M_PI;
      heading_errors(k, t) = error;
    }
  }

  // Velocity tracking error
  auto v_errors = (x.v.rowwise() - reference.col(3).transpose().array()).square();

  // Time-discounting weights: exponentially decay influence of distant timesteps
  // This reduces end-of-horizon effects where predictions are less reliable
  // instead focuses on near-term accuracy (just like in RL!)
  Eigen::ArrayXf time_weights(T_);
  const float discount_factor = 0.98f;
  for (std::size_t t = 0; t < T_; t++) {
    time_weights(t) = std::pow(discount_factor, static_cast<float>(t));
  }

  // Compute weighted stage costs with time discounting
  costs_ = w_cross_track_ * (cross_track_errors.square().rowwise() * time_weights.transpose()).rowwise().sum() +
           w_along_track_ * (along_track_errors.square().rowwise() * time_weights.transpose()).rowwise().sum() +
           w_heading_ * (heading_errors.square().rowwise() * time_weights.transpose()).rowwise().sum() +
           w_velocity_ * (v_errors.rowwise() * time_weights.transpose()).rowwise().sum() -
           w_progress_ * (along_track_errors.rowwise() * time_weights.transpose()).rowwise().sum();  // Reward forward progress

  // NOTE:
  // along_track_errors² (quadratic penalty) -> Penalizes both directions equally: Being 2m ahead costs the same as 2m behind
  // progress = along_track_errors (linear reward) -> Breaks symmetry: Being ahead is rewarded, behind is penalized
  // keep both: synchronization (following a timed reference)
  // Remove squared term if timing doesn't matter (go as fast as possible along the path)

  auto bounded_noises_steer = x.steer.rowwise() - u_.steer.transpose();
  const float gamma_vx = param_gamma_ / (sigma_(0, 0) * sigma_(0, 0));
  costs_ +=
      (gamma_vx *
       (bounded_noises_steer.rowwise() * u_.steer.transpose()).rowwise().sum())
          .eval();

  auto bounded_noises_a = x.a.rowwise() - u_.a.transpose();
  const float gamma_va = param_gamma_ / (sigma_(1, 1) * sigma_(1, 1));
  costs_ += (gamma_va *
             (bounded_noises_a.rowwise() * u_.a.transpose()).rowwise().sum())
                .eval();

  auto costs_normalized = costs_ - costs_.minCoeff();
  const float inv_temp = 1.0f / param_lambda_;
  auto softmaxes = (-inv_temp * costs_normalized).exp().eval();
  softmaxes /= softmaxes.sum();

  // update
  auto softmax_mat = softmaxes.matrix();
  u_.a = x.a.transpose().matrix() * softmax_mat;
  u_.steer = x.steer.transpose().matrix() * softmax_mat;

  // clamp
  u_.steer = u_.steer.array().min(steer_max_abs_).max(-steer_max_abs_);
  u_.a = u_.a.array().min(a_max_abs_).max(-a_max_abs_);

  // smooth sampled ctrl
  apply_savitzky_golay_filter(u_.steer);
  apply_savitzky_golay_filter(u_.a);
 
  // calculate optimal trajectory
  Eigen::MatrixXf optimal_traj = Eigen::MatrixXf::Zero(T_, dim_x_);
  Eigen::Vector4f xn = x0;
  for (std::size_t t = 0; t < T_; t++) {
    auto ut = Eigen::Vector2f(u_.steer(t), u_.a(t));
    xn = F_(xn, g_(ut));
    optimal_traj.row(t) = xn;
  }

  // get cmd (steering, acceleration)
  auto next_cmd = MppiCmd(u_.steer(0), u_.a(0));

  // shift nominal control sequence by 1 timestep to the left, for next iter
  u_.steer(Eigen::seq(0, T_ - 2)) = u_.steer(Eigen::seq(1, T_ - 1)).eval();
  u_.a(Eigen::seq(0, T_ - 2)) = u_.a(Eigen::seq(1, T_ - 1)).eval();

  // Decay last control toward neutral to avoid end-of-horizon bias
  // This prevents steering from being artificially held at boundary
  u_.steer(T_ - 1) = u_.steer(T_ - 2) * 0.5f;
  u_.a(T_ - 1) = u_.a(T_ - 2) * 0.5f;

  // update ranking of costs
  // 1th: best (i.e. minimum cost), K: worst (i.e. maximum cost)
  std::vector<int> costs_rank_(K_);
  std::iota(costs_rank_.begin(), costs_rank_.end(),
            0); // initialize costs_rank_ with 0, 1, 2, ..., K-1
  std::sort(costs_rank_.begin(), costs_rank_.end(),
            [&](int i, int j) { return costs_[i] < costs_[j]; });

  // sort costs_rank_ based on score value
  // NOTE: best (minimum) cost is costs_[costs_rank_[0]], worst (maximum) cost
  // is costs_[costs_rank_[K-1]]
  auto samples_to_return = std::ceil(K_/10);
  std::vector<Eigen::MatrixXf> best_samples(samples_to_return);
  for (std::size_t i = 0; i < best_samples.size(); i++) {
    Eigen::MatrixXf xx;
    xx.setZero(T_,2);
    xx.col(0) = x.x.row(costs_rank_[i]);
    xx.col(1) = x.y.row(costs_rank_[i]);
    best_samples[i] = xx;
  }

  return std::make_tuple(next_cmd,optimal_traj, best_samples);
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

MatrixXf MPPI::reinterpolate_reference_trajectory(const MatrixXf &traj,
                                                  const Vector4f &x) const {
  Eigen::MatrixXf waypoints;
  waypoints.setZero(T_, 5);  // 5 columns: [x, y, yaw, v, path_tangent]

  // Find the index of the closest trajectory point to the vehicle.
  std::vector<float> distances(traj.rows());
  for (Eigen::Index i = 0; i < traj.rows(); i++) {
    distances[i] = std::hypot(x(0) - traj(i, 0), x(1) - traj(i, 1));
  }

  auto min_element = std::min_element(distances.begin(), distances.end());
  std::size_t closest_idx = std::distance(distances.begin(), min_element);

  // find target states by interpolating along trajectory length.
  // compute first the distance along the trajectory for each traj point
  // these will be the interpolation knot points
  Eigen::RowVectorXf cdist(traj.rows());
  cdist(0) = 0.0;
  for (Eigen::Index i = 1; i < traj.rows(); i++) {
    cdist(i) = cdist(i - 1) + std::hypot(traj(i, 0) - traj(i - 1, 0),
                                         traj(i, 1) - traj(i - 1, 1));
  }

  auto start_dist = cdist[closest_idx];
  float v_ref = traj.col(3).mean();

  Eigen::VectorXf intp_pts(T_);
  for (std::size_t i = 0; i < T_; i++) {
    intp_pts(i) = std::clamp(start_dist + (i + 1) * v_ref * dt_, cdist.head(1)[0],
                             cdist.tail(1)[0]);
  }

  // FINE I'LL DO IT MYSELF
  auto lerp = [](float a, float b, float f) {
    return (a * (1.0 - f)) + (b * f);
  };

  for (std::size_t t = 0; t < T_; t++) {
    // find index along cdist
    const auto it = std::find_if(cdist.begin(), cdist.end(),
                                 [&](float x) { return x > intp_pts(t); });
    const auto idx = std::distance(cdist.begin(), it);

    // edge cases to be addressed

    auto c1 = cdist[idx - 1];
    auto c2 = cdist[idx];
    // our point is somewhere along here
    auto ratio = (intp_pts(t) - c1) / (c2 - c1);

    // so we know wich trajectory points to interpolate
    for (std::size_t x = 0; x < dim_x_; x++) {
      auto intp = lerp(traj(idx - 1, x), traj(idx, x), ratio);
      waypoints(t, x) = intp;
    }
  }

  // NOTE: equivalent of MATLAB unwrap, removes jumps from heading
  auto unwrap = [](float previous_angle, float new_angle) {
    float d = new_angle - previous_angle;
    d = d > M_PI ? d - 2 * M_PI : (d < -M_PI ? d + 2 * M_PI : d);
    return previous_angle + d;
  };

  waypoints(0, 2) = unwrap(x(2), waypoints(0, 2));
  for (std::size_t i = 1; i < T_; i++) {
    waypoints(i, 2) = unwrap(waypoints(i - 1, 2), waypoints(i, 2));
  }

  // Compute path tangent angles (column 4) from consecutive waypoints
  // This gives the actual direction of the path at each point
  for (std::size_t t = 0; t < T_ - 1; t++) {
    waypoints(t, 4) = std::atan2(waypoints(t + 1, 1) - waypoints(t, 1),
                                  waypoints(t + 1, 0) - waypoints(t, 0));
  }
  // Extrapolate the last tangent angle using forward difference from last two points
  // This better captures path curvature than simple copy
  if (T_ >= 3) {
    // Use curvature trend from last three points
    float tangent_rate = waypoints(T_ - 2, 4) - waypoints(T_ - 3, 4);
    // Unwrap angle difference
    while (tangent_rate > M_PI) tangent_rate -= 2.0f * M_PI;
    while (tangent_rate < -M_PI) tangent_rate += 2.0f * M_PI;
    waypoints(T_ - 1, 4) = waypoints(T_ - 2, 4) + tangent_rate;
  } else {
    waypoints(T_ - 1, 4) = waypoints(T_ - 2, 4);
  }

  return waypoints;
}

} // namespace mppi
