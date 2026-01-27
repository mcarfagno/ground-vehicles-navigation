#include <stdexcept>

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

  // TODO: make this without nested loop
  //// accumulate stage cost
  // for (std::size_t k = 0; k < K_; k++) {
  //  for (std::size_t t = 1; t < T_; t++) {
  //    costs_(k) +=
  //        stage_cost_weight_[0] * std::pow((x.x(k, t) - reference(t, 0)), 2) +
  //        stage_cost_weight_[1] * std::pow((x.y(k, t) - reference(t, 1)), 2) +
  //        stage_cost_weight_[2] * std::pow((x.yaw(k, t) - reference(t, 2)), 2)
  //        + stage_cost_weight_[3] * std::pow((x.v(k, t) - reference(t, 3)),
  //        2);
  //  }
  //}

  // accumulate stage cost
  auto x_errors =
      stage_cost_weight_[0] *
      (x.x.rowwise() - reference.col(0).transpose().array()).square();
  auto y_errors =
      stage_cost_weight_[1] *
      (x.y.rowwise() - reference.col(1).transpose().array()).square();
  auto v_errors =
      stage_cost_weight_[3] *
      (x.v.rowwise() - reference.col(3).transpose().array()).square();
  auto yaw_errors =
      stage_cost_weight_[2] *
      (x.yaw.rowwise() - reference.col(2).transpose().array()).square();
  
  
  // TODO: add steer_rate to cost function
  // need correct u_steer_prev for 1st timestep
  //Eigen::ArrayXXf steer_diff = Eigen::ArrayXXf::Zero(K_, T_);
  //steer_diff.col(0) = x.steer.col(0) - u_.steer(0); 

  //for(int t=1; t<T_; ++t) {
  //    steer_diff.col(t) = x.steer.col(t) - x.steer.col(t-1);
  //}

  //auto steer_rate_costs = steer_rate_cost_weight * steer_diff.square();
  //costs_ += steer_rate_costs.rowwise().sum();

  costs_ += x_errors.rowwise().sum();
  costs_ += y_errors.rowwise().sum();
  costs_ += v_errors.rowwise().sum();
  costs_ += yaw_errors.rowwise().sum();

  // terminal state goal cost
  costs_ += 10.0f * ((x.x.col(T_ - 1) - reference(T_ - 1, 0)).square() +
                        (x.y.col(T_ - 1) - reference(T_ - 1, 1)).square())
                           .sqrt();

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

  // get cmd
  auto next_cmd = MppiCmd(u_.a(0), u_.steer(0));

  // shift nominal control sequence by 1 timestep to the left, for next iter
  u_.steer(Eigen::seq(0, T_ - 2)) = u_.steer(Eigen::seq(1, T_ - 1)).eval();
  u_.a(Eigen::seq(0, T_ - 2)) = u_.a(Eigen::seq(1, T_ - 1)).eval();

  // update ranking of costs
  // 1th: best (i.e. minimum cost), K: worst (i.e. maximum cost)
  const auto best_x = std::ceil(K_ / 20);
  std::vector<int> costs_rank_(K_);
  std::iota(costs_rank_.begin(), costs_rank_.end(),
            0); // initialize costs_rank_ with 0, 1, 2, ..., K-1
  std::sort(costs_rank_.begin(), costs_rank_.end(),
            [&](int i, int j) { return costs_[i] < costs_[j]; });

  // sort costs_rank_ based on score value
  // NOTE: best (minimum) cost is costs_[costs_rank_[0]], worst (maximum) cost
  // is costs_[costs_rank_[K-1]]
  std::vector<Eigen::MatrixXf> best_samples(best_x);
  for (std::size_t i = 0; i < best_x; i++) {
    Eigen::MatrixXf xx;
    xx.setZero(2, T_);
    xx.row(0) = x.x.row(costs_rank_[i]);
    xx.row(1) = x.y.row(costs_rank_[i]);
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
  waypoints.setZero(T_, 4);

  // Find the index of the closest trajectory point to the vehicle.
  std::vector<float> distances(traj.rows());
  for (std::size_t i = 0; i < traj.rows(); i++) {
    distances[i] = std::hypot(x(0) - traj(i, 0), x(1) - traj(i, 1));
  }

  auto min_element = std::min_element(distances.begin(), distances.end());
  std::size_t closest_idx = std::distance(distances.begin(), min_element);

  // find target states by interpolating along trajectory length.
  // compute first the distance along the trajectory for each traj point
  // these will be the interpolation knot points
  Eigen::RowVectorXf cdist(traj.rows());
  cdist(0) = 0.0;
  for (std::size_t i = 1; i < traj.rows(); i++) {
    cdist(i) = cdist(i - 1) + std::hypot(traj(i, 0) - traj(i - 1, 0),
                                         traj(i, 1) - traj(i - 1, 1));
  }

  auto start_dist = cdist[closest_idx];

  // TODO: make this work for reverse (negative velocities
  auto v = x(3);
  float v_ref = traj.col(3).mean();
  float a_max = (v < v_ref) ? a_max_abs_ : 0.0;

  Eigen::VectorXf intp_pts(T_);
  for (std::size_t i = 0; i < T_; i++) {
    v = std::clamp(v + a_max * dt_, -v_ref, v_ref);
    intp_pts(i) = std::clamp(start_dist + (i + 1) * v * dt_, cdist.head(1)[0],
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

  return waypoints;
}

} // namespace mppi
