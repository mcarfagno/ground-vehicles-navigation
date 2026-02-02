#ifndef MPPI_CONTROLLER__MPPI_HPP_
#define MPPI_CONTROLLER__MPPI_HPP_

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "mppi_control/utils.hpp"
namespace mppi {

/* commonly used eigen types
 * */
using Eigen::ArrayXf;
using Eigen::ArrayXXf;
using Eigen::Matrix2f;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector4f;
using Eigen::VectorXf;

/* @brief (steer,accel)
 * */
typedef std::pair<double, double> MppiCmd;

// NOTE: The default in Eigen is column-major.
// see here https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html
struct Control {
  Eigen::ArrayXf steer;
  Eigen::ArrayXf a;

  void reset(std::size_t T) {
    a.setZero(T);
    steer.setZero(T);
  }
};

/**
 * @brief Helper for Mppi State information: (batch_size, time_step)
 */
struct State {
  Eigen::ArrayXXf x;
  Eigen::ArrayXXf y;
  Eigen::ArrayXXf v;
  Eigen::ArrayXXf yaw;

  Eigen::ArrayXXf steer;
  Eigen::ArrayXXf a;

  void reset(std::size_t K, std::size_t T) {
    x.setZero(K, T);
    y.setZero(K, T);
    v.setZero(K, T);
    yaw.setZero(K, T);

    steer.setZero(K, T);
    a.setZero(K, T);
  }
};

/**
 * @brief Helper for Mppi sampled trajectories: (batch_size, time_step)
 */
struct Trajectories {
  Eigen::ArrayXXf x;
  Eigen::ArrayXXf y;
  Eigen::ArrayXXf yaw;
  Eigen::ArrayXXf v;

  void reset(std::size_t K, std::size_t T) {
    x.setZero(K, T);
    y.setZero(K, T);
    yaw.setZero(K, T);
    v.setZero(K, T);
  }
};

class MPPI {
public:
  explicit MPPI(
      const float delta_t = 0.05, const std::size_t horizon_step_T = 30,
      const std::size_t number_of_samples_K = 1000,
      const float param_lambda = 100.0,
      const float param_gamma = 0.0,
      const float steer_noise = 0.1,
      const float accel_noise = 1.0,
      const float cross_track_weight = 100.0,
      const float along_track_weight = 1.0,
      const float heading_weight = 10.0,
      const float velocity_weight = 20.0,
      const float progress_weight = 5.0,
      const float obstacle_margin = 0.25,
      const float obstacle_avoidance_weight = 5.0
      )
      : dt_(delta_t), T_(horizon_step_T), K_(number_of_samples_K),
        param_lambda_(param_lambda),
        param_gamma_(param_gamma),
        w_cross_track_(cross_track_weight),
        w_along_track_(along_track_weight),
        w_heading_(heading_weight),
        w_velocity_(velocity_weight),
        w_progress_(progress_weight),
        obstacle_margin_(obstacle_margin),
        obstacle_avoidance_weight_(obstacle_avoidance_weight) {

    // Initialize noise covariance matrix with provided values
    sigma_ << steer_noise, 0.0, 0.0, accel_noise;

    u_.reset(T_);
  }

  ~MPPI() {}

  /*
   * @brief resets the nominal control sequence
   * */
  void reset() {u_.reset(T_);}

  std::tuple<MppiCmd, MatrixXf, std::vector<MatrixXf>>
  compute_optimal_input(const MatrixXf &trajectory, const Vector4f &x0,
                       const MatrixXf &obstacles);

private:
  // mppi parameters
  const std::size_t dim_x_ = 4; // dimension of system state vector
  const std::size_t dim_u_ = 2; // dimension of control input vector

  float dt_;      // interval (s) between two sampled points in trajectories
  std::size_t T_; // prediction horizon or time steps in each sampled trajectory
  std::size_t K_; // number of rollouts or randomly sampled trajectories
  float param_exploration_; // constant parameter of mppi
  float param_lambda_;      // temperature, lambda -> 0 selects only best
                            // trajectories with low costs
  float
      param_gamma_; // smoothness parameter, this should be a small number < 0.1
  Matrix2f sigma_;  // noise covariance matrix

  // Frenet frame cost weights
  float w_cross_track_;  // weight for cross-track error (perpendicular to path)
  float w_along_track_;  // weight for along-track error (along path direction)
  float w_heading_;      // weight for heading error relative to path tangent
  float w_velocity_;     // weight for velocity tracking error
  float w_progress_;     // weight for progress reward (negative cost)

  // Obstacle avoidance parameters
  float obstacle_margin_;           // safety margin beyond obstacle radius [m]
  float obstacle_avoidance_weight_; // weight for obstacle avoidance cost

  Control u_; // nominal control sequence (prev iteration)

  // vehicle parameters
  float wheel_base_ = 1.75;        // [m]
  float vehicle_width_ = 1.2;      // [m]
  float v_min_ = 0.0;              // [m/s]
  float v_max_ = 10.0;             // [m/s]
  float a_max_abs_ = 3.0;          // [m/ss]
  float jerk_max_abs_ = 1.5;       // [m/sss]
  float steer_max_abs_ = 0.61;     // [rad]
  float steer_rate_max_abs_ = 0.5; // [rad/s]

  /**
   * @brief reinterpolates a trajectory to one of the correct
   * size and starting point
   * */
  MatrixXf reinterpolate_reference_trajectory(const MatrixXf &traj,
                                              const Vector4f &x) const;

  /**
   * @brief vehicle kinematics
   * */
  Vector4f F_(const Vector4f &x_t, const Vector2f &u_t) const;

  /**
   * @brief input clamp function
   * */
  Vector2f g_(const Vector2f &u_t) const;
};

/**
 * @brief utility function for simple 1D Savitzky-Golay filter
 * (Window size 5, Polynomial order 2)
 * */
template <typename Derived>
void apply_savitzky_golay_filter(Eigen::DenseBase<Derived>& u) {
  // DenseBase accepts both Matrices (Vectors) and Arrays
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);

  if (u.size() < 5) return;
  typename Derived::PlainObject u_copy = u;

  // Coefficients: [-3, 12, 17, 12, -3] / 35.0
  for (Eigen::Index i = 2; i < u.size() - 2; ++i) {
    u(i) = (-3.0f * u_copy(i - 2) + 
             12.0f * u_copy(i - 1) + 
             17.0f * u_copy(i) + 
             12.0f * u_copy(i + 1) - 
             3.0f * u_copy(i + 2)) / 35.0f;
  }

  u(0) = u_copy(0);
  u(1) = u_copy(1);
  u(u.size() - 2) = u_copy(u.size() - 2);
  u(u.size() - 1) = u_copy(u.size() - 1);
}

} // namespace mppi
#endif
