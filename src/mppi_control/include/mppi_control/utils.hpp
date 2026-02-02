#ifndef MPPI_CONTROLLER__UTILS_HPP_
#define MPPI_CONTROLLER__UTILS_HPP_

#include <Eigen/Dense>
#include <random>
#include <utility>

inline Eigen::ArrayXXf sample_noise(float stddev, std::size_t batch_size,
                             std::size_t time_steps) {
  // Use thread-local static generator to maintain state across calls
  static thread_local std::mt19937 generator(std::random_device{}());

  std::normal_distribution<float> dist(0.0f, stddev);

  return Eigen::ArrayXXf::NullaryExpr(
    batch_size, time_steps, [&]() {return dist(generator);});
}

#endif
