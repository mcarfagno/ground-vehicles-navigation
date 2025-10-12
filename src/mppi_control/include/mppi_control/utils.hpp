#ifndef MPPI_CONTROLLER__UTILS_HPP_
#define MPPI_CONTROLLER__UTILS_HPP_

#include <Eigen/Dense>
#include <random>
#include <utility>

static Eigen::ArrayXXf sample_noise(float stddev, std::size_t batch_size,
                             std::size_t time_steps) {
  std::default_random_engine generator_;
  generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());

  std::normal_distribution<float> ndistribution =
      std::normal_distribution(0.0f, stddev);
  return Eigen::ArrayXXf::NullaryExpr(
      batch_size, time_steps, [&]() { return ndistribution(generator_); });
}

#endif
