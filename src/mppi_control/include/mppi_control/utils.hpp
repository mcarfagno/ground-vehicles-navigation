#ifndef MPPI_CONTROLLER__UTILS_HPP_
#define MPPI_CONTROLLER__UTILS_HPP_

#include <Eigen/Dense>
#include <random>

/* @brief used to sample a multivariate_normal
 * SOURCE:
 * https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
 * usage:
 * int size = 2;
 * Eigen::MatrixXd covar(size,size);
 *  covar << 1, .5,
 *         .5, 1;
 *
 * normal_random_variable sample { covar };
 *
 * std::cout << sample() << std::endl;
 * */
struct normal_random_variable {
  normal_random_variable(Eigen::MatrixXd const &covar)
      : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar) {}

  normal_random_variable(Eigen::VectorXd const &mean,
                         Eigen::MatrixXd const &covar)
      : mean(mean) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    transform = eigenSolver.eigenvectors() *
                eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd mean;
  Eigen::MatrixXd transform;

  Eigen::VectorXd operator()() const {
    static std::mt19937 gen{std::random_device{}()};
    static std::normal_distribution<> dist;

    return mean + transform * Eigen::VectorXd{mean.size()}.unaryExpr(
                                  [&](auto x) { return dist(gen); });
  }
};

#endif
