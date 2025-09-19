#include <iostream>

#include "Stanford.hpp"

#include "kc/CostFunction.hpp"
#include "kc/Robot.hpp"
#include "kc/types.hpp"
#include "kc/utils.hpp"

#include "ceres/ceres.h"

int main(void)
{
  using Stanford = kc::Robot<kc::LR, kc::LR, kc::LP,
                             kc::LR, kc::LR, kc::LR>;

  // Read data
  const auto  xyz          = kc::read_xyz<kc::PositionVector>(P_Stanford);
  const auto  joint_angles = kc::read_xyz<Stanford::JointAngles>(Q_Stanford);
  std::size_t training_set = std::size_t((double)xyz.size() * 0.8);

  // Define initial parameter blocks values
  // clang-format off
  double         a[]     = {        0.1,       0.05,  0,       0.05,        0.7,    0 };
  double         alpha[] = { 3 * M_PI_2, 3 * M_PI_2,  0,     M_PI_2, 3 * M_PI_2,    0 };
  double         d[]     = {        0.9,       1.35,  0,        0.3,          0, 0.05 };
  double         theta[] = { 3 * M_PI_2,       M_PI,  0, 3 * M_PI_2, 3 * M_PI_2,    0 };
  // clang-format on
  // Problem definition
  ceres::Problem problem;
  for (std::size_t i{}; i < training_set; ++i)
    problem.AddResidualBlock(
        kc::CostFunction<Stanford>::create(joint_angles[i], xyz[i]),
        nullptr,
        a, alpha, d, theta);

  // Wrap angles between 0 and 2*pi
  for (std::size_t i{}; i < Stanford::N; ++i)
  {
    problem.SetParameterLowerBound(a, i, 0.);
    problem.SetParameterLowerBound(alpha, i, 0.);
    problem.SetParameterLowerBound(d, i, 0.);
    problem.SetParameterLowerBound(theta, i, 0.);
    problem.SetParameterUpperBound(alpha, i, 2.0 * ceres::constants::pi);
    problem.SetParameterUpperBound(theta, i, 2.0 * ceres::constants::pi);
  }

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations           = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  std::cout << "\n\n";

  // Summary of DH Parameters
  kc::report<Stanford::N>(a, alpha, d, theta);

  // Basic statistics on validation set
  double rmse{};
  double L = double(xyz.size() - training_set);
  for (std::size_t i{training_set}; i < xyz.size(); ++i)
  {
    kc::PositionVector tmp = Stanford::fk(a, alpha, d, theta, joint_angles[i]);
    kc::PositionVector error;
    error.x()   = tmp.x() - xyz[i].x();
    error.y()   = tmp.y() - xyz[i].y();
    error.z()   = tmp.z() - xyz[i].z();
    double norm = error.norm();
    rmse += (norm * norm) / L;
  }
  std::cerr << "Mean Squared Root: " << sqrt(rmse) << "\n";
  return 0;
}
