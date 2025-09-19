#include <iostream>

#include "3R.hpp"

#include "kc/CostFunction.hpp"
#include "kc/Robot.hpp"
#include "kc/types.hpp"
#include "kc/utils.hpp"

#include "ceres/ceres.h"

int main(void)
{
  using threeR = kc::Robot<kc::LR, kc::LR, kc::LR>;

  // Read data
  const auto  xyz          = kc::read_xyz<kc::PositionVector>(P_3R);
  const auto  joint_angles = kc::read_xyz<threeR::JointAngles>(Q_3R);
  std::size_t training_set = xyz.size();

  // Define initial parameter blocks values
  // clang-format off
  double         a[]     = {    0.9,    0.4,    1.9 };
  double         alpha[] = {      0,      0,      0 };
  double         d[]     = {      0,      0,      0 };
  double         theta[] = {      0,      0,      0 };
  // clang-format on
  // Problem definition
  ceres::Problem problem;
  for (std::size_t i{}; i < training_set; ++i)
    problem.AddResidualBlock(
        kc::CostFunction<threeR>::create(joint_angles[i], xyz[i]),
        nullptr,
        a, alpha, d, theta);

  // Wrap angles between 0 and 2*pi
  problem.SetParameterBlockConstant(alpha);
  problem.SetParameterBlockConstant(d);
  for (std::size_t i{}; i < threeR::N; ++i)
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
  kc::report<threeR::N>(a, alpha, d, theta);

  // Basic statistics on validation set
  double rmse{};
  double L = double(xyz.size() - training_set);
  for (std::size_t i{training_set}; i < xyz.size(); ++i)
  {
    kc::PositionVector tmp = threeR::fk(a, alpha, d, theta, joint_angles[i]);
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
