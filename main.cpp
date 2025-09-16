#include <iostream>

#include "kc/Robot.hpp"
#include "kc/CostFunction.hpp"
#include "kc/types.hpp"
#include "kc/utils.hpp"

#include "ceres/ceres.h"

int main(void)
{
  using KUKA = kc::Robot<7>;

  // Read data
  constexpr std::size_t training_set = 50800;
  const auto            xyz          = kc::read_xyz<kc::PositionVector>("../data/P_KUKA.txt");
  const auto            joint_angles = kc::read_xyz<KUKA::JointAngles>("../data/Q_KUKA.txt");

  // Define initial parameter blocks values
  // clang-format off
  double         a[]     = {      0,      0,      0,      0,      0,      0,     0 };
  double         alpha[] = { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2,     0 };
  double         d[]     = {   0.34,      0,    0.4,      0,    0.4,      0, 0.126 };
  double         theta[] = {   M_PI,   M_PI,      0,   M_PI,      0,   M_PI,     0 };
  // clang-format on

  // Problem definition
  ceres::Problem problem;
  for (std::size_t i{}; i < training_set; ++i)
    problem.AddResidualBlock(
        kc::CostFunction<KUKA::value>::create(joint_angles[i], xyz[i]),
        nullptr,
        a, alpha, d, theta);

  // Wrap angles between -pi and pi
  ceres::Manifold *angle_manifold = new ceres::EuclideanManifold<7>();
  problem.SetManifold(alpha, angle_manifold);
  problem.SetManifold(theta, angle_manifold);

  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  // std::cout << summary.FullReport() << "\n";
  std::cout << "\n\n";

  // Summary of DH Parameters
  kc::report<KUKA::value>(a, alpha, d, theta);

  // Basic statistics on validation set
  double rmse{};
  double L = double(xyz.size() - training_set);
  for (std::size_t i{training_set}; i < xyz.size(); ++i)
  {
    kc::PositionVector tmp = KUKA::fk(a, alpha, d, theta, joint_angles[i]);
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
