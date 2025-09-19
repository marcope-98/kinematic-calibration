#ifndef KC_COSTFUNCTION_HPP_
#define KC_COSTFUNCTION_HPP_

#include "kc/Robot.hpp"

#include "ceres/ceres.h"

namespace kc
{
  template<class Robot>
  struct CostFunction : public ceres::SizedCostFunction<3, Robot::N, Robot::N, Robot::N, Robot::N>
  {
    CostFunction(const typename Robot::JointAngles &x, const PositionVector &y) : x_{x}, y_{y} {}
    virtual ~CostFunction() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *             residuals,
                          double **            jacobians) const
    {
      PositionVector xyz = Robot::fk(parameters[0], parameters[1], parameters[2], parameters[3], this->x_);
      residuals[0]       = xyz.x() - this->y_.x();
      residuals[1]       = xyz.y() - this->y_.y();
      residuals[2]       = xyz.z() - this->y_.z();

      if (jacobians == nullptr) return true;

      typename Robot::JacobianMatrix jac = Robot::jacobian(parameters[0], parameters[1], parameters[2], parameters[3], this->x_);
      for (std::size_t i{}; i < 4; ++i)
      {
        std::size_t counter{};
        if (jacobians[i] != nullptr)
          for (std::size_t row{}; row < 3; ++row)
            for (std::size_t col{i}; col < Robot::N * 4; col += 4)
              jacobians[i][counter++] = jac(row, col);
      }
      return true;
    }

    static auto create(const typename Robot::JointAngles &x, const PositionVector &y) -> ceres::CostFunction * { return new CostFunction<Robot>(x, y); }

  private:
    typename Robot::JointAngles x_;
    PositionVector              y_;
  };
} // namespace kc
#endif