#ifndef KC_COSTFUNCTION_HPP_
#define KC_COSTFUNCTION_HPP_

#include "kc/Robot.hpp"

#include "ceres/ceres.h"

namespace kc
{
  template<std::size_t N>
  struct CostFunction : public ceres::SizedCostFunction<3, N, N, N, N>
  {
    CostFunction(const typename Robot<N>::JointAngles &x, const PositionVector &y) : x_{x}, y_{y} {}
    virtual ~CostFunction() {}
    virtual bool Evaluate(double const *const *parameters,
                          double *             residuals,
                          double **            jacobians) const
    {
      PositionVector xyz                    = Robot<N>::fk(parameters[0], parameters[1], parameters[2], parameters[3], this->x_);
      residuals[0]                          = xyz.x() - this->y_.x();
      residuals[1]                          = xyz.y() - this->y_.y();
      residuals[2]                          = xyz.z() - this->y_.z();
      typename Robot<N>::JacobianMatrix jac = Robot<N>::jacobian(parameters[0], parameters[1], parameters[2], parameters[3], this->x_);

      if (jacobians == nullptr) return true;

      for (std::size_t i{}; i < 4; ++i)
      {
        std::size_t counter{};
        if (jacobians[i] != nullptr)
          for (std::size_t row{}; row < 3; ++row)
            for (std::size_t col{i}; col < N * 4; col += 4)
              jacobians[i][counter++] = jac(row, col);
      }
      return true;
    }

    static auto create(const typename Robot<N>::JointAngles &x, const PositionVector &y) -> ceres::CostFunction * { return new CostFunction<N>(x, y); }

  private:
    typename Robot<N>::JointAngles x_;
    PositionVector                 y_;
  };
}
  #endif