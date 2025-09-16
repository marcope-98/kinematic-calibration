#ifndef KC_TYPES_HPP_
#define KC_TYPES_HPP_

#include <fstream>

#include "Eigen/Dense"

namespace kc
{
  template<std::size_t N>
  struct Vector : public Eigen::Vector<double, N>
  {
    using Eigen::Vector<double, N>::Vector; // Inherits Eigen::Vector Constructors
    
    Vector(const double *const data) : Eigen::Vector<double, N>{data} {}

    friend auto operator>>(std::ifstream &istrm, Vector &vector) -> std::ifstream &
    {
      for (std::size_t i{}; i < N; ++i)
        istrm >> vector[i];
      return istrm;
    }
  };

  using PositionVector       = Vector<3>;
  using TransformationMatrix = Eigen::Matrix<double, 4, 4>;
} // namespace kc

#endif // KC_TYPES_HPP_
