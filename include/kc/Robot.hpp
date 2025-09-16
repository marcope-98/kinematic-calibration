#ifndef KC_ROBOT_HPP_
#define KC_ROBOT_HPP_


#include "kc/Link.hpp"
#include "kc/types.hpp"

#include "Eigen/Dense"

namespace kc
{
  template<std::size_t N>
  struct Robot
  {
    static constexpr std::size_t value = N;

    using JointAngles       = Vector<N>;
    using DHParameterVector = Vector<N>;
    using JacobianMatrix    = Eigen::Matrix<double, 3, 4 * N>;

    static auto fk(const DHParameterVector &a,
                   const DHParameterVector &alpha,
                   const DHParameterVector &d,
                   const DHParameterVector &theta,
                   const JointAngles &      q) -> PositionVector;
    static auto jacobian(const DHParameterVector &a,
                         const DHParameterVector &alpha,
                         const DHParameterVector &d,
                         const DHParameterVector &theta,
                         const JointAngles &      q) -> JacobianMatrix;
  };

  template<std::size_t N>
  inline auto Robot<N>::fk(const DHParameterVector &a, const DHParameterVector &alpha, const DHParameterVector &d, const DHParameterVector &theta, const JointAngles &q) -> PositionVector
  {
    TransformationMatrix out = TransformationMatrix::Identity();
    for (std::size_t link{}; link < N; ++link)
      out *= Link::transform(a[link], alpha[link], d[link], theta[link] + q[link]);
    return out.block(0, 3, 3, 1);
  }

  template<std::size_t N>
  inline auto Robot<N>::jacobian(const DHParameterVector &a, const DHParameterVector &alpha, const DHParameterVector &d, const DHParameterVector &theta, const JointAngles &q) -> JacobianMatrix
  {
    JacobianMatrix       out = JacobianMatrix::Zero();
    TransformationMatrix pre{}, post{};

    std::vector<TransformationMatrix> transforms;
    transforms.reserve(N);
    for (std::size_t link{}; link < N; ++link)
      transforms.push_back(Link::transform(a[link], alpha[link], d[link], theta[link] + q[link]));

    pre = TransformationMatrix::Identity();
    for (std::size_t link{}; link < N; ++link)
    {
      if (link != 0) pre *= transforms[link - 1];

      post = TransformationMatrix::Identity();
      for (std::size_t i{link + 1}; i < N; ++i)
        post *= transforms[i];

      out.block(0, link * 4 + 0, 3, 1) = (pre * Link::delta_a(theta[link]) * post).block(0, 3, 3, 1);
      out.block(0, link * 4 + 1, 3, 1) = (pre * Link::delta_alpha(alpha[link], theta[link]) * post).block(0, 3, 3, 1);
      out.block(0, link * 4 + 2, 3, 1) = (pre * Link::delta_d() * post).block(0, 3, 3, 1);
      out.block(0, link * 4 + 3, 3, 1) = (pre * Link::delta_theta(a[link], alpha[link], theta[link]) * post).block(0, 3, 3, 1);
    }
    return out;
  }

} // namespace kc

#endif // KC_ROBOT_HPP_