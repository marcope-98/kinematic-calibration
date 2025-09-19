#ifndef KC_ROBOT_HPP_
#define KC_ROBOT_HPP_

#include <utility>

#include "kc/Link.hpp"
#include "kc/types.hpp"

#include "Eigen/Dense"

namespace kc
{
  // Forward Declaration Robot
  template<class... Links>
  struct Robot;

  // Links Counter
  template<class V>
  constexpr std::size_t LinksCounter;

  template<class... Links>
  constexpr std::size_t LinksCounter<Robot<Links...>> = sizeof...(Links);

  // Robot
  template<class... Links>
  struct Robot
  {
    static constexpr std::size_t N = LinksCounter<Robot<Links...>>;

    using JointAngles    = Vector<N>;
    using JacobianMatrix = Eigen::Matrix<double, 3, 4 * N>;

    template<std::size_t... Is>
    static auto fk_internal(const double *const a, const double *const alpha,
                            const double *const d, const double *const theta,
                            const JointAngles &q, std::index_sequence<Is...>) -> PositionVector
    {
      TransformationMatrix out = (TransformationMatrix::Identity() *
                                  ... *
                                  Links::transform(a[Is], alpha[Is], d[Is], theta[Is], q[Is]));
      return out.block(0, 3, 3, 1);
    }

    static auto fk(const double *const a, const double *const alpha,
                   const double *const d, const double *const theta,
                   const JointAngles &q) -> PositionVector
    {
      return fk_internal(a, alpha, d, theta, q, std::index_sequence_for<Links...>{});
    }

    template<std::size_t... Is>
    static auto jacobian_internal(const double *const a, const double *const alpha,
                                  const double *const d, const double *const theta,
                                  const JointAngles &                      q,
                                  const std::vector<TransformationMatrix> &transforms,
                                  std::index_sequence<Is...>) -> std::vector<TransformationMatrix>
    {
      std::vector<TransformationMatrix> out;
      out.reserve(N);
      (out.push_back(Links::jacobian(a[Is], alpha[Is], d[Is], theta[Is], q[Is], Is, transforms)), ...);
      return out;
    }

    static auto jacobian(const double *const a, const double *const alpha,
                         const double *const d, const double *const theta,
                         const JointAngles &q) -> JacobianMatrix
    {
      const auto transforms = getTransforms(a, alpha, d, theta, q, std::index_sequence_for<Links...>{});
      const auto jacobians  = jacobian_internal(a, alpha, d, theta, q, transforms, std::index_sequence_for<Links...>{});

      JacobianMatrix out;
      for (std::size_t link{}; link < N; ++link)
        out.block(0, link * 4, 3, 4) = jacobians[link].block(0, 0, 3, 4);

      return out;
    }

    template<std::size_t... Is>
    static auto getTransforms(const double *const a, const double *const alpha,
                              const double *const d, const double *const theta,
                              const JointAngles &q, std::index_sequence<Is...>) -> std::vector<TransformationMatrix>
    {
      std::vector<TransformationMatrix> out;
      out.reserve(N);
      (out.push_back(Links::transform(a[Is], alpha[Is], d[Is], theta[Is], q[Is])), ...);
      return out;
    }
  };

} // namespace kc

#endif // KC_ROBOT_HPP_