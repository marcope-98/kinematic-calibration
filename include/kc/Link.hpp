#ifndef KC_LINK_HPP_
#define KC_LINK_HPP_

#include "kc/types.hpp"

namespace kc
{
  enum class LinkType
  {
    Prismatic,
    Revolute
  };

  constexpr static LinkType P = LinkType::Prismatic;
  constexpr static LinkType R = LinkType::Revolute;

  template<LinkType LT>
  struct Link
  {
    [[nodiscard]] static auto jacobian(const double a, const double alpha,
                                       const double d, const double theta,
                                       const double      q,
                                       const std::size_t index, const std::vector<TransformationMatrix> &transforms) -> TransformationMatrix
    {
      (void)d;
      TransformationMatrix out, pre = TransformationMatrix::Identity(), post = TransformationMatrix::Identity();
      for (std::size_t link = 0; link < index; ++link)
        pre *= transforms[link];
      for (std::size_t link{index + 1}; link < transforms.size(); ++link)
        post *= transforms[link];

      out.block(0, 0, 3, 1) = (pre * delta_a(theta, q) * post).block(0, 3, 3, 1);
      out.block(0, 1, 3, 1) = (pre * delta_alpha(alpha, theta, q) * post).block(0, 3, 3, 1);
      out.block(0, 2, 3, 1) = (pre * delta_d() * post).block(0, 3, 3, 1);
      out.block(0, 3, 3, 1) = (pre * delta_theta(a, alpha, theta, q) * post).block(0, 3, 3, 1);

      return out;
    }

    [[nodiscard]] static auto transform(const double a, const double alpha, const double d, const double theta, const double q) -> TransformationMatrix;
    [[nodiscard]] static auto transform(const double a, const double alpha, const double d, const double theta) -> TransformationMatrix
    {
      const double sin_alpha = std::sin(alpha), cos_alpha = std::cos(alpha);
      const double sin_theta = std::sin(theta), cos_theta = std::cos(theta);
      // clang-format off
      return TransformationMatrix{{cos_theta, -sin_theta * cos_alpha,  sin_theta * sin_alpha, a * cos_theta},
                                  {sin_theta,  cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta},
                                  {        0,              sin_alpha,              cos_alpha,             d},
                                  {        0,                      0,                      0,             1}};
      // clang-format on
    }

    [[nodiscard]] static auto delta_a(const double theta, const double q) -> TransformationMatrix;
    [[nodiscard]] static auto delta_a(const double theta) -> TransformationMatrix
    {
      // clang-format off
      return TransformationMatrix{{0, 0, 0, std::cos(theta)},
                                  {0, 0, 0, std::sin(theta)},
                                  {0, 0, 0,               0},
                                  {0, 0, 0,               0}};
      // clang-format on
    }

    [[nodiscard]] static auto delta_alpha(const double alpha, const double theta, const double q) -> TransformationMatrix;
    [[nodiscard]] static auto delta_alpha(const double alpha, const double theta) -> TransformationMatrix
    {
      const double sin_alpha = std::sin(alpha), cos_alpha = std::cos(alpha);
      const double sin_theta = std::sin(theta), cos_theta = std::cos(theta);
      // clang-format off
      return TransformationMatrix{{0,  sin_theta * sin_alpha,  sin_theta * cos_alpha, 0},
                                  {0, -cos_theta * sin_alpha, -cos_theta * cos_alpha, 0},
                                  {0,              cos_alpha,             -sin_alpha, 0},
                                  {0,                      0,                      0, 0}};
      // clang-format on
    }

    [[nodiscard]] static auto delta_d() -> TransformationMatrix
    {
      return TransformationMatrix{{0, 0, 0, 0},
                                  {0, 0, 0, 0},
                                  {0, 0, 0, 1},
                                  {0, 0, 0, 0}};
    }

    [[nodiscard]] static auto delta_theta(const double a, const double alpha, const double theta, const double q) -> TransformationMatrix;
    [[nodiscard]] static auto delta_theta(const double a, const double alpha, const double theta) -> TransformationMatrix
    {
      const double sin_alpha = std::sin(alpha), cos_alpha = std::cos(alpha);
      const double sin_theta = std::sin(theta), cos_theta = std::cos(theta);
      // clang-format off
      return TransformationMatrix{{-sin_theta, -cos_theta * cos_alpha, cos_theta * sin_alpha, -a * sin_theta},
                                  { cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha,  a * cos_theta},
                                  {         0,                      0,                     0,              0},
                                  {         0,                      0,                     0,              0}};
      // clang-format on
    }
  };

  using LP = Link<LinkType::Prismatic>;
  using LR = Link<LinkType::Revolute>;

  // Specialization for Prismatic Link
  template<>
  [[nodiscard]] auto LP::transform(const double a, const double alpha, const double d, const double theta, const double q) -> TransformationMatrix
  {
    return LP::transform(a, alpha, d + q, theta);
  }

  template<>
  [[nodiscard]] auto LP::delta_a(const double theta, const double q) -> TransformationMatrix
  {
    (void)q;
    return LP::delta_a(theta);
  }

  template<>
  [[nodiscard]] auto LP::delta_alpha(const double alpha, const double theta, const double q) -> TransformationMatrix
  {
    (void)q;
    return LP::delta_alpha(alpha, theta);
  }

  template<>
  [[nodiscard]] auto LP::delta_theta(const double a, const double alpha, const double theta, const double q) -> TransformationMatrix
  {
    (void)q;
    return LP::delta_theta(a, alpha, theta);
  }

  // Specialization for Revolute Link
  template<>
  [[nodiscard]] auto LR::transform(const double a, const double alpha, const double d, const double theta, const double q) -> TransformationMatrix
  {
    return LR::transform(a, alpha, d, theta + q);
  }

  template<>
  [[nodiscard]] auto LR::delta_a(const double theta, const double q) -> TransformationMatrix
  {
    return LR::delta_a(theta + q);
  }

  template<>
  [[nodiscard]] auto LR::delta_alpha(const double alpha, const double theta, const double q) -> TransformationMatrix
  {
    return LR::delta_alpha(alpha, theta + q);
  }

  template<>
  [[nodiscard]] auto LR::delta_theta(const double a, const double alpha, const double theta, const double q) -> TransformationMatrix
  {
    return LR::delta_theta(a, alpha, theta + q);
  }
} // namespace kc
#endif // KC_LINK_HPP_