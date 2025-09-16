#ifndef KC_LINK_HPP_
#define KC_LINK_HPP_

#include "kc/types.hpp"

namespace kc
{
  struct Link
  {
    [[nodiscard]] static auto transform(const double a, const double alpha, const double d, const double theta) -> TransformationMatrix;
    [[nodiscard]] static auto delta_a(const double theta) -> TransformationMatrix;
    [[nodiscard]] static auto delta_alpha(const double alpha, const double theta) -> TransformationMatrix;
    [[nodiscard]] static auto delta_d() -> TransformationMatrix;
    [[nodiscard]] static auto delta_theta(const double a, const double alpha, const double theta) -> TransformationMatrix;
  };
} // namespace kc
#endif // KC_LINK_HPP_