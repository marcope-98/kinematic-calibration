#include "kc/Link.hpp"

#include <cmath>

auto kc::Link::transform(const double a, const double alpha, const double d, const double theta) -> TransformationMatrix
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

auto kc::Link::delta_a(const double theta) -> TransformationMatrix
{
  // clang-format off
  return TransformationMatrix{{0, 0, 0, std::cos(theta)},
                              {0, 0, 0, std::sin(theta)},
                              {0, 0, 0,               0},
                              {0, 0, 0,               0}};
  // clang-format on
}

auto kc::Link::delta_alpha(const double alpha, const double theta) -> TransformationMatrix
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
auto kc::Link::delta_d() -> TransformationMatrix
{
  return TransformationMatrix{{0, 0, 0, 0},
                              {0, 0, 0, 0},
                              {0, 0, 0, 1},
                              {0, 0, 0, 0}};
}
auto kc::Link::delta_theta(const double a, const double alpha, const double theta) -> TransformationMatrix
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
