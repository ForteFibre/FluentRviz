#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>

#include <std_msgs/msg/color_rgba.hpp>

#include "fluent_rviz/traits/convert.hpp"
#include "fluent_rviz/traits/like_message.hpp"

namespace flrv::color
{
struct Color : public std_msgs::msg::ColorRGBA
{
  Color() = default;

  template <
    typename ColorLike,
    std::enable_if_t<traits::like_color<ColorLike>, std::nullptr_t> = nullptr>
  Color(const ColorLike &color)
    : std_msgs::msg::ColorRGBA{ traits::convert<std_msgs::msg::ColorRGBA>(color) }
  { }

  Color(const float r, const float g, const float b, const float a = 1.0) noexcept
  { this->r = r, this->g = g, this->b = b, this->a = a; }
};

[[nodiscard]]
inline auto RGBA(const float r, const float g, const float b, const float a = 1.0) noexcept -> Color
{ return { r, g, b, a }; }

[[nodiscard]]
inline auto HSLA(float h, float s, float l, float a = 1.0) noexcept -> Color
{
  h = std::fmod(h, 360.f);
  if (h < 0) {
    h += 360;
  }
  s = std::clamp(s, 0.f, 1.f);
  l = std::clamp(l, 0.f, 1.f);
  a = std::clamp(a, 0.f, 1.f);

  auto c = s * (1.f - std::abs(2.f * l - 1.f));
  auto x = c * (1.f - std::abs(std::fmod(h / 60.f, 2.f) - 1.f));
  auto m = l - c / 2.f;
  c += m;
  x += m;

  if (0 <= h and h < 60) {
    return { c, x, 0, a };
  } else if (60 <= h and h < 120) {
    return { x, c, 0, a };
  } else if (120 <= h and h < 180) {
    return { 0, c, x, a };
  } else if (180 <= h and h < 240) {
    return { 0, x, c, a };
  } else if (240 <= h and h < 300) {
    return { x, 0, c, a };
  } else {
    return { c, 0, x, a };
  }
}
}  // namespace flrv::color

template <>
struct flrv::traits::convertor<std_msgs::msg::ColorRGBA, flrv::color::Color>
{
  static auto do_convert(const flrv::color::Color &color) -> std_msgs::msg::ColorRGBA
  { return color; }
};
