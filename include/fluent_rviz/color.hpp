#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
inline auto RGBA(float r, float g, float b, float a = 1.f) noexcept -> Color
{
  r = std::clamp(r, 0.f, 1.f);
  g = std::clamp(g, 0.f, 1.f);
  b = std::clamp(b, 0.f, 1.f);
  a = std::clamp(a, 0.f, 1.f);
  return { r, g, b, a };
}

[[nodiscard]]
inline auto HSLA(float h, float s, float l, float a = 1.f) noexcept -> Color
{
  h = std::fmod(h, 360.f);
  if (h < 0) {
    h += 360.f;
  }
  s = std::clamp(s, 0.f, 1.f);
  l = std::clamp(l, 0.f, 1.f);
  a = std::clamp(a, 0.f, 1.f);

  auto f = [&](float n) {
    auto k = std::fmod(n + h / 30.f, 12.f);
    return l - s * std::min(l, 1.f - l) * std::max(-1.f, std::min({ k - 3.f, 9.f - k, 1.f }));
  };
  return { f(0.f), f(8.f), f(4.f), a };
}

[[nodiscard]]
inline auto HWBA(float h, float w, float b, float a = 1.f) noexcept -> Color
{
  w = std::clamp(w, 0.f, 1.f);
  b = std::clamp(b, 0.f, 1.f);
  if (w + b > 1.f) {
    auto g = w / (w + b);
    return { g, g, g, a };
  }

  auto [ rr, gg, bb, aa ] = HSLA(h, 1.f, .5f, a);
  auto f = [&](float c) {
    return c * (1.f - w - b) + w;
  };
  return { f(rr), f(gg), f(bb), aa };
}
}  // namespace flrv::color

template <>
struct flrv::traits::Converter<std_msgs::msg::ColorRGBA, flrv::color::Color>
{
  static auto do_convert(const flrv::color::Color &color) -> std_msgs::msg::ColorRGBA
  { return color; }
};
