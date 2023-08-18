#pragma once

#include <cstddef>
#include <type_traits>

#include <visualization_msgs/msg/uv_coordinate.hpp>

#include "fluent_rviz/traits/convert.hpp"
#include "fluent_rviz/traits/like_message.hpp"

namespace flrv::uv
{
struct UV : public visualization_msgs::msg::UVCoordinate
{
  UV() = default;

  template <
    typename UVLike,
    std::enable_if_t<traits::like_uv_coordinate<UVLike>, std::nullptr_t> = nullptr>
  UV(const UVLike &uv)
    : visualization_msgs::msg::UVCoordinate{ traits::convert<visualization_msgs::msg::UVCoordinate>(uv) }
  { }

  UV(const float u, const float v) noexcept
  { this->u = u, this->v = v; }
};
}  // namespace flrv::uv

template <>
struct flrv::traits::Converter<visualization_msgs::msg::UVCoordinate, flrv::uv::UV>
{
  static auto do_convert(const flrv::uv::UV &uv) noexcept -> visualization_msgs::msg::UVCoordinate
  { return uv; }
};
