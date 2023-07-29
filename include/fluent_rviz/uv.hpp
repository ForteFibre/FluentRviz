#pragma once

#include <visualization_msgs/msg/uv_coordinate.hpp>

#include "fluent_rviz/traits.hpp"

namespace flrv::uv
{
struct UV : public visualization_msgs::msg::UVCoordinate
{
  using visualization_msgs::msg::UVCoordinate::UVCoordinate;

  template <
    typename UVLike,
    traits::Require<
      traits::ConversionDefined<visualization_msgs::msg::UVCoordinate, UVLike>> = nullptr>
  UV(const UVLike &uv)
    : visualization_msgs::msg::UVCoordinate{ traits::convert<visualization_msgs::msg::UVCoordinate>(uv) }
  { }

  UV(const float u, const float v) noexcept
  { this->u = u, this->v = v; }
};
}  // namespace flrv::uv

template <>
struct flrv::traits::Convert<visualization_msgs::msg::UVCoordinate, flrv::uv::UV>
{
  static auto do_convert(const flrv::uv::UV &uv) noexcept -> visualization_msgs::msg::UVCoordinate
  { return uv; }
};